#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include "rclcpp/rclcpp.hpp"
#include <ur_robot_driver/test_admittance_control.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <urdf/model.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include <thread>

// ROS includes
#include "controller_manager/controller_manager.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <ur_rtde/rtde_control_interface.h>

#include <vector>
#include <cmath>
#include <chrono>
#include "ur_robot_driver/hardware_interface.hpp"


void compute_admittance(const Eigen::MatrixXd& M, const Eigen::MatrixXd& D, const Eigen::MatrixXd& K,
                            Eigen::VectorXd& error, Eigen::VectorXd& q_vec, Eigen::VectorXd& init_q_vec,
                            Eigen::VectorXd& torque_ext, Eigen::VectorXd& q_dot_vec, double& dt);

void getExternalTorque(const Eigen::MatrixXd& M_arm_mat, const Eigen::MatrixXd& Coriolis_arm, const Eigen::VectorXd& G_arm_mat,
                           Eigen::VectorXd& q_dot_vec, Eigen::VectorXd& torque_current, double& dt);


Eigen::MatrixXd KDLFrameToEigenFrame(const KDL::Frame& Frame);

class EffortExtractor : public rclcpp::Node 
{    
public:
    EffortExtractor(): Node("effort_extractor")
    {
        //joint states 받아오는 subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 100, std::bind(&EffortExtractor::joint_states_callback, this, std::placeholders::_1));

        //desired cartesian space position 받는 subscriber
        pos_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/desired_position", 100, std::bind(&EffortExtractor::position_callback, this, std::placeholders::_1));

        //M,D,K,W 값을 실시간으로 바꾸기 위한 subscriber
        param_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/param_sub", 100, std::bind(&EffortExtractor::param_callback, this, std::placeholders::_1));
        
        //velocity control을 위한 q_dot, publisher
        joint_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 100); //velocity controller

        //end_effector force/torque 받아오는 subscriber
        wrench_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/filter_check", 100); 
        torque_ext_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/torque_ext_check", 100); 
        

        //chain부분
        std::string base_link = "base"; 
        std::string end_link = "tool0";
        urdf_filename = ament_index_cpp::get_package_share_directory("ur_description") + "/urdf/ur5e_.urdf"; // 윗 부분이 경로 인식이 잘 안되서 상대경로로 지정
        RCLCPP_INFO(rclcpp::get_logger("logger_name"), "URDF file path: %s", urdf_filename.c_str());
        urdf::Model model;
        if (!model.initFile(urdf_filename)) {
            RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Failed to initialize URDF model.");   // URDF 파일로부터 초기화 실패 시
        }
        //KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(model, tree)) {
            RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Failed to construct KDL tree from URDF.");  // URDF 모델로부터 KDL 트리 생성 실패 시 에러 메시지 출력
            return;
        }
        // KDL::Chain chain;
        if (!tree.getChain(base_link, end_link, chain))
        {
            RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Failed to get chain from tree"); // 트리로부터 체인을 얻지 못한 경우 에러 메시지 출력
        }
        joint_size = chain.getNrOfJoints();
        RCLCPP_INFO(rclcpp::get_logger("joint_size_logger"), "Joint size: %d", joint_size); // joint_size와 n 값이 다르면 chain에서 오류가 발생 -> 이를 확인하기 위함


        //임의의 Mass(M_,M_ori_), Damping(D_), Stiffness(K_)
        double M_ = 5; //M_   낮추면 -> 전체스케일 커짐, 적은힘,진동심함   높이면 -> 전체스케일 작아짐, 큰힘, 진동덜함
        double D_ = 11;
        double K_ = 15;
        M.diagonal() << M_, M_, M_,M_,M_,M_;
        D.diagonal() << D_,D_,D_,D_,D_,D_;
        K.diagonal() << K_,K_,K_,K_,K_,K_; 
        L.diagonal() << 1,1,1,1,1,1; //모멘텀 옵저버의 계수
        W.diagonal() << 10.2312,10.2874,10.2604,7.79396,8.06884,8.06884; //KI - G(q)에서의 q. 위에 K가 있어 W로 대체 
    }
        
    

    //M,D,K,W 값을 실시간으로 바꾸기 위한 subscriber
    void param_callback(const std_msgs::msg::Float64MultiArray::ConstPtr& Param_Data)
    {
        M.diagonal() << Param_Data -> data[0], Param_Data -> data[1], Param_Data -> data[2],Param_Data -> data[3],Param_Data -> data[4],Param_Data -> data[5]; //M_   낮추면 -> 전체스케일 커짐, 적은힘,진동심함, 높이면 -> 전체스케일 작아짐, 큰힘, 진동덜함
        D.diagonal() << Param_Data -> data[6],  Param_Data -> data[7], Param_Data -> data[8],Param_Data -> data[9],Param_Data -> data[10], Param_Data -> data[11];
        K.diagonal() << Param_Data -> data[12], Param_Data -> data[13], Param_Data -> data[14], Param_Data -> data[15],Param_Data -> data[16],Param_Data -> data[17];
        W.diagonal() << Param_Data -> data[18], Param_Data -> data[19], Param_Data -> data[20], Param_Data -> data[21],Param_Data -> data[22],Param_Data -> data[23];
    }
    

    //cartesian space desired position subscribe
    void position_callback(const std_msgs::msg::Float64MultiArray::ConstPtr& Position_Data)
    {
        desire_position << Position_Data -> data[0] , Position_Data -> data[1], Position_Data -> data[2], Position_Data -> data[3], Position_Data -> data[4], Position_Data -> data[5];
    }


    void joint_states_callback(const sensor_msgs::msg::JointState::ConstPtr& JointState_Data)
    {

        JointPosition[0] = JointState_Data->position[5];
        JointPosition[1] = JointState_Data->position[0];
        JointPosition[2] = JointState_Data->position[1];
        JointPosition[3] = JointState_Data->position[2];
        JointPosition[4] = JointState_Data->position[3];
        JointPosition[5] = JointState_Data->position[4];

        JointVelocity[0] = JointState_Data->velocity[5];
        JointVelocity[1] = JointState_Data->velocity[0];
        JointVelocity[2] = JointState_Data->velocity[1];
        JointVelocity[3] = JointState_Data->velocity[2];
        JointVelocity[4] = JointState_Data->velocity[3];
        JointVelocity[5] = JointState_Data->velocity[4];

        JointEffort[0] = JointState_Data->effort[5]; //joint_states에서 받아온 effort == actual current
        JointEffort[1] = JointState_Data->effort[0];
        JointEffort[2] = JointState_Data->effort[1];
        JointEffort[3] = JointState_Data->effort[2];
        JointEffort[4] = JointState_Data->effort[3];
        JointEffort[5] = JointState_Data->effort[4]; 


        //실제 전류를 Low Pass Filter적용한 부분
        //수식 X = (1-a) * X_previous + a * X_now (a가 작으면 필터가 쎄게 입혀지지만 그 만큼 지연이 되고, a가 커지면 필터링은 약해지지만 지연이 줄어듦)
        a1 = 0.5;
        a2 = 0.5;
        a3 = 0.01;
        a4 = 0.5;
        a5 = 0.5;
        a6 = 0.5;

        JointEffort[0] = (1 - a1) * torque_current_1_fold + a1 * JointEffort[0];
        torque_current_1_fold = JointEffort[0];

        JointEffort[1] = (1 - a2)*torque_current_2_fold + a2 * JointEffort[1];
        torque_current_2_fold = JointEffort[1];
                    
        JointEffort[2] = (1 - a3) * torque_current_3_fold + a3 * JointEffort[2];
        torque_current_3_fold = JointEffort[2];

        JointEffort[3] = (1 - a4) * torque_current_4_fold + a4 * JointEffort[3];
        torque_current_4_fold = JointEffort[3];
                    
        JointEffort[4] = (1 - a5)*torque_current_5_fold + a5 * JointEffort[4];
        torque_current_5_fold = JointEffort[4];
                    
        JointEffort[5] = (1 - a6) * torque_current_6_fold + a6 * JointEffort[5];
        torque_current_6_fold = JointEffort[5];
                    
        torque_current << JointEffort[0],JointEffort[1],JointEffort[2],JointEffort[3],JointEffort[4],JointEffort[5];

    }


    void run()  
    {
        rclcpp::Rate loop_rate(100);
        while(rclcpp::ok())
        {
            //M,D,K,W값 실시간으로 바뀌는거 확인용
            // std::cout << "M: " << std::endl << M << std::endl;
            // std::cout << "D: " << std::endl << D << std::endl;
            // std::cout << "K: " << std::endl << K << std::endl;
            // std::cout << "W: " << std::endl << W << std::endl;


            
            KDL::JntArray q(joint_size);
            for (int i = 0; i < joint_size; i++) {
                q(i) = JointPosition[i];
            }
            q_vec = q.data;

            //처음에 한번 초기 포즈 추출
            KDL::JntArray init_q(joint_size);
            if (init_flag == true)
            {

                init_q(0) = init_base; 
                init_q(1) = init_shoulder; 
                init_q(2) = init_elbow; 
                init_q(3) = init_wrist1; 
                init_q(4) = init_wrist2;
                init_q(5) = init_wrist3; 

                //초기 q값을 desire position으로 세팅
                desire_position << init_q(0), init_q(1),init_q(2), init_q(3), init_q(4), init_q(5);
                init_q_vec = init_q.data;

                init_flag = false;
            }


            KDL::JntArray q_dot(joint_size);
            for (int i = 0; i < joint_size; i++) {
                q_dot(i) = JointVelocity[i];
            }
            q_dot_vec = q_dot.data;
            

            KDL::ChainFkSolverPos_recursive fk_solver(chain);
            KDL::ChainJntToJacSolver jac_solver(chain);
            KDL::ChainDynParam dyn_param(chain,KDL::Vector(0.0,0.0,-9.8));

            // Compute the Mass matrix
            KDL::JntSpaceInertiaMatrix M_arm(joint_size);
            dyn_param.JntToMass(q,M_arm);
            M_arm_mat = M_arm.data;

            // Compute the Gravity term
            KDL::JntArray G_arm(joint_size);
            dyn_param.JntToGravity(q,G_arm);
            G_arm_mat = G_arm.data;
            
            // Compute joint position
            now_position << JointPosition[0],JointPosition[1],JointPosition[2],JointPosition[3],JointPosition[4],JointPosition[5];
            

            // Coriolis Matrix 계산
            pinocchio::Model model;
            pinocchio::urdf::buildModel(urdf_filename, model);
            pinocchio::Data data(model);
            Coriolis_arm = pinocchio::computeCoriolisMatrix(model, data, q_vec, q_dot_vec);
            

            //loop 도는데 걸리는 시간 측정
            dt = (rclcpp::Clock{}.now() - last_update_time).seconds();
            last_update_time = rclcpp::Clock{}.now();


            

            //로봇으로부터 sub 받기 전에는 초기 joint_states로 부터 받아온 값들이 0.0인데 시작하면 JointEffort값이 0.0이 아니니까 그때부터 코드 시작
            force_ext_upper_limit_ = 10; // 외력의 upper_limit 
            if (start_flag && JointEffort[0] != compare_value ){ start_flag = false;}

            if ( start_flag == false)
            {
                //////////////////////KI -G(q)계산///////////////////////////////////////////////////////////////////////////////////////
                for (int i = 0; i < joint_size; i++)
                {
                    residual_vec[i] = 0.001 * (-G_arm_mat[i] + W(i,i) * torque_current[i]);
                }



                if(fabs(residual_vec[0]) < dzn_lower_limit_1 || fabs(residual_vec[0]) > force_ext_upper_limit_){residual_vec[0] = 0;}
                else
                {
                    if(residual_vec[0] > 0){residual_vec[0] -= dzn_lower_limit_1;}
                    else{residual_vec[0] += dzn_lower_limit_1;}

                    residual_vec[0] = (1 - coeff_ext_filter_2) * residual_vec_1_fold + coeff_ext_filter_2 * residual_vec[0];
                    residual_vec_1_fold = residual_vec[0];
                }

                if(fabs(residual_vec[1]) < dzn_lower_limit_2 || fabs(residual_vec[1]) > force_ext_upper_limit_){residual_vec[1] = 0;}
                else
                {
                    if(residual_vec[1] > 0){residual_vec[1] -= dzn_lower_limit_2;}
                    else{residual_vec[1] += dzn_lower_limit_2;}
                    
                    residual_vec[1] = (1 - coeff_ext_filter_2)*residual_vec_2_fold + coeff_ext_filter_2 * residual_vec[1];
                    residual_vec_2_fold = residual_vec[1];
                }

                if(fabs(residual_vec[2]) < dzn_lower_limit_3 || fabs(residual_vec[2]) > force_ext_upper_limit_){residual_vec[2] = 0;}
                else
                {
                    if(residual_vec[2] > 0){residual_vec[2] -= dzn_lower_limit_3;}
                    else{residual_vec[2] += dzn_lower_limit_3;}
                        
            
                    residual_vec[2] = (1 - 0.5) * residual_vec_3_fold + 0.5 * residual_vec[2];
                    residual_vec_3_fold = residual_vec[2];
                }
                    
                if(fabs(residual_vec[3]) < dzn_lower_limit_4 || fabs(residual_vec[3]) > force_ext_upper_limit_){residual_vec[3] = 0;}
                else
                {
                    if(residual_vec[3] > 0){residual_vec[3] -= dzn_lower_limit_4;}
                    else{residual_vec[3] += dzn_lower_limit_4;}

                    residual_vec[3] = (1 - coeff_ext_filter_2) * residual_vec_4_fold + coeff_ext_filter_2 * residual_vec[3];
                    residual_vec_4_fold = residual_vec[3];
                }

                if(fabs(residual_vec[4]) < dzn_lower_limit_5 || fabs(residual_vec[4]) > force_ext_upper_limit_){residual_vec[4] = 0;}
                else
                {
                    if(residual_vec[4] > 0){residual_vec[4] -= dzn_lower_limit_5;}
                    else{residual_vec[4] += dzn_lower_limit_5;}
                        
                    residual_vec[4] = (1 - 0.5)*residual_vec_5_fold + 0.5 * residual_vec[4];
                    residual_vec_5_fold = residual_vec[4];
                }

                if(fabs(residual_vec[5]) < dzn_lower_limit_6 || fabs(residual_vec[5]) > force_ext_upper_limit_){residual_vec[5] = 0;}
                else
                {
                    if(residual_vec[5] > 0){residual_vec[5] -= dzn_lower_limit_6;}
                    else{residual_vec[5] += dzn_lower_limit_6;}
                        
            
                    residual_vec[5] = (1 - coeff_ext_filter_2) * residual_vec_6_fold + coeff_ext_filter_2 * residual_vec[5];
                    residual_vec_6_fold = residual_vec[5];
                }
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






                ///////////////////////Momentum observer 사용////////////////////////////////////////////////////////////////////////////
                // getExternalTorque(M_arm_mat, Coriolis_arm, G_arm_mat,
                //               q_dot_vec, torque_current, dt);
                //     // momentum observer를 통해 구한 외부 torque에 LPF씌우는 과정


                    

                // if(fabs(torque_ext[0]) < torque_ext_lower_limit_1 || fabs(torque_ext[0]) > force_ext_upper_limit_){torque_ext[0] = 0;}
                // else
                // {
                //     if(torque_ext[0] > 0){torque_ext[0] -= torque_ext_lower_limit_1;}
                //     else{torque_ext[0] += torque_ext_lower_limit_1;}

                //     torque_ext[0] = (1 - coeff_ext_filter) * torque_ext_1_fold + coeff_ext_filter * torque_ext[0];
                //     torque_ext_1_fold = torque_ext[0];
                // }

                // if(fabs(torque_ext[1]) < torque_ext_lower_limit_2 || fabs(torque_ext[1]) > force_ext_upper_limit_){torque_ext[1] = 0;}
                // else
                // {
                //     if(torque_ext[1] > 0){torque_ext[1] -= torque_ext_lower_limit_2;}
                //     else{torque_ext[1] += torque_ext_lower_limit_2;}
                        
                //     torque_ext[1] = (1 - coeff_ext_filter)*torque_ext_2_fold + coeff_ext_filter * torque_ext[1];
                //     torque_ext_2_fold = torque_ext[1];
                // }

                // if(fabs(torque_ext[2]) < torque_ext_lower_limit_3 || fabs(torque_ext[2]) > force_ext_upper_limit_){torque_ext[2] = 0;}
                // else
                // {
                //     if(torque_ext[2] > 0){torque_ext[2] -= torque_ext_lower_limit_3;}
                //     else{torque_ext[2] += torque_ext_lower_limit_3;}
                        
            
                //     torque_ext[2] = (1 - coeff_ext_filter) * torque_ext_3_fold + coeff_ext_filter * torque_ext[2];
                //     torque_ext_3_fold = torque_ext[2];
                // }
                    
                // if(fabs(torque_ext[3]) < torque_ext_lower_limit_4 || fabs(torque_ext[3]) > force_ext_upper_limit_){torque_ext[3] = 0;}
                // else
                // {
                //     if(torque_ext[3] > 0){torque_ext[3] -= torque_ext_lower_limit_4;}
                //     else{torque_ext[3] += torque_ext_lower_limit_4;}

                //     torque_ext[3] = (1 - coeff_ext_filter) * torque_ext_4_fold + coeff_ext_filter * torque_ext[3];
                //     torque_ext_4_fold = torque_ext[3];
                // }

                // if(fabs(torque_ext[4]) < torque_ext_lower_limit_5 || fabs(torque_ext[4]) > force_ext_upper_limit_){torque_ext[4] = 0;}
                // else
                // {
                //     if(torque_ext[4] > 0){torque_ext[4] -= torque_ext_lower_limit_5;}
                //     else{torque_ext[4] += torque_ext_lower_limit_5;}
                        
                //     torque_ext[4] = (1 - coeff_ext_filter)*torque_ext_5_fold + coeff_ext_filter * torque_ext[4];
                //     torque_ext_5_fold = torque_ext[4];
                // }

                // if(fabs(torque_ext[5]) < torque_ext_lower_limit_6 || fabs(torque_ext[5]) > force_ext_upper_limit_){torque_ext[5] = 0;}
                // else
                // {
                //     if(torque_ext[5] > 0){torque_ext[5] -= torque_ext_lower_limit_6;}
                //     else{torque_ext[5] += torque_ext_lower_limit_6;}
                        
            
                //     torque_ext[5] = (1 - coeff_ext_filter) * torque_ext_6_fold + coeff_ext_filter * torque_ext[5];
                //     torque_ext_6_fold = torque_ext[5];
                // }
                // torque_current << JointEffort[0],JointEffort[1],JointEffort[2],JointEffort[3],JointEffort[4],JointEffort[5];// ?? 이건 왜 있는거지 무슨 용이지
                /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




                
                ////////////////////////////////////////////3번 조인트 부분 적 테스트용//////////////////////////////////////////////////////////////////////////////////////////////////

                // if((residual_vec[2]) < 0.0005 && (residual_vec[2] > -0.0225)){residual_vec[2] = 0;}
                // else
                // {
                //     if(residual_vec[2] > 0.0005){residual_vec[2] -= 0.0005;}
                //     else if (residual_vec[2] < -0.0225) {residual_vec[2] += 0.0225;}
                    
        
                //     residual_vec[2] = (1 - 0.5) * residual_vec_3_fold + 0.5 * residual_vec[2];
                //     residual_vec_3_fold = residual_vec[2];
                // }



                // if(fabs(residual_vec[2]) < 0.01 || fabs(residual_vec[2]) > force_ext_upper_limit_){residual_vec[2] = 0;}
                // else
                // {
                //     if(residual_vec[2] > 0){residual_vec[2] -= 0.01;}
                //     else{residual_vec[2] += 0.01;}
                    
        
                //     residual_vec[2] = (1 - 0.5) * residual_vec_3_fold + 0.5 * residual_vec[2];
                //     residual_vec_3_fold = residual_vec[2];
                // }
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


                compute_admittance(M,D,K,
                                error, q_vec, init_q_vec,
                                residual_vec, q_dot_vec, dt);
                
                
                //vel_msg 생성
                std_msgs::msg::Float64MultiArray vel_msg;



                //velocity control를 위한 desire qd가 너무 작으면 0으로 처리
                for(int i =0 ; i< joint_size; i++)
                {
                    if(fabs(desire_q_dot[i]) < 0.0001){desire_q_dot[i] = 0;}
                }

                
                vel_msg.data = {desire_q_dot[0],desire_q_dot[1],desire_q_dot[2],desire_q_dot[3],desire_q_dot[4],desire_q_dot[5]};

                // vel_msg.data = {0,0,0,0,0,0};
                joint_vel_pub->publish(vel_msg);
                

                //plotjuggler로 확인하기 위한 pub
                std_msgs::msg::Float64MultiArray wrench_msg;
                wrench_msg.data = {residual_vec[0],0.1 * residual_vec[1], residual_vec[2],residual_vec[3], residual_vec[4],residual_vec[5],
                                    torque_current[0],torque_current[1],torque_current[2],torque_current[3],torque_current[4],torque_current[5],
                                    torque_ext[0],torque_ext[1],torque_ext[2],torque_ext[3],torque_ext[4],torque_ext[5],
                                    error[0],error[1],error[2],error[3],error[4],error[5]};
                wrench_pub->publish(wrench_msg);


                //plotjuggler로 확인하기 위한 pub
                std_msgs::msg::Float64MultiArray torque_ext_msg;
                torque_ext_msg.data = {torque_ext[0],torque_ext[1],torque_ext[2],torque_ext[3],torque_ext[4],torque_ext[5]};

                torque_ext_pub->publish(torque_ext_msg);
            }
            loop_rate.sleep();
        }
    }

    void compute_admittance(const Eigen::MatrixXd& M, const Eigen::MatrixXd& D, const Eigen::MatrixXd& K,
                            Eigen::VectorXd& error, Eigen::VectorXd& q_vec, Eigen::VectorXd& init_q_vec,
                            Eigen::VectorXd& torque_ext, Eigen::VectorXd& q_dot_vec, double& dt)
    {
        error = q_vec - init_q_vec;

        Eigen::VectorXd coupling_wrench = Eigen::VectorXd::Zero(6);

        coupling_wrench = D * q_dot_vec + K * error;
        // q_ddot_vec = M.inverse() * (-coupling_wrench - W * torque_current);
        q_ddot_vec = M.inverse() * (-coupling_wrench - 1000 * torque_ext);
        // q_ddot_vec = M.inverse() * (-coupling_wrench );

        desire_q_dot += q_ddot_vec * dt;
    }


    //Momentum observer (현재는 사용 x)
    void getExternalTorque(const Eigen::MatrixXd& M_arm_mat, const Eigen::MatrixXd& Coriolis_arm, const Eigen::VectorXd& G_arm_mat,
                           Eigen::VectorXd& q_dot_vec, Eigen::VectorXd& torque_current, double& dt)
    {
        p_q = M_arm_mat * q_dot_vec; // M * qd
        beta = -Coriolis_arm.transpose() * q_dot_vec + G_arm_mat; 



        if(init_ext_flag == false)
        {   
            sum += 0.5 * dt *(sum_0 - beta + W * torque_current + r) ;
            sum_0 = -beta + W * torque_current + r;


            r = L * (p_q -p_q_0 - sum );

            torque_ext = 0.1 * r;
            
        }

        //로봇에게 sub하기 시작되면 시작
        if (torque_current[0] != compare_value && q_dot_vec[0] != compare_value && init_ext_flag == true)
        {
            p_q_0 = M_arm_mat * q_dot_vec; 
            r = Eigen::VectorXd::Zero(6);
            sum_0 = W *  torque_current - beta + r;

            torque_ext_offset = sum_0;

            sum += 0.5 * dt *(sum_0 - beta + W * torque_current + r);   
            sum_0 = -beta + W * torque_current;

            r = L * (p_q -p_q_0 - sum);
            init_ext_flag =false;
        }
    } 

private:
    bool init_flag = true; //처음에 한번 초기 포즈 추출용 flag
    bool init_ext_flag =true; //Momentum observer용
    bool start_flag =true; 

    float force_ext_upper_limit_ = 10; // 센서의 upper_limit 

    //Momentum observer로 측정한 외부 torque에 씌우는 LPF Deadzone 변수
    double coeff_ext_filter = 0.5; // filter 계수
    float torque_ext_lower_limit_1 = 0.5;  // 1번 조인트
    float torque_ext_lower_limit_2 = 0.5;  // 2번조인트
    float torque_ext_lower_limit_3 = 0.5;  // 3번 조인트
    float torque_ext_lower_limit_4 = 0.35; // 4번조인트
    float torque_ext_lower_limit_5 = 0.35; // 5번조인트
    float torque_ext_lower_limit_6 = 0.35; // 6번 조인트
    double torque_ext_1_fold = 0.0; //1번 조인트 이전 외력
    double torque_ext_2_fold = 0.0; //2번 조인트 이전 외력
    double torque_ext_3_fold = 0.0; //3번 조인트 이전 외력
    double torque_ext_4_fold = 0.0; //4번 조인트 이전 외력
    double torque_ext_5_fold = 0.0; //5번 조인트 이전 외력
    double torque_ext_6_fold = 0.0; //6번 조인트 이전 외력

    // KI - G(q) filter
    double coeff_ext_filter_2 = 0.2; // ki-g filter 계수
    float dzn_lower_limit_1 = 0.1; // 1번 조인트
    float dzn_lower_limit_2 = 0.1; // 2번조인트
    float dzn_lower_limit_3 = 0.6; // 3번조인트
    float dzn_lower_limit_4 = 0.2; // 4번 조인트
    float dzn_lower_limit_5 = 0.3; // 5번 조인트
    float dzn_lower_limit_6 = 0.1; // 6번 조인트
    double residual_vec_1_fold = 0.0; //1번 조인트 이전 외력 
    double residual_vec_2_fold = 0.0; //2번 조인트 이전 외력
    double residual_vec_3_fold = 0.0; //3번 조인트 이전 외력
    double residual_vec_4_fold = 0.0; //4번 조인트 이전 외력
    double residual_vec_5_fold = 0.0; //5번 조인트 이전 외력
    double residual_vec_6_fold = 0.0; //6번 조인트 이전 외력

    //actual current LPF 변수
    double a1 = 0.0;
    double a2 = 0.0;
    double a3 = 0.0;
    double a4 = 0.0;
    double a5 = 0.0;
    double a6 = 0.0;
    double torque_current_1_fold = 0.0; //1번 조인트 이전 current
    double torque_current_2_fold = 0.0; //2번 조인트 이전 current
    double torque_current_3_fold = 0.0; //3번 조인트 이전 current
    double torque_current_4_fold = 0.0; //4번 조인트 이전 current
    double torque_current_5_fold = 0.0; //5번 조인트 이전 current
    double torque_current_6_fold = 0.0; //6번 조인트 이전 current

    //로봇 sub 시작했는지 확인할 때 쓰는 값
    double compare_value = 0.0;

    int joint_size;
    rclcpp::Time last_update_time = rclcpp::Clock{}.now();
    double dt = 0.0;


    //admittance control 변수
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd L = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(6,6);

    Eigen::VectorXd error =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd now_position =Eigen::VectorXd::Zero(3);
    Eigen::VectorXd desire_position =Eigen::VectorXd::Zero(3);


    Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd init_q_vec = Eigen::VectorXd::Zero(6);


    Eigen::VectorXd q_dot_vec = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_ddot_vec = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd desire_q_dot =Eigen::VectorXd::Zero(6);

    Eigen::MatrixXd M_arm_mat = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd Coriolis_arm = Eigen::MatrixXd::Zero(6,6);
    Eigen::VectorXd G_arm_mat =Eigen::VectorXd::Zero(6);


    //Momentum observer 변수
    Eigen::VectorXd p_q =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd p_q_0 =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd beta =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd r =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd torque_ext =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd torque_ext_offset =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd residual_vec =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd sum =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd sum_0 =Eigen::VectorXd::Zero(6);


    Eigen::VectorXd torque_current =Eigen::VectorXd::Zero(6);
    

    //init joint pose
    float init_base = 5.32325; //305도
    float init_shoulder  = -1.22173; //-70도
    float init_elbow = -2.00713; //-115도
    float init_wrist1 = -2.80998; //-161도
    float init_wrist2 = 1.53589; //88도
    float init_wrist3 = 0.0;  //0도

    float JointPosition[6] = {init_base, init_shoulder,  init_elbow, init_wrist1, init_wrist2, init_wrist3};
    float JointVelocity[6] = {0.0};// initial pos error
    float JointEffort[6] = {0.0};// initial pos error

    std::string base_link;
    std::string end_link;
    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_filename;


    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_ext_pub; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wrench_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub; 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr param_sub;


};


Eigen::MatrixXd KDLFrameToEigenFrame(const KDL::Frame& Frame)
{
    Eigen::MatrixXd TF(4, 4);
    for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                TF(i,j) = Frame(i,j);
            }
        }

    return TF;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("Node ON"), "@@@@@@@@@@@Mobile impedance node START@@@@@@@@@@@");

  auto effort_node = std::make_shared<EffortExtractor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(effort_node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  effort_node->run();
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}