// #include <pinocchio/parsers/urdf.hpp>
// #include <pinocchio/algorithm/rnea.hpp>
// #include <pinocchio/algorithm/crba.hpp>

#include "rclcpp/rclcpp.hpp"
#include <manipulator_admittance/admittance_control.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <urdf/model.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "std_msgs/msg/float64_multi_array.hpp" 
#include <thread>
// ROS includes
#include "controller_manager/controller_manager.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <linkpose_msgs/msg/link_pose.hpp>   
#include <vector>
#include <cmath>
#include <chrono>


void compute_admittance(const Eigen::MatrixXd& M, const Eigen::MatrixXd& D, const Eigen::MatrixXd& K,
                            Eigen::VectorXd& current_position, Eigen::VectorXd& desire_position,
                            Eigen::Quaterniond& current_quat, Eigen::Quaterniond& desire_quat, Eigen::VectorXd& ForceTorque,
                            Eigen::VectorXd& desire_adm_vel, Eigen::VectorXd& desire_adm_acc, double& dt);



Eigen::MatrixXd KDLFrameToEigenFrame(const KDL::Frame& Frame);
class EffortExtractor : public rclcpp::Node
{
    
public:
    EffortExtractor(): Node("effort_extractor")
    {
        //joint states 받아오는 subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 100, std::bind(&EffortExtractor::joint_states_callback, this, std::placeholders::_1));

        //end_effector force/torque 받아오는 subscriber
        wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/wrench", 100, std::bind(&EffortExtractor::force_torque_callback, this, std::placeholders::_1));

        //desired cartesian space position 받는 subscriber
        pos_sub = this->create_subscription<geometry_msgs::msg::Pose>(
            "/desired_position", 100, std::bind(&EffortExtractor::position_callback, this, std::placeholders::_1));

        mobile_pos_sub = this->create_subscription<linkpose_msgs::msg::LinkPose>(
            "/LinkPose_mobile_ur5e_base_link", 100, std::bind(&EffortExtractor::mobile_pose_callback, this, std::placeholders::_1));
        //velocity control을 위한 q_dot, publisher
        joint_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ur5e_controller/commands", 100); //mani velocity controller
        lift_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lift_controller/commands", 100); //lift velocity controller
        //값을 확인하기 위한 publisher
        wrench_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/filter_check", 100); //velocity controller

        


        //chain부분
        std::string base_link = "base_link"; 
        std::string end_link = "tool0";
        urdf_filename = ament_index_cpp::get_package_share_directory("mobis_description") + "/urdf/mobile_ur5e.urdf"; // 윗 부분이 경로 인식이 잘 안되서 상대경로로 지정
        RCLCPP_INFO(rclcpp::get_logger("logger_name"), "URDF file path: %s", urdf_filename.c_str());
        urdf::Model model;
        if (!model.initFile(urdf_filename)) {
            RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Failed to initialize URDF model.");   // URDF 파일로부터 초기화 실패 시 에러 메시지 출력
            return;
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
        double M_ = 5;
        double M_ori_ = 0.3;
        double D_ = 16;
        double K_ = 40;
        double W_ = 1;
        // M
        M.diagonal() << M_, M_, M_,M_ori_,M_ori_,M_ori_;
        D.diagonal() << D_,D_,D_,0.2 *D_,0.2 *D_,0.2 *D_;
        K.diagonal() << K_,K_,K_,0.2 *K_,0.2 *K_,0.2 *K_; 
        W.diagonal() << W_,W_,W_,W_,W_,W_,W_,W_,0.1*W_,0.1*W_,0.1*W_,0.1*W_,W_,W_,W_,W_,W_,W_; //x1,y1,x2,y2, ... ,z1,z2,z3,z4,q1, ... q6;

        Jacobian_mobile_inv <<   1,  0,  0,   0,  0, -Ly,  //ref:mobile_base   
                                 0,  1,  0,   0,  0,  Lx,
                                 1,  0,  0,   0,  0,  Ly,
                                 0,  1,  0,   0,  0,  Lx,
                                 1,  0,  0,   0,  0,  Ly,
                                 0,  1,  0,   0,  0, -Lx,
                                 1,  0,  0,   0,  0, -Ly,
                                 0,  1,  0,   0,  0, -Lx,
                                 0,  0, -1,  Ly, Lx,   0,  
                                 0,  0, -1,- Ly, Lx,   0,  
                                 0,  0, -1,- Ly,-Lx,   0,  
                                 0,  0, -1,  Ly,-Lx,   0;  
        
        Jacobian_mobile <<  0.25,0,0.25,0,0.25,0,0.25,0,0,0,0,0, //ref :mobile_base     x
                            0,0.25,0,0.25,0,0.25,0,0.25,0,0,0,0,
                            0,0,0,0,0,0,0,0, -0.25   , -0.25    , -0.25    , -0.25,
                            0,0,0,0,0,0,0,0,1/(4*Ly),-1/(4*Ly),-1/(4*Ly),1/(4*Ly),                    //f_r,f_l,r_l,r_r
                            0,0,0,0,0,0,0,0,1/(4*Lx),1/(4*Lx),-1/(4*Lx),-1/(4*Lx), 
                            1/(4*Ly),0,-1/(4*Ly),0,-1/(4*Ly),0,1/(4*Ly),0,0,0,0,0;
 
    }


    //cartesian space desired position subscribe
    void position_callback(const geometry_msgs::msg::Pose::ConstPtr& Position_Data)
    {
        desire_position(0) = Position_Data->position.x;
        desire_position(1) = Position_Data->position.y;
        desire_position(2) = Position_Data->position.z;
        desire_quat.x() = Position_Data->orientation.x;
        desire_quat.y() = Position_Data->orientation.y;
        desire_quat.z() = Position_Data->orientation.z;
        desire_quat.w() = Position_Data->orientation.w;
    }

    void mobile_pose_callback(const linkpose_msgs::msg::LinkPose Pose_Data)
    {
    
        moible_position(0) = Pose_Data.x;
        moible_position(1) = Pose_Data.y;
        moible_position(2) = Pose_Data.z;
        moible_quat.x() = Pose_Data.qx;
        moible_quat.y() = Pose_Data.qy;
        moible_quat.z() = Pose_Data.qz;
        moible_quat.w() = Pose_Data.qw;

        wRm = moible_quat.normalized().toRotationMatrix();
        
        wRm_e.block(0,0,3,3) = wRm;
        wRm_e.block(3,3,3,3) = wRm;

        moible_TF.block(0,0,3,3) = wRm;
        moible_TF.block(0,3,3,1) = moible_position;
        
        //std::cout<<"mobile TF: "<<moible_TF<<std::endl;

    }
    

    void joint_states_callback(const sensor_msgs::msg::JointState::ConstPtr& JointState_Data)
    {
        //q
        JointPosition[0] = JointState_Data->position[0];
        JointPosition[1] = JointState_Data->position[1];
        JointPosition[2] = JointState_Data->position[2];
        JointPosition[3] = JointState_Data->position[3];
        JointPosition[4] = JointState_Data->position[13];
        JointPosition[5] = JointState_Data->position[16];

        //q_dot
        JointVelocity[0] = JointState_Data->velocity[0];
        JointVelocity[1] = JointState_Data->velocity[1];
        JointVelocity[2] = JointState_Data->velocity[2];
        JointVelocity[3] = JointState_Data->velocity[3];
        JointVelocity[4] = JointState_Data->velocity[13];
        JointVelocity[5] = JointState_Data->velocity[16];
    }


    void force_torque_callback(const geometry_msgs::msg::WrenchStamped::ConstPtr& ForceTorque_Data)
    {   
        //end frame 기준 force, torque 받아오는 부분
        ForceTorque[0] = ForceTorque_Data->wrench.force.x;
        ForceTorque[1] = ForceTorque_Data->wrench.force.y;
        ForceTorque[2] = ForceTorque_Data->wrench.force.z;
        ForceTorque[3] = ForceTorque_Data->wrench.torque.x;
        ForceTorque[4] = ForceTorque_Data->wrench.torque.y;
        ForceTorque[5] = ForceTorque_Data->wrench.torque.z;

        
        //LOW-PASS FILTER 적용
        force_thres_lower_limit_ = 7.0; // 임의의 lower_limit 값
        force_thres_upper_limit_ = 160; // 센서의 upper_limit 


        //deadzone으로 낮은 값은 짤라내고, 큰 값은 lower_limit만큼 빼줘서 0부분에서 이어지게 함
        //torque는 비교적 안정적이라서 force x,y,z만 deadzone과 LPF 사용
        if(fabs(ForceTorque[0]) < force_thres_lower_limit_ || fabs(ForceTorque[0]) > force_thres_upper_limit_){ForceTorque[0] = 0;}
        else
        {
          if(ForceTorque[0] > 0){ForceTorque[0] -= force_thres_lower_limit_;}
          else{ForceTorque[0] += force_thres_lower_limit_;}
          
          //여기가 Low Pass Filter부분
          //수식 X = (1-a) * X_previous + a * X_now (a가 작으면 필터가 쎄게 입혀지지만 그 만큼 지연이 되고, a가 커지면 필터링은 약해지지만 지연이 줄어듦)
          ForceTorque[0] = (1 - coeff_filter) * force_x_fold + coeff_filter * ForceTorque[0];
          force_x_fold = ForceTorque[0];
        }
        if(fabs(ForceTorque[1]) < force_thres_lower_limit_ || fabs(ForceTorque[1]) > force_thres_upper_limit_){ForceTorque[1] = 0;}
        else
        {
          if(ForceTorque[1] > 0){ForceTorque[1] -= force_thres_lower_limit_;}
          else{ForceTorque[1] += force_thres_lower_limit_;}
          
          ForceTorque[1] = (1 - coeff_filter)*force_y_fold + coeff_filter * ForceTorque[1];
          force_y_fold = ForceTorque[1];
        }
        if(fabs(ForceTorque[2]) < force_thres_lower_limit_ || fabs(ForceTorque[2]) > force_thres_upper_limit_){ForceTorque[2] = 0;}
        else
        {
          if(ForceTorque[2] > 0){ForceTorque[2] -= force_thres_lower_limit_;}
          else{ForceTorque[2] += force_thres_lower_limit_;}
          
          ForceTorque[2] = (1 - coeff_filter) * force_z_fold + coeff_filter * ForceTorque[2];
          force_z_fold = ForceTorque[2];
        }
    }


    void run()  
    {
        rclcpp::Rate loop_rate(100);
        while(rclcpp::ok())
        {
            KDL::JntArray q(joint_size);
            for (int i = 0; i < joint_size; i++) {
                q(i) = JointPosition[i];
            }
            q_vec = q.data;


            //처음에 한번 초기 포즈에 대한 rotation matrix 추출
            KDL::JntArray init_q(joint_size);
            if (init_flag == true){

                init_q(0) = init_base; 
                init_q(1) = init_shoulder; 
                init_q(2) = init_elbow; 
                init_q(3) = init_wrist1; 
                init_q(4) = init_wrist2;
                init_q(5) = init_wrist3; 

                //init_q에 대한 rotation matrix와 cartesian space position을 구하는 과정
                KDL::Frame init_end_effector_pose;
                KDL::ChainFkSolverPos_recursive init_fk_solver(chain);
                //forward kinematics로 초기 postion 구함(desired position)
                init_fk_solver.JntToCart(init_q, init_end_effector_pose);

                //초기 위치의 transformation matrix
                end_effector_tmp_TF = KDLFrameToEigenFrame(init_end_effector_pose);
                init_end_effector_TF = moible_TF*end_effector_tmp_TF;                      
                //desired rotation matrix 구하기
                Eigen::Matrix3d Rd =Eigen::MatrixXd::Identity(3,3);
                Rd = init_end_effector_TF.block<3, 3>(0, 0);
                
                //desire_quat 계산
                desire_quat = Eigen::Quaterniond(Rd);

                // x_d value
                Eigen::VectorXd init_position = init_end_effector_TF.block(0,3,3,1); 
                desire_position << init_position(0), init_position(1), init_position(2) + 0.5;

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

            
            // Compute End effector position
            fk_solver.JntToCart(q, end_effector_pose);
            
            
            //Compute now transformation matrix
            end_effector_tmp_TF = KDLFrameToEigenFrame(end_effector_pose);
            end_effector_TF = moible_TF*end_effector_tmp_TF;
            
            current_position = end_effector_TF.block(0,3,3,1);
            
            //현재end-effector rotation matrix 얻기
            Eigen::Matrix3d Re =Eigen::MatrixXd::Identity(3,3);
            Re = end_effector_TF.block<3, 3>(0, 0);

            //end-effector frame의 Force/torque 센서값을 world frame기준으로 바꿔 줌
            ForceTorque.head(3) = Re *  ForceTorque.head(3);
            ForceTorque.tail(3) = Re *  ForceTorque.tail(3);

            //현재 quat 계산
            current_quat = Eigen::Quaterniond(Re);

            //loop 도는데 걸리는 시간 측정
            dt = (rclcpp::Clock{}.now() - last_update_time).seconds();
            last_update_time = rclcpp::Clock{}.now();


            compute_admittance(M,D,K,
                            current_position, desire_position,
                            current_quat, desire_quat, ForceTorque,
                            desire_adm_vel, desire_adm_acc, dt);
            
            //geometric jacobian
            KDL::Jacobian J_arm(joint_size); 
            jac_solver.JntToJac(q, J_arm);
            Jacobian_arm = wRm_e*J_arm.data;
            std::cout<<"j_arm: "<<std::endl<<Jacobian_arm<<std::endl;

            Jacobian_whole.block(0,0,6,12) = wRm_e*Jacobian_mobile;
            Jacobian_whole.block(0,12,6,6) = Jacobian_arm;
            Eigen::MatrixXd J = Jacobian_whole;

            //DPI 적용
            //Jacobian_arm_DPI_inverse = Jacobian_arm.transpose()*(Jacobian_arm *Jacobian_arm.transpose() + 0.01*Eigen::MatrixXd::Identity(6,6)).inverse();
            //std::cout<<"j DPI_inverse"<<std::endl<<Jacobian_arm_DPI_inverse<<std::endl;
            //weighted pseudo-inverse
            
            Jw_inverse = W.inverse()*J.transpose()*(J*W.inverse()*J.transpose()).inverse();
            std::cout<<"jW_inverse: "<<std::endl<<Jw_inverse<<std::endl;


            //velocity control을 위한 q_dot
            desire_q_dot = Jw_inverse * desire_adm_vel;

            //vel_msg 생성
            std_msgs::msg::Float64MultiArray vel_msg;
            std_msgs::msg::Float64MultiArray lift_msg;
            vel_msg.data = {desire_q_dot[12],desire_q_dot[13],desire_q_dot[14],desire_q_dot[15],desire_q_dot[16],desire_q_dot[17]};
            lift_msg.data = {desire_q_dot[9],desire_q_dot[8],desire_q_dot[10],desire_q_dot[11]}; //f_l,f_r,r_l,r_r
            joint_vel_pub->publish(vel_msg);
            lift_vel_pub->publish(lift_msg);
            std::cout<<"d_q_dot_lift: "<<desire_q_dot[8]<<", "<<desire_q_dot[9]<<", "<<desire_q_dot[10]<<", "<<desire_q_dot[11]<<", "<<std::endl;
            
            std_msgs::msg::Float64MultiArray wrench_msg;





            
            wrench_msg.data = {ForceTorque[0],ForceTorque[1], ForceTorque[2],ForceTorque[3],ForceTorque[4],ForceTorque[5],
                            desire_q_dot[0],desire_q_dot[1],desire_q_dot[2],desire_q_dot[3],desire_q_dot[4],desire_q_dot[5]};
            wrench_pub->publish(wrench_msg);
            
            std::cout << "current_position" << std::endl << current_position.transpose() << std::endl;
            std::cout << "current_quat" << std::endl << current_quat.coeffs() << std::endl;

            loop_rate.sleep();
        }
    }

    void compute_admittance(const Eigen::MatrixXd& M, const Eigen::MatrixXd& D, const Eigen::MatrixXd& K,
                             Eigen::VectorXd& current_position, Eigen::VectorXd& desire_position,
                            Eigen::Quaterniond& current_quat, Eigen::Quaterniond& desire_quat, Eigen::VectorXd& ForceTorque,
                            Eigen::VectorXd& desire_adm_vel, Eigen::VectorXd& desire_adm_acc, double& dt)
    {
        error.head(3) = current_position - desire_position;
        if(desire_quat.coeffs().dot(current_quat.coeffs()) < 0.0)
        {
            current_quat.coeffs() << -current_quat.coeffs();
        }
        Eigen::Quaterniond quat_rot_err (current_quat * desire_quat.inverse());
        if(quat_rot_err.coeffs().norm() > 1e-3)
        {
          quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
        }

        Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
        error.tail(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();


        Eigen::VectorXd coupling_wrench = Eigen::VectorXd::Zero(6);
        coupling_wrench = D * desire_adm_vel + K * error;
        desire_adm_acc = M.inverse() * (-coupling_wrench + ForceTorque);


        double a_acc_norm = (desire_adm_acc.segment(0, 3)).norm();
        double arm_max_acc_ = 1.0;
        if (a_acc_norm > arm_max_acc_) 
        {
          desire_adm_acc.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
        }
        desire_adm_vel += desire_adm_acc * dt;
    }

private:
    bool init_flag = true;
    int joint_size;

    rclcpp::Time last_update_time = rclcpp::Clock{}.now();
    double dt = 0.0;

    // F/T sensor LOW-PASS FILTER 적용
    float force_thres_lower_limit_ = 7.0; // 임의의 lower_limit 값
    float force_thres_upper_limit_ = 160; // 센서의 upper_limit 
    double coeff_filter = 0.75; 
    double force_x_fold = 0.0;
    double force_y_fold = 0.0;
    double force_z_fold = 0.0;

    
    
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Aw = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(18,18);
    Eigen::VectorXd error =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd current_position =Eigen::VectorXd::Zero(3);
    Eigen::VectorXd desire_position =Eigen::VectorXd::Zero(3);
    Eigen::VectorXd moible_position =Eigen::VectorXd::Zero(3);
    Eigen::VectorXd q_vec = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_dot_vec = Eigen::VectorXd::Zero(6);
    Eigen::Quaterniond current_quat;
    Eigen::Quaterniond desire_quat;
    Eigen::Quaterniond moible_quat;
    Eigen::VectorXd desire_adm_acc =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd desire_adm_vel =Eigen::VectorXd::Zero(6);
    Eigen::VectorXd desire_q_dot =Eigen::VectorXd::Zero(18);
    Eigen::MatrixXd Jacobian_arm = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Jacobian_mobile = Eigen::MatrixXd::Identity(6,12);
    Eigen::MatrixXd Jacobian_mobile_inv = Eigen::MatrixXd::Identity(12,6);
    Eigen::MatrixXd Jacobian_whole = Eigen::MatrixXd::Zero(6,18);
    Eigen::MatrixXd Jw_inverse = Eigen::MatrixXd::Zero(18,6);
    Eigen::MatrixXd Jacobian_arm_DPI_inverse = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd end_effector_tmp_TF;
    Eigen::MatrixXd end_effector_TF;
    Eigen::MatrixXd moible_TF = Eigen::MatrixXd::Identity(4,4);
    Eigen::Matrix3d wRm = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd wRm_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd init_end_effector_TF;
    KDL::Frame end_effector_pose;



    //init joint state
    float init_base = 0.0; //270
    float init_shoulder  = -1.0; //-90도
    float init_elbow = 1.0; //-90도
    float init_wrist1 = 0.0; //-180
    float init_wrist2 = 0.0;
    float init_wrist3 = 0.0;  


    float JointPosition[6] = {init_base, init_shoulder,  init_elbow, init_wrist1, init_wrist2, init_wrist3};
    float JointVelocity[6] = {0.0};// initial pos error
    Eigen::VectorXd ForceTorque = Eigen::VectorXd::Zero(6);
    double Lx = 67.0/2.0;
    double Ly = 46.0/2.0;


    std::string base_link;
    std::string end_link;
    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_filename;


    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wrench_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub; //velocity controller
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lift_vel_pub; //velocity controller
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pos_sub;
    rclcpp::Subscription<linkpose_msgs::msg::LinkPose>::SharedPtr mobile_pos_sub;
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
  RCLCPP_INFO(rclcpp::get_logger("Node ON"), "@@@@@@@@@@@Task sapce admittance node START@@@@@@@@@@@");

  auto effort_node = std::make_shared<EffortExtractor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(effort_node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  effort_node->run();
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}