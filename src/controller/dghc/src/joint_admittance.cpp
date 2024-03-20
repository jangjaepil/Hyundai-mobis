#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include "rclcpp/rclcpp.hpp"
#include <mobis_admittance/momentum_observer.hpp>
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

#define DOF 6
#define N 200  // Low pass filter size, adjust as needed

double K_i[6] = {10.2312, 10.2874, 10.2604, 7.79396, 8.06884, 10.1232}; //torque constant 파라미터

class EffortFilter : public rclcpp::Node
{
public:
    EffortFilter()
        : Node("effort_filter_node"), last_efforts_(DOF, std::vector<double>(N, 0.0))
    {
        // Subscribe to /joint_states
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&EffortFilter::joint_states_callback, this, std::placeholders::_1));

        // Advertise new topic
        // effort_filtered_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("filtered_current", 10);
        joint_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 100);

        effort_current_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("current", 10);
        external_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("external_torque", 10);
        gravity_comp_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gravity_comp", 100); 

        current_force_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("current_force", 100); 

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


        Eigen::VectorXd k(joint_size);
        k << 90,50,50,90,90,40;
        m_observer = new MomentumObserver(k);

        //admittance parameter
        double M_ = 10; 
        double D_ = 5;
        double K_ = 25;
        M_admittance.diagonal() << M_, M_, M_, M_, M_, M_;
        D_admittance.diagonal() << D_, D_, D_, D_, D_, D_;
        K_admittance.diagonal() << K_, K_, K_, K_, K_, K_; 

        // std::vector<double> M_ = {3.761, 8.058, 2.846, 1.37, 1.3, 0.365};
        // std::vector<double> K_ = {10, 10, 5, 1, 1, 0.5};
        // std::vector<double> D_ = {2*sqrt(K_[0]*M_[0]), 2*sqrt(K_[1]*M_[1]), 2*sqrt(K_[2]*M_[2]), sqrt(K_[3]*M_[3]), sqrt(K_[4]*M_[4]), sqrt(K_[5]*M_[5])};
        
        // M_admittance.diagonal() << M_[0], M_[1], M_[2], M_[3], M_[4], M_[5];
        // D_admittance.diagonal() << D_[0], D_[1], D_[2], D_[3], D_[4], D_[5];
        // K_admittance.diagonal() << K_[0], K_[1], K_[2], K_[3], K_[4], K_[5]; 

        desired_q_vec << init_base, init_shoulder, init_elbow, init_wrist1, init_wrist2, init_wrist3;
    }

    void compute_admittance(const Eigen::MatrixXd& M, const Eigen::MatrixXd& D, const Eigen::MatrixXd& K,
                            Eigen::VectorXd& q_vec, Eigen::VectorXd& desired_q_vec,
                            Eigen::VectorXd& torque_ext, Eigen::VectorXd& q_dot_vec, double& dt)
    {
        Eigen::VectorXd error = q_vec - desired_q_vec;

        Eigen::VectorXd coupling_wrench = Eigen::VectorXd::Zero(6);

        coupling_wrench = - D * q_dot_vec - K * error;
        // q_ddot_vec = M.inverse() * (-coupling_wrench - W * torque_current);

        // Eigen::VectorXd q_ddot_vec = M.inverse() * (coupling_wrench);
        Eigen::VectorXd q_ddot_vec = M.inverse() * (coupling_wrench + torque_ext);


        // q_ddot_vec = M.inverse() * (-coupling_wrench );

        desire_q_dot += q_ddot_vec * dt;
    }

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        
        dt = (rclcpp::Clock{}.now() - last_update_time).seconds();
        last_update_time = rclcpp::Clock{}.now();

        JointPosition[0] = msg->position[5];
        JointPosition[1] = msg->position[0];
        JointPosition[2] = msg->position[1];
        JointPosition[3] = msg->position[2];
        JointPosition[4] = msg->position[3];
        JointPosition[5] = msg->position[4];

        JointVelocity[0] = msg->velocity[5];
        JointVelocity[1] = msg->velocity[0];
        JointVelocity[2] = msg->velocity[1];
        JointVelocity[3] = msg->velocity[2];
        JointVelocity[4] = msg->velocity[3];
        JointVelocity[5] = msg->velocity[4];

        motor_current << msg->effort[5], msg->effort[0], msg->effort[1], msg->effort[2], msg->effort[3], msg->effort[4];

        KDL::JntArray q(joint_size);
        for (int i = 0; i < joint_size; i++) 
        {
            q(i) = JointPosition[i];
        }
        Eigen::VectorXd q_vec = q.data;

        KDL::JntArray q_dot(joint_size);
        for (int i = 0; i < joint_size; i++) 
        {
            q_dot(i) = JointVelocity[i];
        }
        Eigen::VectorXd q_dot_vec = q_dot.data;

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::ChainJntToJacSolver jac_solver(chain);
        KDL::ChainDynParam dyn_param(chain,KDL::Vector(0.0,0.0,-9.8));

        // Compute the inertia matrix
        KDL::JntSpaceInertiaMatrix M_arm(joint_size);
        dyn_param.JntToMass(q,M_arm);
        Eigen::MatrixXd M_arm_mat = M_arm.data;

        // Compute the Gravity term
        KDL::JntArray G_arm(joint_size);
        dyn_param.JntToGravity(q,G_arm);
        Eigen::VectorXd G_arm_mat = G_arm.data;
        
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);
        pinocchio::Data data(model);
        Eigen::MatrixXd Coriolis_arm = pinocchio::computeCoriolisMatrix(model, data, q_vec, q_dot_vec);

        if(skip) 
        {
            skip--;
        }
        else
        {
            // find torques 
            for(int i = 0; i < joint_size; i++) 
                tau(i) = K_i[i] * motor_current(i); //current
            
            // find external torques
            Eigen::VectorXd ext = m_observer->getExternalTorque(M_arm_mat, Coriolis_arm, G_arm_mat, q_vec, q_dot_vec, tau, dt);

          
            compute_admittance(M_admittance, D_admittance, K_admittance, q_vec, desired_q_vec, ext, q_dot_vec, dt);

            std::cout << "M_arm_mat" << std::endl << M_arm_mat << std::endl;
            std::cout << "Coriolis_arm" << std::endl << Coriolis_arm << std::endl;
            std::cout << "G_arm_mat" << std::endl << G_arm_mat.transpose() << std::endl;

            // publish 
            std_msgs::msg::Float64MultiArray external_torque;
            external_torque.data.resize(6);

            std_msgs::msg::Float64MultiArray motor_current_msg;
            motor_current_msg.data.resize(6);

            for(int i = 0; i < joint_size; i++)
            {
                external_torque.data[i] = ext(i);
            }

            for(int i = 0; i < joint_size; i++)
            {
                motor_current_msg.data[i] = motor_current(i);
            }
            
            std_msgs::msg::Float64MultiArray gravity_comp_msg;
            gravity_comp_msg.data.resize(joint_size);
        
            for(int i = 0; i < joint_size; i ++)
            {
                gravity_comp_msg.data[i] = G_arm_mat(i);
            }

            std_msgs::msg::Float64MultiArray vel_msg;
            vel_msg.data.resize(joint_size);
            
            for(int i = 0; i < joint_size; i ++)
            {
                vel_msg.data[i] = desire_q_dot(i);
            }
            vel_msg.data[0] = 0.0;
            vel_msg.data[1] = 0.0;
            vel_msg.data[2] = 0.0;
            // vel_msg.data[3] = 0.0;
            // vel_msg.data[4] = 0.0;
            // vel_msg.data[5] = 0.0;
            
            std_msgs::msg::Float64MultiArray current_force_msg;
            current_force_msg.data.resize(joint_size);
        
            for(int i = 0; i < joint_size; i ++)
            {
                current_force_msg.data[i] = tau(i);
            }
            
            effort_current_pub_->publish(motor_current_msg);
            external_torque_pub_->publish(external_torque);
            gravity_comp_pub_ ->publish(gravity_comp_msg);
            joint_vel_pub->publish(vel_msg);
            current_force_pub_->publish(current_force_msg);
        }
    
    }


    //init joint pose
    float init_base = 4.71239; //270
    float init_shoulder  = -1.5708; //-90도
    float init_elbow = -1.5708; //-90도
    float init_wrist1 = -3.141592; //-180
    float init_wrist2 = 0.0;
    float init_wrist3 = 0.0;  

    float JointPosition[6] = {init_base, init_shoulder,  init_elbow, init_wrist1, init_wrist2, init_wrist3};
    float JointVelocity[6] = {0.0};
    float JointVelocity_past[6] = {0.0};
    float JointAccel[6] = {0.0};
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_comp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr external_torque_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_filtered_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr current_force_pub_;
    std::vector<std::vector<double>> last_efforts_;

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd motor_current = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd desire_q_dot = Eigen::VectorXd::Zero(6);
    
    double K[6] = {1.05, 1.05, 1.05, 0.34, 0.34, 0.34};

    std::string base_link;
    std::string end_link;
    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_filename;
    int joint_size;
    double dt = 0.0;
    rclcpp::Time last_update_time = rclcpp::Clock{}.now();

    MomentumObserver *m_observer = 0;

    int skip = 100;

    Eigen::MatrixXd M_admittance = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd D_admittance = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd K_admittance = Eigen::MatrixXd::Identity(6,6);

    Eigen::VectorXd desired_q_vec = Eigen::VectorXd::Zero(6);


};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EffortFilter>());
    rclcpp::shutdown();
    return 0;
}

