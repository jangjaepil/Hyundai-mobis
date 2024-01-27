
#include "GHCProjections.hpp"
#include "osqp_solver.hpp"
#include "RequiredHeaders.hpp"
#pragma once

//순서대로, taskNumber, a_ii, a_ij
typedef std::vector<std::tuple<int,double,double>> priorityTuples;

class dghc_controller : public GHCProjections,public qp_solver,public rclcpp::Node{
public:
    dghc_controller();
    void joint_states_callback(const sensor_msgs::msg::JointState& JointState_Data);
    void mobile_pose_callback(const linkpose_msgs::msg::LinkPose Pose_Data);
    void mobile_twist_callback(const linkpose_msgs::msg::LinkTwist Twist_Data);
    void desired_pose_callback(const geometry_msgs::msg::Pose& d_Pose_Data);
    void desired_mobile_pose_callback(const geometry_msgs::msg::Pose& d_Mobile_Pose_Data);
    Eigen::MatrixXd KDLFrameToEigenFrame(const KDL::Frame& Frame);
    
    // void obstacle_states_callback(const geometry_msgs::PoseArray::ConstPtr& obstacleState);
    // void priority_input_callback(const std_msgs::String::ConstPtr& priority_input_string);
    // std::vector<double> getDesiredAlphas(priorityTuples priority);
    void getModel();
    void getJacobian();
    void getTwist();
    void setInertia();
    void setPriority();
    void model2realCmd(Eigen::VectorXd V);
    void mobile_pose_estimation();
    void mobile_pose_estimation_kalman(double dt_estimate,Eigen::VectorXd mobile_acc);
    
    // void setTrackingPriority(const int manipulatorTaskNum, const int mobileTaskNum, const int poseTaskNum);
    // void setObstaclePrirority(const std::vector<int> obstacleTaskNums);
    // void setJointLimitPriority(const std::vector<int> jointLimitTaskNums);
    // void priorityFilter();
    // void changeAlphas(std::vector<double>& alphas,double t, double dt, double duration);
    void getProjectionM();
    // bool alphasSetDone(const std::vector<double>& vec1, const std::vector<double>& vec2, double epsilon);
    int run();
    
    
private:

    void timer_callback();
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lift_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr estimated_mobile_pose_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<linkpose_msgs::msg::LinkPose>::SharedPtr mobile_pos_sub_;
    rclcpp::Subscription<linkpose_msgs::msg::LinkTwist>::SharedPtr mobile_twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_mobile_pose_sub;

    std::string base_link;
    std::string end_link;
    KDL::Tree tree;
    KDL::Chain chain;
    KDL::Frame end_effector_pose;
    std::string urdf_filename;
    rclcpp::Time last_update_time = rclcpp::Clock{}.now();
    rclcpp::Time last_estimate_time = rclcpp::Clock{}.now();
    double dt = 0;
    double Lx = 0.3;
    double Ly = 0.3;
    double wheel_radius = 0.001*(173/2); //m
    int joint_size;
    bool init_joint_flag = 0;
    bool init_step = 0;
    unsigned int numTasks = 0;
    unsigned int Dof = 0;
    bool fisrt_loop = 0;
    
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(6);
    
    Eigen::VectorXd wheel_ro = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_ew = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_pr = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd thetalist = Eigen::VectorXd::Zero(4);

    Eigen::VectorXd wheel_ro_dot = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_ew_dot = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_pr_dot = Eigen::VectorXd::Zero(4);
    
    Eigen::VectorXd allq = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd allq_dot = Eigen::VectorXd::Zero(18);

      
    Eigen::MatrixXd Jacobian_arm = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Jacobian_mobile = Eigen::MatrixXd::Identity(6,12);
    Eigen::MatrixXd Jacobian_mobile_inv = Eigen::MatrixXd::Identity(12,6);
    Eigen::MatrixXd Jacobian_whole = Eigen::MatrixXd::Identity(6,18);
    Eigen::MatrixXd Jacobian_base  = Eigen::MatrixXd::Identity(2,18);

    Eigen::MatrixXd end_effector_tmp_TF = Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd end_effector_TF = Eigen::MatrixXd::Identity(4,4);

    Eigen::VectorXd end_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd d_end_position = Eigen::VectorXd::Zero(3);

    Eigen::MatrixXd mobile_TF = Eigen::MatrixXd::Identity(4,4);
    Eigen::VectorXd mobile_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_twist = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd d_mobile_twist = Eigen::VectorXd::Zero(2);

    Eigen::VectorXd end_twist = Eigen::VectorXd::Zero(6);
    Eigen::Matrix3d wRm = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd wRm_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix3d wRe = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd wRe_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix3d mRe = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd mRe_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd desire_adm_vel = Eigen::VectorXd::Zero(6);
    
    Eigen::VectorXd wheel_ew_vel_cmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_ro_vel_cmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_pr_vel_cmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd mani_q_vel_cmd = Eigen::VectorXd::Zero(6);
    
    Eigen::VectorXd estimated_mobile_vel = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd estimated_mobile_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_twist_last = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd X_kk = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd X_k_1_k = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd X_k_1_k_1 = Eigen::VectorXd::Zero(6);
    
    Eigen::MatrixXd P_kk = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd P_k_1_k = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd P_k_1_k_1 = Eigen::MatrixXd::Identity(6,6);
    
    Eigen::MatrixXd Fk = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Bk = Eigen::MatrixXd::Zero(6,3);
    Eigen::VectorXd Uk = Eigen::VectorXd::Zero(3);
    
    Eigen::VectorXd Yk = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd Zk = Eigen::VectorXd::Zero(6);
    
    Eigen::MatrixXd Hk = Eigen::MatrixXd::Identity(6,6);    //measurement model
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Identity(6,6);    //Process covariance  
    Eigen::MatrixXd Rk = Eigen::MatrixXd::Identity(6,6);    //measurement noise covariance
    Eigen::MatrixXd Sk = Eigen::MatrixXd::Identity(6,6);    //residual covariance
    Eigen::MatrixXd Kk = Eigen::MatrixXd::Identity(6,6);    //kalman gain
  
    Eigen::Quaterniond end_quat;
    Eigen::Quaterniond d_end_quat;
    Eigen::Quaterniond d_mobile_quat;
    Eigen::Quaterniond mobile_quat;
    Eigen::VectorXd tasksize;
    std::vector<Eigen::MatrixXd> allJacobians;
    std::vector<Eigen::MatrixXd> allProjections;
    std::vector<Eigen::VectorXd> allx_dot_d;
    
    
    
};