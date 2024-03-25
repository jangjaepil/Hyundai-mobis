
#include "GHCProjections.hpp"
#include "LowPassFilter.hpp"
#include "osqp_solver.hpp"
#include "RequiredHeaders.hpp"
#pragma once

//순서대로, taskNumber, a_ii, a_ij
typedef std::vector<std::tuple<int,double,double>> priorityTuples;

class dghc_controller : public GHCProjections,public qp_solver,public LowPassFilter,public rclcpp::Node{
public:
    dghc_controller();
    void joint_states_callback(const sensor_msgs::msg::JointState& JointState_Data);
    void mobile_joint_states_callback(const hw_msgs::msg::Control::SharedPtr JointState_Data);
    void imu_callback(const sensor_msgs::msg::Imu IMU_Data);
    void ft_callback(const geometry_msgs::msg::WrenchStamped& ft_DATA);
    void desired_pose_callback(const geometry_msgs::msg::Pose& d_Pose_Data);
    void desired_mobile_pose_callback(const geometry_msgs::msg::Pose& d_Mobile_Pose_Data);
    void desired_mobile_parameters_callback(const geometry_msgs::msg::Pose& d_Mobile_Pose_Data);
    bool init_topics();
    void obs_callback(const std_msgs::msg::Float64MultiArray& obs_DATA);
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
    void momentumObs();
    void getProjectionM();
      int run();
    
    
private:

    void timer_callback();
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wrench_pub; 
    rclcpp::Publisher<hw_msgs::msg::Control>::SharedPtr control_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr estimated_end_pose_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<hw_msgs::msg::Control>::SharedPtr mobile_joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_mobile_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_mobile_parameters_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obs_sub;
    std::string base_link;
    std::string end_link;
    KDL::Tree tree;
    KDL::Chain chain;
    KDL::Frame end_effector_pose;
    std::string urdf_filename;

    pinocchio::Model p_model; 
    pinocchio::Data data;

    rclcpp::Time last_update_time = rclcpp::Clock{}.now();  // for getTwist
    rclcpp::Time last_update_time2 = rclcpp::Clock{}.now(); // for DOB
    rclcpp::Time last_update_time3 = rclcpp::Clock{}.now(); // for motor current
    rclcpp::Time last_estimate_time = rclcpp::Clock{}.now(); // mobile pose estimation
    
    double dt = 0;  // admitance
    double dt2 = 0; // for DOB
    double dt3 = 0; // for motor current
    double Lx = 0.240; // m
    double Ly = 0.165;
    double wheel_radius = 0.086; // m
    int joint_size;
    bool init_mobile_joint_flag = 0;
    bool init_mani_joint_flag = 0;
    bool init_imu_flag = 0;
    bool init_parameters_flag = 0;
    bool init_ft_flag = 0;
    bool init_bias_flag = 0;
    bool init_step = 0;
    bool qp_init_flag = 0;
    bool init_obs_flag = 0;
    bool count1 = 0;
    bool count2 = 0;
    bool count3 = 0;
    bool count4 = 0;
    bool count5 = 0;
    bool count6 = 0;
    unsigned int numTasks = 0;
    unsigned int Dof = 0;
    bool fisrt_loop = 0;
    
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd motor_current = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd motor_torque = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd motor_current_tmp = Eigen::VectorXd::Zero(6);
    
    
    Eigen::VectorXd wheel_ro = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_ro_bias = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_ew = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_pr = Eigen::VectorXd::Zero(4);
    
    Eigen::VectorXd wheel_ro_dot = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_ew_dot = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_pr_dot = Eigen::VectorXd::Zero(4);
    
    Eigen::VectorXd allq = Eigen::VectorXd::Zero(18);
    Eigen::VectorXd allq_dot = Eigen::VectorXd::Zero(18);

      
    Eigen::MatrixXd Jacobian_arm = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Jacobian_mobile = Eigen::MatrixXd::Identity(6,12);
    Eigen::MatrixXd Jacobian_mobile_inv = Eigen::MatrixXd::Identity(12,6);
    Eigen::MatrixXd Jacobian_whole = Eigen::MatrixXd::Identity(6,18);
    Eigen::MatrixXd Jacobian_base = Eigen::MatrixXd::Identity(2,18); // mobile jacobian ref: mobile base frame
    Eigen::MatrixXd Jacobian_obs_tmp = Eigen::MatrixXd::Identity(3,18);
    Eigen::MatrixXd Jacobian_obs = Eigen::MatrixXd::Identity(1,18);

    Eigen::MatrixXd M_arm_mat = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd C_arm_mat = Eigen::MatrixXd::Zero(6,6);
    Eigen::VectorXd G_arm_mat = Eigen::VectorXd::Zero(6);
    
    Eigen::MatrixXd end_effector_tmp_TF = Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd end_effector_TF = Eigen::MatrixXd::Identity(4,4);

    Eigen::VectorXd end_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd d_end_position = Eigen::VectorXd::Zero(3);

    Eigen::MatrixXd mobile_TF = Eigen::MatrixXd::Identity(4,4);
    Eigen::VectorXd mobile_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_twist = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd d_mobile_twist = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd b = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd k = Eigen::MatrixXd::Identity(3,3);

    Eigen::VectorXd qd_dd = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd qd_d = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd qd = Eigen::VectorXd::Zero(6);

    Eigen::VectorXd mobile_acc = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd end_twist = Eigen::VectorXd::Zero(6);
    Eigen::Matrix3d wRm = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd wRm_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix3d wRe = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd wRe_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix3d mRe = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd mRe_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd desire_adm_vel = Eigen::VectorXd::Zero(6);
 
    Eigen::VectorXd ForceTorque = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ForceTorque_tmp = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ForceTorque_past = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd tmp = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd bias = Eigen::VectorXd::Zero(6);
    int bias_count = 0; // for FT sensor 
    
    LowPassFilter lpf;
    LowPassFilter lpf_c; //for motor current
    LowPassFilter lpf_t; //for ext torque estimation
    Eigen::VectorXd bias_T = Eigen::VectorXd::Zero(6);
    int bias_count_T = 0; //for ext torque estimation
    
 

    Eigen::VectorXd wheel_ew_vel_cmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_ro_vel_cmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd wheel_pr_vel_cmd = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd mani_q_vel_cmd = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd thetalist = Eigen::VectorXd::Zero(4);

    Eigen::VectorXd estimated_mobile_vel = Eigen::VectorXd::Zero(3);
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

    Eigen::VectorXd mobile_wrench_obs = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd d_mobile_vel_obs = Eigen::VectorXd::Zero(2);
    std::vector<Eigen::VectorXd> obs_position;
    std::vector<Eigen::VectorXd> obs_vel;
    Eigen::VectorXd obs_data;
    
    Eigen::VectorXd Pn = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd Pn_dot = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd Text = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd Text_wg = Eigen::VectorXd::Zero(6);
    
    Eigen::MatrixXd L = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd Tcmd = Eigen::VectorXd::Zero(6);

    
    
    
};