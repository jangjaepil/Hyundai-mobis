
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
    Eigen::MatrixXd KDLFrameToEigenFrame(const KDL::Frame& Frame);
    // void obstacle_states_callback(const geometry_msgs::PoseArray::ConstPtr& obstacleState);
    // void desired_pose_callback(const geometry_msgs::Pose& dsired_pose);
    // void mode_input_callback(const std_msgs::Bool::ConstPtr& mode_input);
    // void priority_input_callback(const std_msgs::String::ConstPtr& priority_input_string);
    // void mass_callback(const geometry_msgs::Inertia& mass_matrix);
    // std::vector<double> getDesiredAlphas(priorityTuples priority);
    void getModel();
    void getJacobian();
    void getTwist();
    // void setInertia();
    // void setPriority();
    // void setTrackingPriority(const int manipulatorTaskNum, const int mobileTaskNum, const int poseTaskNum);
    // void setObstaclePrirority(const std::vector<int> obstacleTaskNums);
    // void setJointLimitPriority(const std::vector<int> jointLimitTaskNums);
    // void priorityFilter();
    // void changeAlphas(std::vector<double>& alphas,double t, double dt, double duration);
    // void getProjectionM();
    // void getProjectedToq();
    // bool alphasSetDone(const std::vector<double>& vec1, const std::vector<double>& vec2, double epsilon);
    int run();
    
    
private:

    void timer_callback();
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<linkpose_msgs::msg::LinkPose>::SharedPtr mobile_pos_sub_;
    rclcpp::Subscription<linkpose_msgs::msg::LinkTwist>::SharedPtr mobile_twist_sub_;

    std::string base_link;
    std::string end_link;
    KDL::Tree tree;
    KDL::Chain chain;
    KDL::Frame end_effector_pose;
    std::string urdf_filename;

    double Lx = 0.3;
    double Ly = 0.3;
    int joint_size;
    bool init_joint_flag = 0;
    bool init_step = 0;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd mobile_q = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd mobile_q_dot = Eigen::VectorXd::Zero(12);
    Eigen::MatrixXd Jacobian_arm = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Jacobian_mobile = Eigen::MatrixXd::Identity(6,12);
    Eigen::MatrixXd Jacobian_mobile_inv = Eigen::MatrixXd::Identity(12,6);
    Eigen::MatrixXd Jacobian_whole = Eigen::MatrixXd::Identity(6,18);
    Eigen::MatrixXd end_effector_tmp_TF = Eigen::MatrixXd::Identity(4,4);
    Eigen::MatrixXd end_effector_TF = Eigen::MatrixXd::Identity(4,4);
    Eigen::VectorXd end_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd d_end_position = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd mobile_TF = Eigen::MatrixXd::Identity(4,4);
    Eigen::VectorXd mobile_position = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_twist = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd end_twist = Eigen::VectorXd::Zero(6);
    Eigen::Matrix3d wRm = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd wRm_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix3d wRe = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd wRe_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::Matrix3d mRe = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd mRe_e = Eigen::MatrixXd::Identity(6,6);
    Eigen::Quaterniond end_quat;
    Eigen::Quaterniond d_end_quat;
    Eigen::Quaterniond mobile_quat;
    
};