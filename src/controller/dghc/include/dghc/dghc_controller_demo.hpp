
#include "GHCProjections.hpp"
#include "RequiredHeaders.hpp"
#pragma once

//순서대로, taskNumber, a_ii, a_ij
typedef std::vector<std::tuple<int,double,double>> priorityTuples;

class dghc_controller : public GHCProjections ,public rclcpp::Node{
public:
    dghc_controller();
    void topic_callback(const std_msgs::msg::String & msg) const;
    // void obstacle_states_callback(const geometry_msgs::PoseArray::ConstPtr& obstacleState);
    // void desired_pose_callback(const geometry_msgs::Pose& dsired_pose);
    // void mode_input_callback(const std_msgs::Bool::ConstPtr& mode_input);
    // void priority_input_callback(const std_msgs::String::ConstPtr& priority_input_string);
    // void mass_callback(const geometry_msgs::Inertia& mass_matrix);
    // std::vector<double> getDesiredAlphas(priorityTuples priority);
    // void getModel();
    // void getJacobian();
    // void getWrench();
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
    void run();
    
    
private:

    void timer_callback();
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
};