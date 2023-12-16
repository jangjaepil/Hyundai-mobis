#include "GHCProjections.hpp"
#include "dghc_controller_demo.hpp"
#include "RequiredHeaders.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
  
    enum TaskNum{
        JOINT_POSE=0,
        WHOLE_BODY_IMPEDANCE,
        MOBILE_IMPEDANCE,
        MOBILE_AVOIDACNE,
        JOINT_LIMIT1,
        JOINT_LIMIT2,
        JOINT_LIMIT3,
        JOINT_LIMIT4,
        JOINT_LIMIT5,
        JOINT_LIMIT6
    };

    int DOFsize = 12;
    int numTasks = 10;
    Eigen::VectorXd tasksize;
    tasksize = Eigen::VectorXd::Zero(numTasks);

    tasksize[JOINT_POSE] = 6; 
    tasksize[WHOLE_BODY_IMPEDANCE] = 6; 
    tasksize[MOBILE_IMPEDANCE] = 2; 
    tasksize[MOBILE_AVOIDACNE] = 2;   
    tasksize[JOINT_LIMIT1] = 1; 
    tasksize[JOINT_LIMIT2] = 1; 
    tasksize[JOINT_LIMIT3] = 1; 
    tasksize[JOINT_LIMIT4] = 1; 
    tasksize[JOINT_LIMIT5] = 1;
    tasksize[JOINT_LIMIT6] = 1; 
    
    dghc_controller force_node;
    force_node.init(numTasks, tasksize, DOFsize);
    force_node.run();
    rclcpp::shutdown();
    
    return 0;
}