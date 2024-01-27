#include "GHCProjections.hpp"
#include "dghc_controller_demo.hpp"
#include "RequiredHeaders.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto DGHC_node = std::make_shared<dghc_controller>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(DGHC_node);
    std::thread executor_thread([&executor]() { executor.spin(); });

    
    

    
    enum TaskNum{
        //JOINT_POSE=0,
        WHOLE_BODY_IMPEDANCE,
        MOBILE_IMPEDANCE
        // MOBILE_AVOIDACNE,
        // JOINT_LIMIT1,
        // JOINT_LIMIT2,
        // JOINT_LIMIT3,
        // JOINT_LIMIT4,
        // JOINT_LIMIT5,
        // JOINT_LIMIT6
    };

    int DOFsize = 18;
    int numTasks = 2;
    Eigen::VectorXd tasksize;
    tasksize = Eigen::VectorXd::Zero(numTasks);

    //tasksize[JOINT_POSE] = 6; 
    tasksize[WHOLE_BODY_IMPEDANCE] = 6; 
    tasksize[MOBILE_IMPEDANCE] = 2; 
    // tasksize[MOBILE_AVOIDACNE] = 2;   
    // tasksize[JOINT_LIMIT1] = 1; 
    // tasksize[JOINT_LIMIT2] = 1; 
    // tasksize[JOINT_LIMIT3] = 1; 
    // tasksize[JOINT_LIMIT4] = 1; 
    // tasksize[JOINT_LIMIT5] = 1;
    // tasksize[JOINT_LIMIT6] = 1; 
    
    Eigen::MatrixXd Qi = Eigen::MatrixXd::Identity(tasksize.sum(),tasksize.sum());
    Eigen::MatrixXd Qr = Eigen::MatrixXd::Identity(DOFsize*numTasks,DOFsize*numTasks);
    
    Qr.diagonal() << 0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001, //task 1 : wheel vel
                    0.1,0.1,0.1,0.1,                                  //task 1 : lift vel
                    0.01,0.01,0.01,0.01,0.01,0.01,                          //task 1 : mani vel
                    0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,                  
                    0.001,0.001,0.001,0.001
                    ,0.1,0.1,0.1,0.1,0.1,0.1;
    DGHC_node->init(numTasks, tasksize, DOFsize);
    DGHC_node->qp_setWeightMatrices(Qr,Qi);
    DGHC_node->run();
    
    executor_thread.join();
    rclcpp::shutdown();
    
    return 0;
}