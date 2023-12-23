#include "dghc_controller_demo.hpp"
#include "GHCProjections.hpp"
#include "RequiredHeaders.hpp"
using namespace std::literals::chrono_literals;
dghc_controller::dghc_controller() :Node("dghc_controller")
{   
    std::string urdf_string;
        if (!this->get_parameter("robot_description", urdf_string)) {
            RCLCPP_ERROR(get_logger(), "Failed to get URDF from parameter server");
            return;
        }

        // Parse URDF model
        urdf::Model urdf_model;
        if (!urdf_model.initString(urdf_string)) {
            RCLCPP_ERROR(get_logger(), "Failed to parse URDF string");
            return;
        }

        // Get the root and tip link names for your desired chain
        std::string base_link = "manipulator_base_link";
        std::string end_effector_link = "tool0";
        // Construct the KDL tree
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
            RCLCPP_ERROR(get_logger(), "Failed to construct KDL tree from URDF model");
            return;
        }

        // Get the KDL chain
        if (!kdl_tree.getChain(base_link, end_effector_link, kdl_chain_)) {
            RCLCPP_ERROR(get_logger(), "Failed to get KDL chain");
            return;
        }

        RCLCPP_INFO(get_logger(), "Successfully created KDL chain from URDF");
        // Now you can work with the KDL chain as needed
        // ...
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&dghc_controller::timer_callback, this));
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic2", 10, std::bind(&dghc_controller::topic_callback, this, std::placeholders::_1));
}


void dghc_controller::timer_callback()
{
    getModel();


    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    
}

void dghc_controller::topic_callback(const std_msgs::msg::String & msg) const
{
    std::string output  = msg.data;
    std::cout << output<<std::endl;
}


void dghc_controller::getModel()
{  
   
}

void dghc_controller::run()
{
    rclcpp::spin(std::make_shared<dghc_controller>());
    
}
