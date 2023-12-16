#include "dghc_controller_demo.hpp"
#include "GHCProjections.hpp"
#include "RequiredHeaders.hpp"
using namespace std::literals::chrono_literals;
dghc_controller::dghc_controller() :Node("dghc_controller")
{   
    KDL::Tree my_tree;
    std::string robot_desc_string;
    node.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
      
    }
    
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
