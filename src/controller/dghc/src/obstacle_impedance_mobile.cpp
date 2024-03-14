#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <Eigen/Dense>

using Eigen::Vector2d;

class PoseArraySubscriber : public rclcpp::Node {
public:
    PoseArraySubscriber() : Node("obstacle_impedance"), first_message_received_(false) {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/detections", 10, std::bind(&PoseArraySubscriber::poseArrayCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/obstacle_force", 10);
    }

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        std::vector<Vector2d> positions;
        std::vector<Vector2d> n;
        std::vector<Vector2d> dx_o;
        std::vector<double> I_o;
        std::vector<Vector2d> ddx_o;
        std::vector<Vector2d> F_o; // F_o 값을 저장할 벡터 추가
        std::vector<double> F_o_norms;
        if (!msg->poses.empty()) 
        {
            //RCLCPP_INFO(this->get_logger(), "Number of Pose data: %d", msg->poses.size());

            // 현재 메시지의 시간
            rclcpp::Time current_time = msg->header.stamp;

            for (auto pose : msg->poses) 
            {
                Vector2d position(pose.position.x, pose.position.y);
                positions.push_back(position);
                double norm = position.norm();
                I_o.push_back(norm); 
                if (norm != 0) 
                {
                    Vector2d normalized_position = position / norm;
                    n.push_back(normalized_position);
                    Vector2d current_dx_o = (norm - r) * normalized_position;
                    dx_o.push_back(current_dx_o);
                    if (first_message_received_) 
                    {
                        double dt = (current_time - prev_time_).seconds();
                        Vector2d current_ddx_o = (current_dx_o - prev_dx_o_) / dt;
                        ddx_o.push_back(current_ddx_o);
                        F_o.push_back(K_obs_ * current_dx_o + D_obs_ * current_ddx_o); // F_o 값을 계산하여 저장
                        //RCLCPP_INFO(this->get_logger(), "F_o[%d]: %f, %f", F_o.size()-1, F_o.back()(0), F_o.back()(1));

                        double norm_F_o = F_o.back().norm();
                        F_o_norms.push_back(norm_F_o);
                    }
                    prev_dx_o_ = current_dx_o;
                }
            }
            prev_time_ = current_time;
            first_message_received_ = true;
            std_msgs::msg::Float64MultiArray norm_msg;
            norm_msg.data = F_o_norms;
            publisher_->publish(norm_msg);
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "No poses detected");
        }

    }


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    bool first_message_received_;
    double prev_x_, prev_y_;
    rclcpp::Time prev_time_;
    double r = 1.5;
    Vector2d prev_dx_o_; // 이전 dx_o 값을 저장할 변수 추가
    
    Eigen::MatrixXd K_obs_ = Eigen::MatrixXd::Identity(2,2) * 5;
    Eigen::MatrixXd D_obs_ = Eigen::MatrixXd::Identity(2,2) * 1;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseArraySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
