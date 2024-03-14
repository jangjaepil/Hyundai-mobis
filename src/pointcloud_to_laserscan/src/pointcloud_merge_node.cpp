#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudMergeNode : public rclcpp::Node
{
public:
    PointCloudMergeNode() : Node("pointcloud_merge_node"), cloud1_received_(false), cloud2_received_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Sub and Pub Start");

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();

        subscription1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/front/cloud", qos,
            std::bind(&PointCloudMergeNode::cloudCallback1, this, std::placeholders::_1));

        subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rear/cloud", qos,
            std::bind(&PointCloudMergeNode::cloudCallback2, this, std::placeholders::_1));
   
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_cloud", qos);
    }

private:
    bool cloud1_received_;
    bool cloud2_received_;
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud1_;
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud2_;

    void cloudCallback1(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        pcl_cloud1_ = cloudmsg2cloud(*cloud_msg);
        cloud1_received_ = true;

        if (cloud2_received_)
        {
            publish_cloud();
            cloud1_received_ = false;
            cloud2_received_ = false;
        }
    }

    void cloudCallback2(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        pcl_cloud2_ = cloudmsg2cloud(*cloud_msg);
        cloud2_received_ = true;

        if (cloud1_received_)
        {
            publish_cloud();
            cloud1_received_ = false;
            cloud2_received_ = false;
        }
    }

    void publish_cloud()
    {
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud_concat = pcl_cloud1_;
        pcl_cloud_concat += pcl_cloud2_;

        sensor_msgs::msg::PointCloud2 merged_cloud;
        pcl::toROSMsg(pcl_cloud_concat, merged_cloud);

        merged_cloud.header.frame_id = "base_frame";
        publisher_->publish(merged_cloud);
    }

    pcl::PointCloud<pcl::PointXYZI> cloudmsg2cloud(const sensor_msgs::msg::PointCloud2& cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud_dst;
        pcl::fromROSMsg(cloud_msg, cloud_dst);
        return cloud_dst;
    }

    void print_pc(pcl::PointCloud<pcl::PointXYZI>& pcl_cloud){
        for (const auto& pt: pcl_cloud.points){
            RCLCPP_INFO(this->get_logger(), "x point is %f", pt.x);
            RCLCPP_INFO(this->get_logger(), "y point is %f", pt.y);
            RCLCPP_INFO(this->get_logger(), "z point is %f", pt.z);
            RCLCPP_INFO(this->get_logger(), "intensity point is %f", pt.intensity);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudMergeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}