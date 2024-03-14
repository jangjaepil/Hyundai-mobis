#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("my_node");
  auto broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  // base_frame 생성
  auto base_frame = geometry_msgs::msg::TransformStamped();
  base_frame.header.stamp = node->now();
  base_frame.header.frame_id = "origin";
  base_frame.child_frame_id = "base_frame";
  base_frame.transform.translation.z = 0.5;

  // front_lidar_frame 생성
  auto front_lidar_frame = geometry_msgs::msg::TransformStamped();
  front_lidar_frame.header.stamp = node->now();
  front_lidar_frame.header.frame_id = "base_frame";
  front_lidar_frame.child_frame_id = "front_lidar_frame";
  front_lidar_frame.transform.translation.x = 0.355;
  front_lidar_frame.transform.translation.y = 0.25;
  tf2::Quaternion q_front;
  q_front.setRPY(-M_PI, 0, M_PI/4); // x축 기준 -180도 회전
  front_lidar_frame.transform.rotation.x = q_front.x();
  front_lidar_frame.transform.rotation.y = q_front.y();
  front_lidar_frame.transform.rotation.z = q_front.z();
  front_lidar_frame.transform.rotation.w = q_front.w();

  // rear_lidar_frame 생성
  auto rear_lidar_frame = geometry_msgs::msg::TransformStamped();
  rear_lidar_frame.header.stamp = node->now();
  rear_lidar_frame.header.frame_id = "base_frame";
  rear_lidar_frame.child_frame_id = "rear_lidar_frame";
  rear_lidar_frame.transform.translation.x = -0.355;
  rear_lidar_frame.transform.translation.y = -0.25;
  tf2::Quaternion q_rear;
  q_rear.setRPY(-M_PI, 0, M_PI+M_PI/4); // z축 기준으로는 +180도, x축 기준으로는 -180도 회전
  rear_lidar_frame.transform.rotation.x = q_rear.x();
  rear_lidar_frame.transform.rotation.y = q_rear.y();
  rear_lidar_frame.transform.rotation.z = q_rear.z();
  rear_lidar_frame.transform.rotation.w = q_rear.w();

  // 프레임들을 브로드캐스트
  broadcaster->sendTransform({base_frame, front_lidar_frame, rear_lidar_frame});

  rclcpp::spin(node);
  
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
