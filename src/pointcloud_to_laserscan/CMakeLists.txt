cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

find_package(ament_cmake_auto REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(laserscan_to_pointcloud SHARED
  src/laserscan_to_pointcloud_node.cpp)

rclcpp_components_register_node(laserscan_to_pointcloud
  PLUGIN "pointcloud_to_laserscan::LaserScanToPointCloudNode"
  EXECUTABLE laserscan_to_pointcloud_node)

ament_auto_add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)

rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

ament_auto_add_executable(dummy_pointcloud_publisher
  src/dummy_pointcloud_publisher.cpp
)

ament_auto_add_executable(pointcloud_merge_node
  src/pointcloud_merge_node.cpp
)

# Include directories
target_include_directories(pointcloud_merge_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(pointcloud_merge_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
