cmake_minimum_required(VERSION 3.8)
project(dghc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hw_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(linkpose_msgs REQUIRED)
find_package(Eigen3)
find_package(OsqpEigen REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/dghc
  )

add_executable(dghc_demo
  src/dghc_controller_demo.cpp
  src/dghc_demo.cpp
  src/GHCProjections_demo.cpp
  src/qp_solver.cpp
  src/LowPassFilter.cpp
  )

add_executable(trajectory
  src/trajectoryMaker.cpp)

add_executable(obstacle
src/obstacle_impedance_mobile.cpp)

target_link_libraries (dghc_demo Eigen3::Eigen OsqpEigen::OsqpEigen)
target_link_libraries (trajectory Eigen3::Eigen)
target_link_libraries (obstacle Eigen3::Eigen)

ament_target_dependencies(dghc_demo rclcpp std_msgs sensor_msgs geometry_msgs linkpose_msgs hw_msgs kdl_parser urdf) 
ament_target_dependencies(trajectory rclcpp std_msgs geometry_msgs) 
ament_target_dependencies(obstacle rclcpp std_msgs geometry_msgs) 

install(
  TARGETS 
  dghc_demo
  trajectory
  obstacle
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)  
ament_package()