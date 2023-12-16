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
find_package(geometry_msgs REQUIRED)
find_package(Eigen3)

include_directories(
  include/dghc
  )

add_executable(dghc_demo
  src/dghc_controller_demo.cpp
  src/dghc_demo.cpp
  src/GHCProjections_demo.cpp)

target_link_libraries (dghc_demo Eigen3::Eigen)


ament_target_dependencies(dghc_demo rclcpp std_msgs geometry_msgs)

install(
  TARGETS dghc_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()