cmake_minimum_required(VERSION 3.8)
project(manipulator_admittance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
#find_package(ur_robot_driver REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(pinocchio REQUIRED)
find_package(tf2 REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(linkpose_msgs REQUIRED)




include_directories(${orocos_kdl_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(task_admittance_control src/task_admittance_control.cpp)
target_include_directories(task_admittance_control PRIVATE ${controller_manager_INCLUDE_DIRS})
ament_target_dependencies(task_admittance_control rclcpp std_msgs geometry_msgs linkpose_msgs sensor_msgs orocos_kdl kdl_parser urdf tf2 controller_manager robot_state_publisher) #ur_robot_driver pinocchio)


install(TARGETS
  task_admittance_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
