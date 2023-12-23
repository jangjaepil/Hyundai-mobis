#pragma once

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <iostream>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/wrench.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>

// ROS includes
#include <controller_manager/controller_manager.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <ur_rtde/rtde_control_interface.h>

#include <vector>
#include <cmath>
#include <chrono>
// #include "ur_robot_driver/hardware_interface.hpp"


#include <math.h>
#include <cmath>
#include <vector>
#include "std_msgs/msg/float32_multi_array.h"
#include <algorithm>
#include <memory>
#include <string>   
#include <utility>
#include <vector>

// #include "ur_client_library/exceptions.h"
// #include "ur_client_library/ur/tool_communication.h"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "ur_robot_driver/hardware_interface.hpp"
// #include "ur_robot_driver/urcl_log_handler.hpp"

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <iostream>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/pose.hpp"


// #define PI 3.141592


//wheel distance
double a = 0.3;
double b = 0.3;//300mm?

// each wheel angle
double wheel_theta_fl = 0;
double wheel_theta_rl = 0;
double wheel_theta_rr = 0;
double wheel_theta_fr = 0;



double act_angle = 0;
// model position & orientation
double Px  = 0;
double Py  = 0;


Eigen::Vector3d Pos;

// wheel position
double x_w_1 = a;
double x_w_2 = -a;
double x_w_3 = -a;
double x_w_4 = a;

double y_w_1 = b;
double y_w_2 = b;
double y_w_3 = -b;
double y_w_4 = -b;

double r_inv = 1/0.173;

// wheel position matrix
Eigen::MatrixXd fl(1,2);
Eigen::MatrixXd rl(1,2);
Eigen::MatrixXd rr(1,2);
Eigen::MatrixXd fr(1,2);

// position matrix
Eigen::MatrixXd P(8,3);
Eigen::MatrixXd X(8,4);
Eigen::MatrixXd Xp(8,4);


double wheel_theta_dot_fl_ew = 0;
double wheel_theta_dot_rl_ew = 0;
double wheel_theta_dot_rr_ew = 0;
double wheel_theta_dot_fr_ew = 0;

double wheel_theta_dot_fl_rot = 0;
double wheel_theta_dot_rl_rot = 0;
double wheel_theta_dot_rr_rot = 0;
double wheel_theta_dot_fr_rot = 0;