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


#include <math.h>
#include <cmath>
#include <vector>
#include "std_msgs/msg/float32_multi_array.h"
#include <algorithm>
#include <memory>
#include <string>   
#include <utility>
#include <vector>

#include "ur_client_library/exceptions.h"
#include "ur_client_library/ur/tool_communication.h"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ur_robot_driver/hardware_interface.hpp"
#include "ur_robot_driver/urcl_log_handler.hpp"