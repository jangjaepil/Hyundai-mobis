#ifndef SERIAL__SERIAL_LOGGER_HPP_
#define SERIAL__SERIAL_LOGGER_HPP_

#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "hw_msgs/msg/control.hpp"
#include "imu_msgs/msg/imu.hpp"
#include "matplotlibcpp.h"

namespace serial_comm
{
    class SerialLogger : public rclcpp::Node
    {
        public:
            SerialLogger();
            ~SerialLogger();
        
        private:
            void getVariableCallback(const hw_msgs::msg::Control::SharedPtr send_variable);
            void setVariableCallback(const hw_msgs::msg::Control::SharedPtr send_variable);
            void imuCallback(const imu_msgs::msg::Imu::SharedPtr send_variable);
            void updateCSVFile();
            void readCSVFile(const std::string& filename);
            

            rclcpp::Subscription<hw_msgs::msg::Control>::SharedPtr getControlLogger;
            rclcpp::Subscription<hw_msgs::msg::Control>::SharedPtr setControlLogger;
            rclcpp::Subscription<imu_msgs::msg::Imu>::SharedPtr imuLogger;

            hw_msgs::msg::Control hw_variable_from_EtherCAT;
            hw_msgs::msg::Control hw_variable_to_EtherCAT;
            imu_msgs::msg::Imu imu_variable_to_EtherCAT;
            std::ofstream csv_file_;
            std::string filename_;
            std::string filename_idx;
            std::vector<std::vector<double>> data_;

    };
}
#endif