#ifndef SERIAL__SERIAL_COMM_HPP_
#define SERIAL__SERIAL_COMM_HPP_

#include <iostream>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <chrono>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "serial_comm/serial/serial_driver.hpp"
#include "imu_msgs/msg/imu.hpp"

#define ASCII_CARIAGERETURN 13
#define ASCII_LINEFEED 10
#define ASCII_MINUS 45
#define ASCII_COMMA 44
#define ASCII_DOT 46

namespace serial_comm
{
    using RawBufferPtr = unsigned char *;

    class ImuSerialComm : public rclcpp::Node
    {
    public:
        ImuSerialComm();
        ~ImuSerialComm();

    private:
        void onInit();
        bool onConfigure();
        void onDeactivate();
        bool read();
        void receiveCallback(const std::vector<uint8_t> &buffer,
                                const size_t &bytes_transferred);

        // void decodeRawBuffer(comm_packet::RxPacket &, RawBufferPtr &);

        bool handleRxData(const std::vector<uint8_t> &buffer,
                            const size_t &bytes_transferred);

        void timerCounter();


    private:
        rclcpp::Publisher<imu_msgs::msg::Imu>::SharedPtr imuPublisher;
        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config;

        std::string port_name;
        uint32_t packet_length_RX;
        uint32_t packet_length_TX;
        uint32_t baud_rate;
        bool isReadOnly;

        std::vector<double> packet;
        double time_;
        double roll_;
        double pitch_;
        double yaw_;

        static const int buff_size = 1024;
        char stack_buffer[buff_size];
    };

} // end namespace drivers

#endif