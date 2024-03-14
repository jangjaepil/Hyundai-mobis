#ifndef SERIAL__SERIAL_COMM_HPP_
#define SERIAL__SERIAL_COMM_HPP_

#include <iostream>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "serial_comm/serial/serial_driver.hpp"
#include "serial_comm/serial/packet.hpp"
#include "serial_comm/serial/control_variable.hpp"
#include "std_msgs/msg/float64.hpp"
#include "hw_msgs/msg/control.hpp"
#include "hw_msgs/msg/control_get_parameter.hpp"
#include "hw_msgs/msg/control_set_parameter.hpp"
#include "imu_msgs/msg/imu.hpp"

namespace serial_comm
{
    using RawBufferPtr = unsigned char *;

    class SerialComm : public rclcpp::Node
    {
    public:
        SerialComm();
        ~SerialComm();

    private:
        void onInit();
        bool onConfigure();
        void onDeactivate();
        bool read();
        void write();
        void receiveCallback(const std::vector<uint8_t> &buffer,
                                const size_t &bytes_transferred);

        void variableCallback(const hw_msgs::msg::Control::SharedPtr send_variable);
        void commandCallback(const hw_msgs::msg::ControlSetParameter::SharedPtr send_variable);
        void imuCallback(const imu_msgs::msg::Imu::SharedPtr send_variable);

        void decodeRawBuffer(comm_packet::RxPacket &, RawBufferPtr &);
        void addCheckSum(comm_packet::TxPacket &tx_packet);

        bool handleRxData(const std::vector<uint8_t> &buffer,
                            const size_t &bytes_transferred);

        void getRxPacketFromRawBuffer(comm_packet::RxPacket &rx_packet,
                                        RawBufferPtr &rb);

        void updateHwStates(comm_packet::RxPacket &rx_packet);

        void statesToPacket(comm_packet::TxPacket &tx_packet);

        void timerCounter();

        void watchdogConnection();

        void resetPacket();

        template <typename T>
        void updateState(double &dest, T &rx_packet_elem, int exponent);


    private:
        rclcpp::Publisher<hw_msgs::msg::Control>::SharedPtr hwVariablePublisher;

        rclcpp::Subscription<hw_msgs::msg::Control>::SharedPtr hwVariableSubscriber;
        rclcpp::Publisher<hw_msgs::msg::ControlGetParameter>::SharedPtr controlCommandPublisher;
        rclcpp::Subscription<hw_msgs::msg::ControlSetParameter>::SharedPtr controlCommandSubscriber;
        rclcpp::Subscription<imu_msgs::msg::Imu>::SharedPtr imuSubscriber;
        
        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config;
        comm_packet::RxPacket rx_packet_;
        hw_msgs::msg::Control hw_variable_from_EtherCAT;
        hw_msgs::msg::Control hw_variable_to_EtherCAT;
        hw_msgs::msg::ControlSetParameter hw_command_to_EtherCAT;
        hw_msgs::msg::ControlGetParameter hw_command_from_EtherCAT;
        imu_msgs::msg::Imu imu_variable_to_EtherCAT;

        std::string port_name;
        uint32_t packet_length_RX;
        uint32_t packet_length_TX;
        uint32_t baud_rate;
        bool isReadOnly;

        comm_packet::TxPacket tx_packet_;
        
        unsigned int watchdog_counter_;
        bool disconnect_trigger;
    
    };

} // end namespace drivers

#endif