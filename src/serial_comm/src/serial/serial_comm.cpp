#include "serial_comm/serial/serial_comm.hpp"
#define m2mm 1E+3
#define mm2m 1E-3
#define deg2rad 0.017453289
using namespace std::chrono_literals;

namespace serial_comm
{

    SerialComm::SerialComm() : Node("serialCommunication")
    {
        onInit();

        // QoS profile 설정
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

        hwVariablePublisher = this->create_publisher<hw_msgs::msg::Control>("Robot_Variable", 10);
        hwVariableSubscriber = this->create_subscription<hw_msgs::msg::Control>("Control_Variable", qos_profile,
                                std::bind(&SerialComm::variableCallback, this, std::placeholders::_1));

        controlCommandPublisher = this->create_publisher<hw_msgs::msg::ControlGetParameter>("Control_Get_Parameter", 10);
        controlCommandSubscriber = this->create_subscription<hw_msgs::msg::ControlSetParameter>("Control_Set_Parameter", qos_profile,
                                    std::bind(&SerialComm::commandCallback, this, std::placeholders::_1));

        imuSubscriber = this->create_subscription<imu_msgs::msg::Imu>("Imu_Variable", qos_profile,
                            std::bind(&SerialComm::imuCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(20ms, std::bind(&SerialComm::timerCounter, this));

        onConfigure();
    }

    SerialComm::~SerialComm()
    {
        onDeactivate();
    }

    void
    SerialComm::onInit()
    {
        packet_length_RX = 66;
        packet_length_TX = 67;
        isReadOnly = false;
        // port_name = "/dev/ttyUSB_Serial";
        port_name = "/dev/ttyUSB0";
        baud_rate = 115200;

        auto flowControl = drivers::serial_driver::FlowControl::NONE;
        auto parityBit = drivers::serial_driver::Parity::NONE;
        auto stopBit = drivers::serial_driver::StopBits::ONE;
        device_config = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, flowControl, parityBit, stopBit);

        drivers::common::IoContext *ctx = new drivers::common::IoContext(2);
        drivers::serial_driver::SerialDriver *pa1 = new drivers::serial_driver::SerialDriver(*ctx);
        serial_driver.reset(pa1);

        tx_packet_.counter = 0;
        watchdog_counter_ = 0;
        disconnect_trigger = false;

        RCLCPP_INFO(this->get_logger(), "Serial Device Configuration Complete...!!");
        RCLCPP_INFO(this->get_logger(), "packet RX length : %li // packet TX length : %li", sizeof(rx_packet_), sizeof(tx_packet_));
    }

    bool
    SerialComm::onConfigure()
    {
        try
        {
            serial_driver->init_port(port_name, *device_config);

            if (!serial_driver->port()->is_open())
            {
                serial_driver->port()->open();
                serial_driver->port()->async_receive(
                    std::bind(&SerialComm::receiveCallback, this, std::placeholders::_1, std::placeholders::_2));
                
                RCLCPP_INFO(this->get_logger(), " Serial Port Connection State : %i ...!! ",serial_driver->port()->is_open());
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(this->get_logger(), " Error Creating Serial Port : %s - %s", port_name.c_str(), ex.what());
            return false;
        }
        return true;
    }

    void
    SerialComm::onDeactivate()
    {
        serial_driver->port()->close();
    }

    bool
    SerialComm::read()
    {   
        updateHwStates(rx_packet_);
        return true;
    }
    void
    SerialComm::write()
    {

        if(disconnect_trigger)
        {
            resetPacket();
            RCLCPP_INFO(this->get_logger(),"Reset the Packet !!!!");
        }
        else
            statesToPacket(tx_packet_);

        
        // RCLCPP_INFO(this->get_logger(),"Control Command : %d ", tx_packet_.controlCommand);

        addCheckSum(tx_packet_);

        uint8_t *serialized_tx = new unsigned char[packet_length_TX];
        memcpy((serialized_tx), (void *)&(tx_packet_), packet_length_TX);

        std::vector<std::uint8_t> tx_byte(&serialized_tx[0], &serialized_tx[packet_length_TX]);

        // int num = 0;
        // for(size_t i = 4; i < packet_length_TX - 1; i++) {
        //     tx_byte[i] = ++num;
        // }


        // static uint8_t counter_chk;
        // static uint8_t pre_counter_chk;
        // counter_chk = tx_byte[2];

        // int ct_chk = counter_chk - pre_counter_chk;

        // if(ct_chk!=1 && ct_chk!=-255) {
        //     RCLCPP_INFO(this->get_logger(),"counter chk : %d", counter_chk);
        // }

        // pre_counter_chk = counter_chk;
        // if(!counter_chk) {
        //     RCLCPP_INFO(this->get_logger(),"counter chk : %d", counter_chk);
        // }

        serial_driver->port()->send(tx_byte);

        memset(&serialized_tx, 0, sizeof(serialized_tx));
    }

    void
    SerialComm::watchdogConnection()
    {
        static unsigned int pre_counter = 0;
        static unsigned int err_counter = 0;

        if(pre_counter == watchdog_counter_)
        {
            err_counter++;
        }
        else err_counter = 0;

        // RCLCPP_INFO(this->get_logger()," err_counter is %d",err_counter);

        pre_counter = watchdog_counter_;

        if(err_counter > 100){
            disconnect_trigger = true;

            hw_command_from_EtherCAT.serial_disconnect_on = true;
            controlCommandPublisher->publish(hw_command_from_EtherCAT);
        }
    }

    void
    SerialComm::timerCounter()
    {
        write();
        watchdogConnection();

        // // static 이전 시간 기록
        // static std::chrono::high_resolution_clock::time_point lastCallTime = std::chrono::high_resolution_clock::now();

        // // 현재 시간 기록
        // std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

        // // 이전 호출에서 현재 호출까지의 시간 간격 계산
        // std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastCallTime);

        // // 실행 주기 출력
        // std::cout << "Function called. Elapsed time: " << elapsedTime.count() << " seconds." << std::endl;

        // // 현재 시간을 마지막 호출 시간으로 설정
        // lastCallTime = currentTime;
    }

    void
    SerialComm::receiveCallback(const std::vector<uint8_t> &buffer,
                                const size_t &bytes_transferred)
    {
        watchdog_counter_ += 1;
        handleRxData(buffer, bytes_transferred);

    }

    void
    SerialComm::imuCallback(const imu_msgs::msg::Imu::SharedPtr send_variable)
    {
        if (!disconnect_trigger){
            imu_variable_to_EtherCAT = *send_variable;
        }
        else
            return;
    }

    void 
    SerialComm::variableCallback(const hw_msgs::msg::Control::SharedPtr send_variable)
    {
        if(!disconnect_trigger){
            hw_variable_to_EtherCAT = *send_variable;
        }
        else return;
    }

    void 
    SerialComm::commandCallback(const hw_msgs::msg::ControlSetParameter::SharedPtr send_variable)
    {
        if(send_variable->serial_reconnect_on){
            disconnect_trigger = false;
        }

        if(!disconnect_trigger){
            hw_command_to_EtherCAT = *send_variable;
        }
        else return;

    }

    bool
    SerialComm::handleRxData(const std::vector<uint8_t> &buffer,
                                const size_t &bytes_transferred)
    {
        
        // RCLCPP_INFO(this->get_logger(), "bytes_Transferred : %ld ",bytes_transferred);

        static bool is_header_find = false;
        static uint16_t current_buff_size = 0;
        static uint16_t current_head_buff_size = 0;
        static std::vector<uint8_t> header_buffer(500);
        static std::vector<uint8_t> stack_buffer(500);
        static std::vector<uint8_t> temp(500);

        uint8_t start_idx = 0;
        bool res = false;



        //--------------------------------//


        //--------------------------------//


        #if 1
        
        if (!is_header_find)
        {

            std::copy(buffer.begin(), buffer.begin() + bytes_transferred, header_buffer.begin() + current_head_buff_size);
            current_head_buff_size += bytes_transferred;
            // RCLCPP_INFO(this->get_logger(), "header finder size : %d",current_head_buff_size);

            if(current_head_buff_size >= 66) {

                for (size_t i = 1; i < current_head_buff_size; i++)
                {
                    if (header_buffer[i - 1] == comm_packet::HEADER && header_buffer[i] == comm_packet::RX_ID)
                    {
                        start_idx = i - 1;
                        std::copy(header_buffer.begin() + start_idx, header_buffer.end(), stack_buffer.begin());
                        current_buff_size = current_head_buff_size - start_idx;

                        current_head_buff_size = 0;
                        std::vector<uint8_t>().swap(header_buffer);
                        is_header_find = true;
                        break;
                    }
                }
                if(!is_header_find) {
                    std::vector<uint8_t>().swap(header_buffer);
                    current_head_buff_size = 0;
                }
            }
        }

        else
        {
            std::copy(buffer.begin(), buffer.begin() + bytes_transferred, stack_buffer.begin() + current_buff_size);
            current_buff_size += bytes_transferred;

            if (current_buff_size >= packet_length_RX) // Complete
            {
                uint8_t remainder = current_buff_size - packet_length_RX;

                if (stack_buffer[0] == comm_packet::HEADER 
                    && stack_buffer[1] == comm_packet::RX_ID
                    && stack_buffer[packet_length_RX - 1] == comm_packet::FOOTER)
                {

                    unsigned char chksum = stack_buffer[0];
                    for (size_t i = 1; i < packet_length_RX - 2; i++)
                    {
                        chksum ^= stack_buffer[i];
                    }

                    chksum &= 0xff;

                    
                    if(chksum != stack_buffer[packet_length_RX - 2]) {
                        RCLCPP_INFO(this->get_logger()," Chksum Do not agree !!");
                    }
                    else {
                        RawBufferPtr rb;
                        rb = stack_buffer.data();
                        decodeRawBuffer(rx_packet_, rb);
                    }

                    if (remainder != 0) {
                        std::copy(stack_buffer.begin(), stack_buffer.begin() + current_buff_size, temp.begin());
                        stack_buffer.clear();
                        std::copy(temp.begin() + packet_length_RX , temp.begin() + current_buff_size, stack_buffer.begin());
                        temp.clear();
                    }
                    else {
                        stack_buffer.clear();
                    }

                    res = true;
                    current_buff_size = remainder;
                }

                else // Lost header
                {
                    // RCLCPP_INFO(this->get_logger(), "lost packet // HEADER : 0x%x ID : 0x%x // FOOTER : 0x%x ",stack_buffer[0], stack_buffer[1],stack_buffer[packet_length_RX - 1]);

                    is_header_find = false;
                    stack_buffer.clear();
                    current_buff_size = 0;
                }
            }
        }
        #endif

        return res;
    }

    void
    SerialComm::decodeRawBuffer(comm_packet::RxPacket &rx_packet, RawBufferPtr &rb)
    {
        getRxPacketFromRawBuffer(rx_packet, rb);
        read();
    }

    void
    SerialComm::addCheckSum(comm_packet::TxPacket &tx_packet)
    {
        auto raw = reinterpret_cast<unsigned char *>(&tx_packet);
        tx_packet.chksum = raw[0];

        for (size_t i = 1; i < sizeof(tx_packet) - 2; i++)
        {
            tx_packet.chksum ^= raw[i];
        }

        tx_packet.chksum &= 0xff;
    }

    void
    SerialComm::getRxPacketFromRawBuffer(comm_packet::RxPacket &rx_packet,
                                            RawBufferPtr &rb)
    {
        static int counter_check;

        rx_packet.header = rb[0];
        rx_packet.type = rb[1];
        rx_packet.counter = rb[2];
        rx_packet.length = rb[3];

        rx_packet.wheelVelFL = (int16_t)((rb[4] & 0x00ff) | ((rb[5] << 8) & 0xff00));
        rx_packet.wheelVelFR = (int16_t)((rb[6] & 0x00ff) | ((rb[7] << 8) & 0xff00));
        rx_packet.wheelVelRL = (int16_t)((rb[8] & 0x00ff) | ((rb[9] << 8) & 0xff00));
        rx_packet.wheelVelRR = (int16_t)((rb[10] & 0x00ff) | ((rb[11] << 8) & 0xff00));
        rx_packet.wheelAcc = (int16_t)((rb[12] & 0x00ff) | ((rb[13] << 8 ) & 0xff00));

        rx_packet.steerVelFL = (int16_t)((rb[14] & 0x00ff) | ((rb[15] << 8) & 0xff00));
        rx_packet.steerVelFR = (int16_t)((rb[16] & 0x00ff) | ((rb[17] << 8) & 0xff00));
        rx_packet.steerVelRL = (int16_t)((rb[18] & 0x00ff) | ((rb[19] << 8) & 0xff00));
        rx_packet.steerVelRR = (int16_t)((rb[20] & 0x00ff) | ((rb[21] << 8) & 0xff00));

        rx_packet.steerPosFL = (int16_t)((rb[22] & 0x00ff) | ((rb[23] << 8) & 0xff00));
        rx_packet.steerPosFR = (int16_t)((rb[24] & 0x00ff) | ((rb[25] << 8) & 0xff00));
        rx_packet.steerPosRL = (int16_t)((rb[26] & 0x00ff) | ((rb[27] << 8) & 0xff00));
        rx_packet.steerPosRR = (int16_t)((rb[28] & 0x00ff) | ((rb[29] << 8) & 0xff00));
        rx_packet.steerAcc = (int16_t)((rb[30] & 0x00ff) | ((rb[31] << 8 ) & 0xff00));

        rx_packet.liftVelFL = (int16_t)((rb[32] & 0x00ff) | ((rb[33] << 8) & 0xff00));
        rx_packet.liftVelFR = (int16_t)((rb[34] & 0x00ff) | ((rb[35] << 8) & 0xff00));
        rx_packet.liftVelRL = (int16_t)((rb[36] & 0x00ff) | ((rb[37] << 8) & 0xff00));
        rx_packet.liftVelRR = (int16_t)((rb[38] & 0x00ff) | ((rb[39] << 8) & 0xff00));

        rx_packet.liftPosFL = (int16_t)((rb[40] & 0x00ff) | ((rb[41] << 8) & 0xff00));
        rx_packet.liftPosFR = (int16_t)((rb[42] & 0x00ff) | ((rb[43] << 8) & 0xff00));
        rx_packet.liftPosRL = (int16_t)((rb[44] & 0x00ff) | ((rb[45] << 8) & 0xff00));
        rx_packet.liftPosRR = (int16_t)((rb[46] & 0x00ff) | ((rb[47] << 8) & 0xff00));

        rx_packet.liftTrqFL = (int16_t)((rb[48] & 0x00ff) | ((rb[49] << 8) & 0xff00));
        rx_packet.liftTrqFR = (int16_t)((rb[50] & 0x00ff) | ((rb[51] << 8) & 0xff00));
        rx_packet.liftTrqRL = (int16_t)((rb[52] & 0x00ff) | ((rb[53] << 8) & 0xff00));
        rx_packet.liftTrqRR = (int16_t)((rb[54] & 0x00ff) | ((rb[55] << 8) & 0xff00));
        rx_packet.liftAcc = (int16_t)((rb[56] & 0x00ff) | ((rb[57] << 8 ) & 0xff00));

        rx_packet.liftDistMin = rb[58];
        rx_packet.liftDistMax = rb[59];

        rx_packet.controlCommand = rb[60];
        rx_packet.battery1SOC = rb[61];
        rx_packet.battery2SOC = rb[62];
        rx_packet.reserved0 = rb[63];
        rx_packet.chksum = rb[64];
        rx_packet.footer = rb[65];

        // auto chk = rx_packet.counter - counter_check;
        // static int n;
        // RCLCPP_INFO(this->get_logger(), "counter check : %d", rx_packet.counter);

        // if(chk != 1 && chk != -255)
        // {
        //     RCLCPP_INFO(this->get_logger(), "RX counter error...!! : %d", n++);
        // }
        // counter_check = rx_packet.counter;

    }

    void
    SerialComm::updateHwStates(comm_packet::RxPacket &rx_packet)
    {
        // packet에 x100 x10 등을 역연산

        updateState(hw_variable_from_EtherCAT.wheel_vel_fl, rx_packet.wheelVelFL, -1);
        updateState(hw_variable_from_EtherCAT.wheel_vel_fr, rx_packet.wheelVelFR, -1);
        updateState(hw_variable_from_EtherCAT.wheel_vel_rl, rx_packet.wheelVelRL, -1);
        updateState(hw_variable_from_EtherCAT.wheel_vel_rr, rx_packet.wheelVelRR, -1);

        updateState(hw_variable_from_EtherCAT.steering_vel_fl, rx_packet.steerVelFL, -1);
        updateState(hw_variable_from_EtherCAT.steering_vel_fr, rx_packet.steerVelFR, -1);
        updateState(hw_variable_from_EtherCAT.steering_vel_rl, rx_packet.steerVelRL, -1);
        updateState(hw_variable_from_EtherCAT.steering_vel_rr, rx_packet.steerVelRR, -1);

        updateState(hw_variable_from_EtherCAT.steering_pos_fl, rx_packet.steerPosFL, -2);
        updateState(hw_variable_from_EtherCAT.steering_pos_fr, rx_packet.steerPosFR, -2);
        updateState(hw_variable_from_EtherCAT.steering_pos_rl, rx_packet.steerPosRL, -2);
        updateState(hw_variable_from_EtherCAT.steering_pos_rr, rx_packet.steerPosRR, -2);

        updateState(hw_variable_from_EtherCAT.lift_vel_fl, rx_packet.liftVelFL, -4);
        updateState(hw_variable_from_EtherCAT.lift_vel_fr, rx_packet.liftVelFR, -4);
        updateState(hw_variable_from_EtherCAT.lift_vel_rl, rx_packet.liftVelRL, -4);
        updateState(hw_variable_from_EtherCAT.lift_vel_rr, rx_packet.liftVelRR, -4);

        updateState(hw_variable_from_EtherCAT.lift_pos_fl, rx_packet.liftPosFL, -4);
        updateState(hw_variable_from_EtherCAT.lift_pos_fr, rx_packet.liftPosFR, -4);
        updateState(hw_variable_from_EtherCAT.lift_pos_rl, rx_packet.liftPosRL, -4);
        updateState(hw_variable_from_EtherCAT.lift_pos_rr, rx_packet.liftPosRR, -4);

        updateState(hw_variable_from_EtherCAT.lift_trq_fl, rx_packet.liftTrqFL, -4);
        updateState(hw_variable_from_EtherCAT.lift_trq_fr, rx_packet.liftTrqFR, -4);
        updateState(hw_variable_from_EtherCAT.lift_trq_rl, rx_packet.liftTrqRL, -4);
        updateState(hw_variable_from_EtherCAT.lift_trq_rr, rx_packet.liftTrqRR, -4);

        updateState(hw_command_from_EtherCAT.wheel_acc_limit, rx_packet.wheelAcc, -1);
        updateState(hw_command_from_EtherCAT.steering_acc_limit, rx_packet.steerAcc, -1);
        updateState(hw_command_from_EtherCAT.lift_acc_limit, rx_packet.liftAcc, -4);
        updateState(hw_command_from_EtherCAT.lift_dist_min, rx_packet.liftDistMin, -4);
        updateState(hw_command_from_EtherCAT.lift_dist_max, rx_packet.liftDistMax, -4);


        hw_command_from_EtherCAT.mobile_battery = rx_packet.battery1SOC;
        hw_command_from_EtherCAT.manipulator_battery = rx_packet.battery2SOC;
        hw_command_from_EtherCAT.motor_drive_on = 0x01 & rx_packet.controlCommand;
        hw_command_from_EtherCAT.lift_brake_on = (0x02 & rx_packet.controlCommand) >> 1;
        hw_command_from_EtherCAT.wheel_brake_on = (0x04 & rx_packet.controlCommand) >> 2;
        hw_command_from_EtherCAT.motor_drive_error_clear = (0x08 & rx_packet.controlCommand) >> 3;
        hw_command_from_EtherCAT.serial_disconnect_on = disconnect_trigger;

        hwVariablePublisher->publish(hw_variable_from_EtherCAT);
        controlCommandPublisher->publish(hw_command_from_EtherCAT);
    }

    void 
    SerialComm::resetPacket()
    {
        //받은거 에코로 뿌려라
        //hw_variable_from_EtherCAT 그대로 패킷으로 보내버려
        hw_variable_to_EtherCAT = hw_variable_from_EtherCAT;

        hw_command_to_EtherCAT.lift_dist_min = hw_command_from_EtherCAT.lift_dist_min;
        hw_command_to_EtherCAT.lift_dist_max = hw_command_from_EtherCAT.lift_dist_max;
        hw_command_to_EtherCAT.motor_drive_error_clear = hw_command_from_EtherCAT.motor_drive_error_clear;
        hw_command_to_EtherCAT.motor_drive_op_mode = hw_command_from_EtherCAT.motor_drive_op_mode;

        hw_command_to_EtherCAT.motor_drive_on = false;
        hw_command_to_EtherCAT.lift_brake_on = false;
        hw_command_to_EtherCAT.wheel_brake_on = false;
        tx_packet_.controlCommand = 0;

    }
    
    void
    SerialComm::statesToPacket(comm_packet::TxPacket &tx_packet)
    {
        tx_packet.header = 0xaa;
        tx_packet.type = 0x10;
        tx_packet.counter = (tx_packet.counter + 1) & 0xff ;
        tx_packet.length = static_cast<unsigned char>(sizeof(tx_packet));

        tx_packet.reserved0 = 0x00;

        tx_packet.controlCommand = hw_command_to_EtherCAT.motor_drive_on |
                                    hw_command_to_EtherCAT.lift_brake_on << 1 |
                                    hw_command_to_EtherCAT.wheel_brake_on << 2 |
                                    hw_command_to_EtherCAT.motor_drive_error_clear <<3 |
                                    hw_command_to_EtherCAT.motor_drive_op_mode << 4 |
                                    hw_command_to_EtherCAT.steer_vel_control_mode << 5 |
                                    hw_command_to_EtherCAT.lift_vel_control_mode << 6 |
                                    hw_command_to_EtherCAT.lift_trq_control_mode << 7;

        tx_packet.wheelVelFL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.wheel_vel_fl)*1E+1) >> 8 );
        tx_packet.wheelVelFR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.wheel_vel_fr)*1E+1) >> 8 );
        tx_packet.wheelVelRL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.wheel_vel_rl)*1E+1) >> 8 );
        tx_packet.wheelVelRR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.wheel_vel_rr)*1E+1) >> 8 );

        tx_packet.steerVelFL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.steering_vel_fl)*1E+1) >> 8);
        tx_packet.steerVelFR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.steering_vel_fr)*1E+1) >> 8);
        tx_packet.steerVelRL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.steering_vel_rl)*1E+1) >> 8);
        tx_packet.steerVelRR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.steering_vel_rr)*1E+1) >> 8);

        tx_packet.steerPosFL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.steering_pos_fl)*1E+2) >> 8);
        tx_packet.steerPosFR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.steering_pos_fr)*1E+2) >> 8);
        tx_packet.steerPosRL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.steering_pos_rl)*1E+2) >> 8);
        tx_packet.steerPosRR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.steering_pos_rr)*1E+2) >> 8);

        tx_packet.liftVelFL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_vel_fl)*1E+1*m2mm) >> 8);
        tx_packet.liftVelFR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_vel_fr)*1E+1*m2mm) >> 8);
        tx_packet.liftVelRL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_vel_rl)*1E+1*m2mm) >> 8);
        tx_packet.liftVelRR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_vel_rr)*1E+1*m2mm) >> 8);
        tx_packet.liftPosFL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_pos_fl)*1E+1*m2mm) >> 8);
        tx_packet.liftPosFR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_pos_fr)*1E+1*m2mm) >> 8);
        tx_packet.liftPosRL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_pos_rl)*1E+1*m2mm) >> 8);
        tx_packet.liftPosRR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_pos_rr)*1E+1*m2mm) >> 8);
        tx_packet.liftTrqFL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_trq_fl)*1E+1*m2mm) >> 8);
        tx_packet.liftTrqFR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_trq_fr)*1E+1*m2mm) >> 8);
        tx_packet.liftTrqRL_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_trq_rl)*1E+1*m2mm) >> 8);
        tx_packet.liftTrqRR_H = 0xff & (static_cast<int16_t>((hw_variable_to_EtherCAT.lift_trq_rr)*1E+1*m2mm) >> 8);

        tx_packet.wheelAcc_H = 0xff & (static_cast<int16_t>(hw_command_to_EtherCAT.wheel_acc_limit*1E+1) >> 8);
        tx_packet.steerAcc_H = 0xff & (static_cast<int16_t>(hw_command_to_EtherCAT.steering_acc_limit*1E+1) >> 8);
        tx_packet.liftAcc_H = 0xff & (static_cast<int16_t>(hw_command_to_EtherCAT.lift_acc_limit*1E+1*m2mm) >> 8);
//------------------------------------------------------------------------------------------------------------
        tx_packet.wheelVelFL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.wheel_vel_fl)*1E+1);
        tx_packet.wheelVelFR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.wheel_vel_fr)*1E+1);
        tx_packet.wheelVelRL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.wheel_vel_rl)*1E+1);
        tx_packet.wheelVelRR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.wheel_vel_rr)*1E+1);

        tx_packet.steerVelFL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.steering_vel_fl)*1E+1);
        tx_packet.steerVelFR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.steering_vel_fr)*1E+1);
        tx_packet.steerVelRL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.steering_vel_rl)*1E+1);
        tx_packet.steerVelRR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.steering_vel_rr)*1E+1);

        tx_packet.steerPosFL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.steering_pos_fl)*1E+2);
        tx_packet.steerPosFR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.steering_pos_fr)*1E+2);
        tx_packet.steerPosRL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.steering_pos_rl)*1E+2);
        tx_packet.steerPosRR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.steering_pos_rr)*1E+2);

        tx_packet.liftVelFL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_vel_fl)*1E+1*m2mm);
        tx_packet.liftVelFR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_vel_fr)*1E+1*m2mm);
        tx_packet.liftVelRL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_vel_rl)*1E+1*m2mm);
        tx_packet.liftVelRR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_vel_rr)*1E+1*m2mm);
        tx_packet.liftPosFL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_pos_fl)*1E+1*m2mm);
        tx_packet.liftPosFR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_pos_fr)*1E+1*m2mm);
        tx_packet.liftPosRL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_pos_rl)*1E+1*m2mm);
        tx_packet.liftPosRR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_pos_rr)*1E+1*m2mm);
        tx_packet.liftTrqFL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_trq_fl)*1E+1*m2mm);
        tx_packet.liftTrqFR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_trq_fr)*1E+1*m2mm);
        tx_packet.liftTrqRL_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_trq_rl)*1E+1*m2mm);
        tx_packet.liftTrqRR_L = 0xff & static_cast<int16_t>((hw_variable_to_EtherCAT.lift_trq_rr)*1E+1*m2mm);

        tx_packet.wheelAcc_L = 0xff & static_cast<int16_t>(hw_command_to_EtherCAT.wheel_acc_limit*1E+1);
        tx_packet.steerAcc_L = 0xff & static_cast<int16_t>(hw_command_to_EtherCAT.steering_acc_limit*1E+1);
        tx_packet.liftAcc_L = 0xff & static_cast<int16_t>(hw_command_to_EtherCAT.lift_acc_limit*1E+1*m2mm);


        tx_packet.liftDistMin = static_cast<int8_t>(hw_command_to_EtherCAT.lift_dist_min*1E+1*m2mm);
        tx_packet.liftDistMax = static_cast<int8_t>(hw_command_to_EtherCAT.lift_dist_max*1E+1*m2mm);

        tx_packet.imuRoll = static_cast<int8_t>(imu_variable_to_EtherCAT.roll*1E+2*deg2rad);
        tx_packet.imuPitch = static_cast<int8_t>(imu_variable_to_EtherCAT.pitch*1E+2*deg2rad);
        tx_packet.imuYaw = static_cast<int8_t>(imu_variable_to_EtherCAT.yaw*1E+2*deg2rad);

        tx_packet.footer = static_cast<unsigned char>(0x55);

        // printTxPacket(tx_packet);
    }

    template <typename T>
    void
    SerialComm::updateState(double &dest,
                                T &rx_packet_elem,
                                int exponent)
    {
        dest = static_cast<double>(rx_packet_elem) * pow(10, exponent);
    }


} // end namespace serial_comm

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<serial_comm::SerialComm>();

    // Run the node
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
