
#include "serial_comm/serial/imu_serial_comm.hpp"
#define m2mm 1E+3
#define mm2m 1E-3
using namespace std::chrono_literals;

namespace serial_comm
{

    ImuSerialComm::ImuSerialComm() : Node("ImuPublisher")
    {
        onInit();

        imuPublisher = this->create_publisher<imu_msgs::msg::Imu>("Imu_Variable", 10);
        timer_ = this->create_wall_timer(0.02s, std::bind(&ImuSerialComm::timerCounter, this));

        onConfigure();
    }

    ImuSerialComm::~ImuSerialComm()
    {
        onDeactivate();
    }

    void
    ImuSerialComm::onInit()
    {

        isReadOnly = false;
        port_name = "/dev/ttyUSB_Imu";
        baud_rate = 115200;

        auto flowControl = drivers::serial_driver::FlowControl::NONE;
        auto parityBit = drivers::serial_driver::Parity::NONE;
        auto stopBit = drivers::serial_driver::StopBits::ONE;
        device_config = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, flowControl, parityBit, stopBit);

        drivers::common::IoContext *ctx = new drivers::common::IoContext(2);
        drivers::serial_driver::SerialDriver *pa1 = new drivers::serial_driver::SerialDriver(*ctx);
        serial_driver.reset(pa1);

        memset(stack_buffer, 0, buff_size);
        
        RCLCPP_INFO(this->get_logger(), "Serial Device Configuration Complete...!!");
    }

    bool
    ImuSerialComm::onConfigure()
    {
        try
        {
            serial_driver->init_port(port_name, *device_config);

            if (!serial_driver->port()->is_open())
            {
                serial_driver->port()->open();
                serial_driver->port()->async_receive(
                    std::bind(&ImuSerialComm::receiveCallback, this, std::placeholders::_1, std::placeholders::_2));
                
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
    ImuSerialComm::onDeactivate()
    {
        serial_driver->port()->close();
    }

    bool
    ImuSerialComm::read()
    {   
        // updateHwStates(rx_packet_);
        return true;
    }

    void
    ImuSerialComm::receiveCallback(const std::vector<uint8_t> &buffer,
                                const size_t &bytes_transferred)
    {
        // static std::chrono::high_resolution_clock::time_point lastCallTime = std::chrono::high_resolution_clock::now();

        // // 현재 시간 기록
        // std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

        // // 이전 호출에서 현재 호출까지의 시간 간격 계산
        // std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastCallTime);

        // // 실행 주기 출력
        // std::cout << "Function called. Elapsed time: " << elapsedTime.count() << " seconds." << std::endl;

        // // 현재 시간을 마지막 호출 시간으로 설정
        // lastCallTime = currentTime;

        // for(size_t i = 0; i < bytes_transferred; i++)
        // {
        //     std::cout << "buffer i : "<< buffer[i] << std::endl;
        // }

        handleRxData(buffer, bytes_transferred);
    }

    bool
    ImuSerialComm::handleRxData(const std::vector<uint8_t> &buffer,
                                const size_t &bytes_transferred)
    {
        bool res = false;

        //Checking NULL character
        // RCLCPP_INFO(this->get_logger(),"%d : %d", buffer[bytes_transferred - 2], buffer[bytes_transferred - 1]);
        

        if(buffer[bytes_transferred - 2] == static_cast<uint8_t>(ASCII_CARIAGERETURN)
            && buffer[bytes_transferred - 1] == static_cast<uint8_t>(ASCII_LINEFEED))
        {
            // RCLCPP_INFO(this->get_logger(),"dd");
            std::mutex mtx;
            std::unique_lock<std::mutex> lock(mtx);

            for(size_t i = 0; i < bytes_transferred; ++i)
            {

                if(buffer[i] == static_cast<uint8_t>(ASCII_COMMA))
                {
                    // std::cout << "flag111" << std::endl;
                    packet.push_back(atof(stack_buffer));
                    memset(stack_buffer, 0, buff_size);
                }

                else if(buffer[i] == static_cast<uint8_t>(ASCII_CARIAGERETURN))
                {
                    // std::cout << "flag222" << std::endl;
                    packet.push_back(atof(stack_buffer));
                    memset(stack_buffer, 0, buff_size);
                    break;
                }
                else
                {
                    // std::cout << "flag333" << std::endl;
                    const char tmp = buffer[i];
                    std::strncat(stack_buffer, &tmp, 1);
                }
            }
            lock.unlock();
        }

        else{
            RCLCPP_WARN(this->get_logger()," There is no end character ...!! ");
        }
        
        // for(auto iter : packet)
        // {
        //     std::cout << " packet : " << iter << std::endl;
        // }
        // std::cout << std::endl;

        packet.clear();
        return res;
    }

    void
    ImuSerialComm::timerCounter()
    {
        imu_msgs::msg::Imu imu;
        imu.roll_rate = packet[1];
        imu.pitch_rate = packet[2];
        imu.yaw_rate = packet[3];

        imu.roll = packet[4];
        imu.pitch = packet[5];
        imu.yaw = packet[6];

        imuPublisher->publish(imu);
    }

} // end namespace serial_comm

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<serial_comm::ImuSerialComm>();

    // Run the node
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
