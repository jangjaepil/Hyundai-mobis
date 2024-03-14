#include "serial_comm/serial/serial_logger.hpp"

namespace plt = matplotlibcpp;

namespace serial_comm
{
    SerialLogger::SerialLogger() : Node("serialLogger")
    {
        getControlLogger = create_subscription<hw_msgs::msg::Control>(
            "Robot_Variable", 10, std::bind(&SerialLogger::getVariableCallback, this, std::placeholders::_1));

        setControlLogger = create_subscription<hw_msgs::msg::Control>(
            "Control_Variable", 10, std::bind(&SerialLogger::setVariableCallback, this, std::placeholders::_1));

        imuLogger = create_subscription<imu_msgs::msg::Imu>(
            "Imu_Variable", 10, std::bind(&SerialLogger::imuCallback, this, std::placeholders::_1));

        //  Get current time to use in the filename
        std::string save_path = "/home/harco/comm_ws/logging/";
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
        filename_ = save_path + "output_" + ss.str() + ".csv";
        // filename_idx = "/home/harco/comm_ws/logging/Imu_breakOn.csv";
        // Open the CSV file for writing
        csv_file_.open(filename_, std::ios::out | std::ios::trunc);

        // Write header to CSV file
        csv_file_ << "Timestamp,wheelVelFL,wheelVelFR,wheelVelRL,wheelVelRR"
        << ",steerPosFL,steerPosFR,steerPosRL,steerPosRR"
        << ",steerVelFL,steerVelFR,steerVelRL,steerVelRR"
        << ",liftPosFL,liftPosFR,liftPosRL,liftPosRR"
        << ",liftVelFL,liftVelFR,liftVelRL,liftVelRR"
        << ",liftTrqFL,liftTrqFR,liftTrqRL,liftTrqRR"
        << ",imuRoll,imuPitch,imuYaw"
        << ",imuRollRate,imuPitchRate,imuYawRate"        
        << ",wheelVelFL(set),wheelVelFR(set),wheelVelRL(set),wheelVelRR(set)"
        << ",steerPosFL(set),steerPosFR(set),steerPosRL(set),steerPosRR(set)"
        << ",steerVelFL(set),steerVelFR(set),steerVelRL(set),steerVelRR(set)"
        << ",liftPosFL(set),liftPosFR(set),liftPosRL(set),liftPosRR(set)"
        << ",liftVelFL(set),liftVelFR(set),liftVelRL(set),liftVelRR(set)"
        << ",liftTrqFL(set),liftTrqFR(set),liftTrqRL(set),liftTrqRR(set)"
        << std::endl;

    }

    SerialLogger::~SerialLogger()
    {
        readCSVFile(filename_);
        // readCSVFile(filename_idx);
    }

    void
    SerialLogger::getVariableCallback(const hw_msgs::msg::Control::SharedPtr send_variable)
    {
        hw_variable_from_EtherCAT = *send_variable;
        // 여기다가 csv 업데이트 하는 부분 추가
        updateCSVFile();

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
    SerialLogger::setVariableCallback(const hw_msgs::msg::Control::SharedPtr send_variable)
    {
        hw_variable_to_EtherCAT = *send_variable;
    }

    void
    SerialLogger::imuCallback(const imu_msgs::msg::Imu::SharedPtr send_variable)
    {
        imu_variable_to_EtherCAT = *send_variable;
    }
    
    void
    SerialLogger::updateCSVFile()
    {
        // Get current time to use in the timestamp
        // auto now = std::chrono::system_clock::now();
        // auto timestamp = std::chrono::system_clock::to_time_t(now);
        // auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch() % std::chrono::seconds(1));
        
        // // Write data to CSV file
        // std::stringstream ss;
        // ss << std::put_time(std::localtime(&timestamp), "%Y-%m-%d %H:%M:%S");
        // ss << "." << std::setw(3) << std::setfill('0') << milliseconds.count();
        // csv_file_ << ss.str() << ","
        
        // static 시작 시간 기록
        static std::chrono::high_resolution_clock::time_point lastCallTime = std::chrono::high_resolution_clock::now();

        // 현재 시간 기록
        std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

        // 시작 호출에서 현재 호출까지의 시간 간격 계산
        std::chrono::duration<double> elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastCallTime);


        csv_file_ << elapsedTime.count() << ","
                    << hw_variable_from_EtherCAT.wheel_vel_fl << "," << hw_variable_from_EtherCAT.wheel_vel_fr << "," << hw_variable_from_EtherCAT.wheel_vel_rl << "," << hw_variable_from_EtherCAT.wheel_vel_rr << ","
                    << hw_variable_from_EtherCAT.steering_pos_fl << "," << hw_variable_from_EtherCAT.steering_pos_fr << "," << hw_variable_from_EtherCAT.steering_pos_rl << "," << hw_variable_from_EtherCAT.steering_pos_rr << ","
                    << hw_variable_from_EtherCAT.steering_vel_fl << "," << hw_variable_from_EtherCAT.steering_vel_fr << "," << hw_variable_from_EtherCAT.steering_vel_rl << "," << hw_variable_from_EtherCAT.steering_vel_rr << ","
                    << hw_variable_from_EtherCAT.lift_pos_fl << "," << hw_variable_from_EtherCAT.lift_pos_fr << "," << hw_variable_from_EtherCAT.lift_pos_rl << "," << hw_variable_from_EtherCAT.lift_pos_rr << ","
                    << hw_variable_from_EtherCAT.lift_vel_fl << "," << hw_variable_from_EtherCAT.lift_vel_fr << "," << hw_variable_from_EtherCAT.lift_vel_rl << "," << hw_variable_from_EtherCAT.lift_vel_rr << ","
                    << hw_variable_from_EtherCAT.lift_trq_fl << "," << hw_variable_from_EtherCAT.lift_trq_fr << "," << hw_variable_from_EtherCAT.lift_trq_rl << "," << hw_variable_from_EtherCAT.lift_trq_rr << ","
                    << imu_variable_to_EtherCAT.roll << "," << imu_variable_to_EtherCAT.pitch << "," << imu_variable_to_EtherCAT.yaw  << ","
                    << imu_variable_to_EtherCAT.roll_rate << "," << imu_variable_to_EtherCAT.pitch_rate << "," << imu_variable_to_EtherCAT.yaw_rate  << ","
                    << hw_variable_to_EtherCAT.wheel_vel_fl << "," << hw_variable_to_EtherCAT.wheel_vel_fr << "," << hw_variable_to_EtherCAT.wheel_vel_rl << "," << hw_variable_to_EtherCAT.wheel_vel_rr << ","
                    << hw_variable_to_EtherCAT.steering_pos_fl << "," << hw_variable_to_EtherCAT.steering_pos_fr << "," << hw_variable_to_EtherCAT.steering_pos_rl << "," << hw_variable_to_EtherCAT.steering_pos_rr << ","
                    << hw_variable_to_EtherCAT.steering_vel_fl << "," << hw_variable_to_EtherCAT.steering_vel_fr << "," << hw_variable_to_EtherCAT.steering_vel_rl << "," << hw_variable_to_EtherCAT.steering_vel_rr << ","
                    << hw_variable_to_EtherCAT.lift_pos_fl << "," << hw_variable_to_EtherCAT.lift_pos_fr << "," << hw_variable_to_EtherCAT.lift_pos_rl << "," << hw_variable_to_EtherCAT.lift_pos_rr << ","
                    << hw_variable_to_EtherCAT.lift_vel_fl << "," << hw_variable_to_EtherCAT.lift_vel_fr << "," << hw_variable_to_EtherCAT.lift_vel_rl << "," << hw_variable_to_EtherCAT.lift_vel_rr << ","
                    << hw_variable_to_EtherCAT.lift_trq_fl << "," << hw_variable_to_EtherCAT.lift_trq_fr << "," << hw_variable_to_EtherCAT.lift_trq_rl << "," << hw_variable_to_EtherCAT.lift_trq_rr 
                    << std::endl;
    }

    void
    SerialLogger::readCSVFile(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Failed to open CSV file" << std::endl;
            return;
        }

        // Read the first line to get the headers
        std::string header;
        std::getline(file, header);
        std::istringstream headerStream(header);
        std::string column;

        // Track column names and their index
        std::map<std::string, size_t> columnMap;
        size_t index = 0;

        while (std::getline(headerStream, column, ','))
        {
            columnMap[column] = index++;
        }

        // Extract data based on column name
        std::vector<std::vector<double>> extractedData;
        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream lineStream(line);
            std::string cell;
            std::vector<double> rowData;

            while (std::getline(lineStream, cell, ','))
            {
                rowData.push_back(std::stod(cell));
            }

            extractedData.push_back(rowData);
        }


        std::vector<size_t> columnIndex;
        columnIndex.push_back(columnMap["Timestamp"]);

        columnIndex.push_back(columnMap["wheelVelFL"]);
        columnIndex.push_back(columnMap["wheelVelFR"]);
        columnIndex.push_back(columnMap["wheelVelRL"]);
        columnIndex.push_back(columnMap["wheelVelRR"]);

        columnIndex.push_back(columnMap["steerVelFL"]);
        columnIndex.push_back(columnMap["steerVelFR"]);
        columnIndex.push_back(columnMap["steerVelRL"]);
        columnIndex.push_back(columnMap["steerVelRR"]);

        columnIndex.push_back(columnMap["liftVelFL"]);
        columnIndex.push_back(columnMap["liftVelFR"]);
        columnIndex.push_back(columnMap["liftVelRL"]);
        columnIndex.push_back(columnMap["liftVelRR"]);

        columnIndex.push_back(columnMap["wheelVelFL(set)"]);
        columnIndex.push_back(columnMap["wheelVelFR(set)"]);
        columnIndex.push_back(columnMap["wheelVelRL(set)"]);
        columnIndex.push_back(columnMap["wheelVelRR(set)"]);

        columnIndex.push_back(columnMap["steerVelFL(set)"]);
        columnIndex.push_back(columnMap["steerVelFR(set)"]);
        columnIndex.push_back(columnMap["steerVelRL(set)"]);
        columnIndex.push_back(columnMap["steerVelRR(set)"]);

        columnIndex.push_back(columnMap["liftVelFL(set)"]);
        columnIndex.push_back(columnMap["liftVelFR(set)"]);
        columnIndex.push_back(columnMap["liftVelRL(set)"]);
        columnIndex.push_back(columnMap["liftVelRR(set)"]);
        
        columnIndex.push_back(columnMap["steerPosFL"]);
        columnIndex.push_back(columnMap["steerPosFR"]);
        columnIndex.push_back(columnMap["steerPosRL"]);
        columnIndex.push_back(columnMap["steerPosRR"]);

        columnIndex.push_back(columnMap["steerPosFL(set)"]);
        columnIndex.push_back(columnMap["steerPosFR(set)"]);
        columnIndex.push_back(columnMap["steerPosRL(set)"]);
        columnIndex.push_back(columnMap["steerPosRR(set)"]);

        columnIndex.push_back(columnMap["imuRoll"]);
        columnIndex.push_back(columnMap["imuPitch"]);
        columnIndex.push_back(columnMap["imuYaw"]);

        columnIndex.push_back(columnMap["imuRollRate"]);
        columnIndex.push_back(columnMap["imuPitchRate"]);
        columnIndex.push_back(columnMap["imuYawRate"]);

        columnIndex.push_back(columnMap["liftPosFL"]);
        columnIndex.push_back(columnMap["liftPosFR"]);
        columnIndex.push_back(columnMap["liftPosRL"]);
        columnIndex.push_back(columnMap["liftPosRR"]);

        // Extract the column of the desired data
        std::vector<std::vector<double>> selectedData;

        for (size_t idx = 0; idx < columnIndex.size(); ++idx)
        {
            size_t columnIdx = columnIndex[idx];
            std::vector<double> dataContainer;

            for (const auto &row : extractedData)
            {
                // RCLCPP_INFO(this->get_logger(),"columnIndex is %ld ", columnIdx);

                dataContainer.push_back(row[columnIdx]);
            }
            selectedData.push_back(dataContainer);
        }


        // RCLCPP_INFO(this->get_logger(),"x size is %ld ", selectedData[0].size());
        // RCLCPP_INFO(this->get_logger(),"y size is %ld ", selectedData[1].size());

        // // Plot the selected data
        // plt::figure_size(1200, 780);
        // plt::plot(selectedData[0],selectedData[1],{{"c", "red"},{"label","wheel_FL"}});
        // plt::plot(selectedData[0],selectedData[2],{{"c", "green"},{"label","wheel_FR"}});
        // plt::plot(selectedData[0],selectedData[3],{{"c", "magenta"},{"label","wheel_RL"}});
        // plt::plot(selectedData[0],selectedData[4],{{"c", "blue"},{"label","wheel_RR"}});
        // plt::plot(selectedData[0],selectedData[13],{{"c", "red"},{"ls", "--"}    ,{"label","wheel_FL(set)"}});
        // plt::plot(selectedData[0],selectedData[14],{{"c", "green"},{"ls", "--"}  ,{"label","wheel_FR(set)"}});
        // plt::plot(selectedData[0],selectedData[15],{{"c", "magenta"},{"ls", "--"},{"label","wheel_RL(set)"}});
        // plt::plot(selectedData[0],selectedData[16],{{"c", "blue"},{"ls", "--"} ,{"label","wheel_RR(set)"}});
        // plt::xlabel("time[s]");
        // plt::ylabel("wheel Velocity(rad/s)");
        // plt::title("wheel Velocity");
        // plt::ylim(-20,20);
        // plt::legend();
        // plt::grid(true);

        // plt::figure_size(1200, 780);
        // plt::plot(selectedData[0],selectedData[1],{{"c", "red"},{"label","wheel_FL"}});
        // plt::plot(selectedData[0],selectedData[13],{{"c", "red"},{"ls", "--"}    ,{"label","wheel_FL(set)"}});
        // plt::xlabel("time[s]");
        // plt::ylabel("wheel Velocity(rad/s)");
        // plt::title("wheel Velocity FL");
        // plt::ylim(-20,20);
        // plt::legend();
        // plt::grid(true);

        // plt::figure_size(1200, 780);
        // plt::plot(selectedData[0], selectedData[5],{{"c", "red"},{"label","steer_FL"}});
        // plt::plot(selectedData[0], selectedData[6],{{"c", "green"},{"label","steer_FR"}});
        // plt::plot(selectedData[0], selectedData[7],{{"c", "magenta"},{"label","steer_RL"}});
        // plt::plot(selectedData[0], selectedData[8],{{"c", "blue"},{"label","steer_RR"}});
        // plt::plot(selectedData[0], selectedData[17],{{"c", "red"},{"ls", "--"}    ,{"label","steer_FL(set)"}});
        // plt::plot(selectedData[0], selectedData[18],{{"c", "green"},{"ls", "--"}  ,{"label","steer_FR(set)"}});
        // plt::plot(selectedData[0], selectedData[19],{{"c", "magenta"},{"ls", "--"},{"label","steer_RL(set)"}});
        // plt::plot(selectedData[0], selectedData[20],{{"c", "blue"},{"ls", "--"} ,{"label","steer_RR(set)"}});
        // plt::xlabel("time[s]");
        // plt::ylabel("steer Velocity(rad/s)");
        // plt::title("steer Velocity");
        // plt::ylim(-2,2);
        // plt::legend();
        // plt::grid(true);

        // plt::figure_size(1200, 780);
        // plt::plot(selectedData[0], selectedData[5],{{"c", "red"},{"label","steer_FL"}});
        // plt::plot(selectedData[0], selectedData[17],{{"c", "red"},{"ls", "--"}    ,{"label","steer_FL(set)"}});
        // plt::xlabel("time[s]");
        // plt::ylabel("steer Velocity(rad/s)");
        // plt::title("steer Velocity FL");
        // plt::ylim(-2,2);
        // plt::legend();
        // plt::grid(true);

        plt::figure_size(1200, 780);
        plt::plot(selectedData[0], selectedData[9] ,{{"c", "red"}    ,{"label","lift_FL"}});
        plt::plot(selectedData[0], selectedData[10],{{"c", "green"}  ,{"label","lift_FR"}});
        plt::plot(selectedData[0], selectedData[11],{{"c", "magenta"},{"label","lift_RL"}});
        plt::plot(selectedData[0], selectedData[12],{{"c", "blue"} ,{"label","lift_RR"}});
        plt::plot(selectedData[0], selectedData[21] ,{{"c", "red"},{"ls", "--"}    ,{"label","lift_FL(set)"}});
        plt::plot(selectedData[0], selectedData[22],{{"c", "green"},{"ls", "--"}  ,{"label","lift_FR(set)"}});
        plt::plot(selectedData[0], selectedData[23],{{"c", "magenta"},{"ls", "--"},{"label","lift_RL(set)"}});
        plt::plot(selectedData[0], selectedData[24],{{"c", "blue"},{"ls", "--"} ,{"label","lift_RR(set)"}});
        plt::xlabel("time[s]");
        plt::ylabel("lift Velocity(rad/s)");
        plt::title("lift Velocity");
        plt::ylim(-0.5,0.5);
        plt::legend();
        plt::grid(true);

        plt::figure_size(1200, 780);
        plt::plot(selectedData[0], selectedData[9] ,{{"c", "red"}    ,{"label","lift_FL"}});
        plt::plot(selectedData[0], selectedData[21] ,{{"c", "red"},{"ls", "--"}    ,{"label","lift_FL(set)"}});
        plt::xlabel("time[s]");
        plt::ylabel("lift Velocity(rad/s)");
        plt::title("lift Velocity FL");
        plt::ylim(-0.5,0.5);
        plt::legend();
        plt::grid(true);

        // plt::figure_size(1200, 780);
        // plt::plot(selectedData[0],selectedData[1], {{"label","wheel_FL"}});
        // plt::plot(selectedData[0],selectedData[2], {{"label","wheel_FR"}});
        // plt::plot(selectedData[0],selectedData[3], {{"label","wheel_RL"}});
        // plt::plot(selectedData[0],selectedData[4], {{"label","wheel_RR"}});        
        // plt::plot(selectedData[0], selectedData[5], {{"label","steer_FL"}});
        // plt::plot(selectedData[0], selectedData[6], {{"label","steer_FR"}});
        // plt::plot(selectedData[0], selectedData[7], {{"label","steer_RL"}});
        // plt::plot(selectedData[0], selectedData[8], {{"label","steer_RR"}});
        // plt::plot(selectedData[0], selectedData[9] , {{"label","lift_FL"}});
        // plt::plot(selectedData[0], selectedData[10], {{"label","lift_FR"}});
        // plt::plot(selectedData[0], selectedData[11], {{"label","lift_RL"}});
        // plt::plot(selectedData[0], selectedData[12], {{"label","lift_RR"}});
        // plt::xlabel("time[s]");
        // plt::ylabel("All Velocity(rad/s)");
        // plt::title("All Velocity");
        // plt::legend();
        // plt::grid(true);


        // plt::figure_size(1200, 780);
        // plt::plot(selectedData[0], selectedData[25],{{"c", "red"},{"label","steer_FL"}});
        // plt::plot(selectedData[0], selectedData[26],{{"c", "green"},{"label","steer_FR"}});
        // plt::plot(selectedData[0], selectedData[27],{{"c", "magenta"},{"label","steer_RL"}});
        // plt::plot(selectedData[0], selectedData[28],{{"c", "blue"},{"label","steer_RR"}});
        // plt::plot(selectedData[0], selectedData[29],{{"c", "red"},{"ls", "--"}    ,{"label","steer_FL(set)"}});
        // plt::plot(selectedData[0], selectedData[30],{{"c", "green"},{"ls", "--"}  ,{"label","steer_FR(set)"}});
        // plt::plot(selectedData[0], selectedData[31],{{"c", "magenta"},{"ls", "--"},{"label","steer_RL(set)"}});
        // plt::plot(selectedData[0], selectedData[32],{{"c", "blue"},{"ls", "--"} ,{"label","steer_RR(set)"}});
        // plt::xlabel("time[s]");
        // plt::ylabel("steer Position(rad)");
        // plt::title("Steer Position");
        // plt::ylim(-2,2);
        // plt::legend();
        // plt::grid(true);


        // plt::figure_size(1200, 780);
        // plt::plot(selectedData[0], selectedData[33],{{"c", "red"},{"label","imu_Roll"}});
        // plt::plot(selectedData[0], selectedData[34],{{"c", "green"},{"label","imu_Pitch"}});
        // plt::plot(selectedData[0], selectedData[35],{{"c", "magenta"},{"label","imu_Yaw"}});
        // plt::xlabel("time[s]");
        // plt::ylabel("imu Value(degree)");
        // plt::title("imu Value");
        // plt::ylim(-180,180);
        // plt::legend();
        // plt::grid(true);

        plt::figure_size(1200, 780);
        plt::plot(selectedData[0], selectedData[36],{{"c", "red"},{"label","imu_Roll_rate"}});
        plt::plot(selectedData[0], selectedData[37],{{"c", "green"},{"label","imu_Pitch_rate"}});
        plt::plot(selectedData[0], selectedData[38],{{"c", "magenta"},{"label","imu_Yaw_rate"}});
        plt::xlabel("time[s]");
        plt::ylabel("imu rate Value(degree/s)");
        plt::title("imu rate Value");
        plt::ylim(-100,100);
        plt::legend();
        plt::grid(true);

        plt::figure_size(1200, 780);
        plt::plot(selectedData[0], selectedData[39],{{"c", "red"},{"label","lift_pos_fl"}});
        plt::plot(selectedData[0], selectedData[40],{{"c", "green"},{"label","lift_pos_fr"}});
        plt::plot(selectedData[0], selectedData[41],{{"c", "magenta"},{"label","lift_pos_rl"}});
        plt::plot(selectedData[0], selectedData[42],{{"c", "blue"},{"label","lift_pos_rr"}});
        plt::xlabel("time[s]");
        plt::ylabel("lift pos (m)");
        plt::title("lift pos (m)");
        plt::ylim(-0.2, 0.2);
        plt::legend();
        plt::grid(true);

        plt::show();

        // plt::save("/home/jjw/comm_ws/logging/plotTest.svg");
    }

} // end namespace serial_comm

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<serial_comm::SerialLogger>();

    // Run the node
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
