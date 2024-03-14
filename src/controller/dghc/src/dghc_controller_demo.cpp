#include "dghc_controller_demo.hpp"
#include "GHCProjections.hpp"
#include "LowPassFilter.hpp"
#include "RequiredHeaders.hpp"

using namespace std::literals::chrono_literals;
dghc_controller::dghc_controller() :Node("dghc_controller")
{   
    
    //chain부분
    std::string base_link = "base_link"; 
    std::string end_link = "wrist_3_link";
    urdf_filename = ament_index_cpp::get_package_share_directory("mobis_description") + "/urdf/ur5e_.urdf"; // 윗 부분이 경로 인식이 잘 안되서 상대경로로 지정
    RCLCPP_INFO(rclcpp::get_logger("logger_name"), "URDF file path: %s", urdf_filename.c_str());
    urdf::Model model;
    if (!model.initFile(urdf_filename)) {
        RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Failed to initialize URDF model.");   // URDF 파일로부터 초기화 실패 시 에러 메시지 출력
        return;
    }
    //KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Failed to construct KDL tree from URDF.");  // URDF 모델로부터 KDL 트리 생성 실패 시 에러 메시지 출력
        return;
    }
    // KDL::Chain chain;
    if (!tree.getChain(base_link, end_link, chain))
    {
        RCLCPP_ERROR(rclcpp::get_logger("logger_name"), "Failed to get chain from tree"); // 트리로부터 체인을 얻지 못한 경우 에러 메시지 출력
    }
    joint_size = chain.getNrOfJoints();
    RCLCPP_INFO(rclcpp::get_logger("joint_size_logger"), "Joint size: %d", joint_size); // joint_size와 n 값이 다르면 chain에서 오류가 발생 -> 이를 확인하기 위함
   
    timer_ = this->create_wall_timer(1ms, std::bind(&dghc_controller::timer_callback, this));
    joint_states_sub_  = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 100, std::bind(&dghc_controller::joint_states_callback, this, std::placeholders::_1));
    mobile_joint_states_sub_ = this->create_subscription<hw_msgs::msg::Control>("/Robot_Variable",10, std::bind(&dghc_controller::mobile_joint_states_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS(), std::bind(&dghc_controller::imu_callback, this, std::placeholders::_1));    
    desired_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/end_desired_pose", 100, std::bind(&dghc_controller::desired_pose_callback, this, std::placeholders::_1));
    joint_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 100);
    control_pub = this->create_publisher<hw_msgs::msg::Control>("/Control_Variable", 10);
    estimated_end_pose_pub = this -> create_publisher<geometry_msgs::msg::Pose>("/estimated_end_pose",100);
    desired_mobile_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>("/mobile_desired_pose", 100, std::bind(&dghc_controller::desired_mobile_pose_callback, this, std::placeholders::_1));
    desired_mobile_parameters_sub = this->create_subscription<geometry_msgs::msg::Pose>("/mobile_desired_parameters", 100, std::bind(&dghc_controller::desired_mobile_parameters_callback, this, std::placeholders::_1));
    ft_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/force_torque_sensor_broadcaster/wrench", 100, std::bind(&dghc_controller::ft_callback, this, std::placeholders::_1));
    wrench_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/filtered_wrench", 100); 
    obs_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/obs_pos_vel", 100, std::bind(&dghc_controller::obs_callback, this, std::placeholders::_1));
    
        

 
    Jacobian_mobile <<  0.25,0,0.25,0,0.25,0,0.25,0,0,0,0,0, //ref :mobile_base     x
                        0,0.25,0,0.25,0,0.25,0,0.25,0,0,0,0,
                        0,0,0,0,0,0,0,0, -0.25   , -0.25    , -0.25    , -0.25,
                        0,0,0,0,0,0,0,0,1/(4*Ly),-1/(4*Ly),-1/(4*Ly),1/(4*Ly),                    //f_r,f_l,r_l,r_r
                        0,0,0,0,0,0,0,0,1/(4*Lx),1/(4*Lx),-1/(4*Lx),-1/(4*Lx), 
                        1/(8*Ly),1/(8*Lx),-1/(8*Ly),1/(8*Lx),-1/(8*Ly),-1/(8*Lx),1/(8*Ly),-1/(8*Lx),0,0,0,0;

    b << 100,0,0, //virtual impedance force parameters for mobile pose     
         0,100,0,
         0,0,100;

    k << 170,0,0,
         0,170,0,
         0,0,170;
}

void dghc_controller::timer_callback()
{
    
    
    if(fisrt_loop)
    {
        std_msgs::msg::Float64MultiArray mani_vel_msg;
        geometry_msgs::msg::Pose estimated_end_pose_msg;
        hw_msgs::msg::Control Control_msg;
        std_msgs::msg::Float64MultiArray wrench_msg;
           
        estimated_end_pose_msg.position.x =end_position(0);//thetalist(0);//X_kk(0)*0.001;
        estimated_end_pose_msg.position.y =end_position(1);//thetalist(1);//X_kk(1)*0.001;
        estimated_end_pose_msg.position.z =end_position(2);//thetalist(2);//X_kk(2)*0.001;
        estimated_end_pose_msg.orientation.x = end_quat.x();//thetalist(0)+ wheel_ro(0)+ wheel_ro_bias(0);
        estimated_end_pose_msg.orientation.y = end_quat.y();//thetalist(1)+ wheel_ro(1)+ wheel_ro_bias(1);
        estimated_end_pose_msg.orientation.z = end_quat.z();//thetalist(2)+ wheel_ro(2)+ wheel_ro_bias(2);
        estimated_end_pose_msg.orientation.w = end_quat.w();//thetalist(3)+ wheel_ro(3)+ wheel_ro_bias(3);        
        
        Control_msg.wheel_vel_fr = wheel_ew_vel_cmd(0)/wheel_radius;
        Control_msg.wheel_vel_fl = wheel_ew_vel_cmd(1)/wheel_radius;
        Control_msg.wheel_vel_rl = wheel_ew_vel_cmd(2)/wheel_radius; 
        Control_msg.wheel_vel_rr = wheel_ew_vel_cmd(3)/wheel_radius;
        
        Control_msg.steering_pos_fr = thetalist(0)+ wheel_ro(0)+ wheel_ro_bias(0);
        Control_msg.steering_pos_fl = thetalist(1)+ wheel_ro(1)+ wheel_ro_bias(1);
        Control_msg.steering_pos_rl = thetalist(2)+ wheel_ro(2)+ wheel_ro_bias(2);
        Control_msg.steering_pos_rr = thetalist(3)+ wheel_ro(3)+ wheel_ro_bias(3);
        
        Control_msg.lift_vel_fr = -wheel_pr_vel_cmd(0);
        Control_msg.lift_vel_fl = -wheel_pr_vel_cmd(1);
        Control_msg.lift_vel_rl = -wheel_pr_vel_cmd(2);
        Control_msg.lift_vel_rr = -wheel_pr_vel_cmd(3);
        
        mani_vel_msg.data = {mani_q_vel_cmd(0),mani_q_vel_cmd(1),mani_q_vel_cmd(2),mani_q_vel_cmd(3),mani_q_vel_cmd(4),mani_q_vel_cmd(5)};
        
        wrench_msg.data = {ForceTorque[0],ForceTorque[1], ForceTorque[2],ForceTorque[3],ForceTorque[4],ForceTorque[5]};
        
        if(init_ft_flag ==1) wrench_pub->publish(wrench_msg);
        control_pub->publish(Control_msg);
        joint_vel_pub->publish(mani_vel_msg);
        estimated_end_pose_pub->publish(estimated_end_pose_msg);
    }
}
void dghc_controller::obs_callback(const std_msgs::msg::Float64MultiArray& obs_DATA)
{   
    std::cout<<"obs_DATA.data.size()"<<std::endl<<obs_DATA.data.size()<<std::endl;
    obs_data.resize(obs_DATA.data.size());
    
    for(unsigned int i = 0;i<obs_DATA.data.size();i++)
    {
        obs_data(i) = obs_DATA.data[i];
    }

    init_obs_flag = 1;
}
void dghc_controller::ft_callback(const geometry_msgs::msg::WrenchStamped& ft_DATA)
{  
    ForceTorque_tmp(0) = ft_DATA.wrench.force.x;
    ForceTorque_tmp(1) = ft_DATA.wrench.force.y;
    ForceTorque_tmp(2) = ft_DATA.wrench.force.z;
    ForceTorque_tmp(3) = ft_DATA.wrench.torque.x;
    ForceTorque_tmp(4) = ft_DATA.wrench.torque.y;
    ForceTorque_tmp(5) = ft_DATA.wrench.torque.z;
    int count = 1000;
  
    if(init_bias_flag == 0)
    {
        if(bias_count< count)
        {

            tmp = tmp + ForceTorque_tmp;

        }
        else
        {
            bias = tmp/count;
            init_bias_flag = 1;

            lpf.reconfigureFilter(0.001,100,6); //deltime,cutoff freq,value dof
          
        }
        bias_count  = bias_count + 1;
    }
    else
    {   
        
       ForceTorque = lpf.update(ForceTorque_tmp - bias);
                 
        init_ft_flag = 1;
    }   
}

void dghc_controller::desired_mobile_parameters_callback(const geometry_msgs::msg::Pose& d_Mobile_Parameter_Data)
{

    k.diagonal()<<d_Mobile_Parameter_Data.position.x,d_Mobile_Parameter_Data.position.x,d_Mobile_Parameter_Data.position.x;
    b.diagonal()<<d_Mobile_Parameter_Data.position.y,d_Mobile_Parameter_Data.position.y,d_Mobile_Parameter_Data.position.y;
    init_parameters_flag = 1;    
}
void dghc_controller::desired_mobile_pose_callback(const geometry_msgs::msg::Pose& d_Mobile_Pose_Data)
{
    d_mobile_quat.x() = d_Mobile_Pose_Data.orientation.x;
    d_mobile_quat.y() = d_Mobile_Pose_Data.orientation.y;
    d_mobile_quat.z() = d_Mobile_Pose_Data.orientation.z;
    d_mobile_quat.w() = d_Mobile_Pose_Data.orientation.w;
}
//cartesian space desired position subscribe
void dghc_controller::desired_pose_callback(const geometry_msgs::msg::Pose& d_Pose_Data)
{
    d_end_position(0) = d_Pose_Data.position.x;
    d_end_position(1) = d_Pose_Data.position.y;
    d_end_position(2) = d_Pose_Data.position.z;
    // d_end_quat.x() = d_Pose_Data.orientation.x;
    // d_end_quat.y() = d_Pose_Data.orientation.y;
    // d_end_quat.z() = d_Pose_Data.orientation.z;
    // d_end_quat.w() = d_Pose_Data.orientation.w;
    //std::cout<<"sub d_position: "<<std::endl<<d_end_position<<std::endl;
}

void dghc_controller::joint_states_callback(const sensor_msgs::msg::JointState& JointState_Data)
{   
    //q
    q(0) = JointState_Data.position[5];
    q(1) = JointState_Data.position[0];
    q(2) = JointState_Data.position[1];
    q(3) = JointState_Data.position[2];
    q(4) = JointState_Data.position[3];
    q(5) = JointState_Data.position[4];
    //q_dot
    q_dot(0) = JointState_Data.velocity[5];
    q_dot(1) = JointState_Data.velocity[0];
    q_dot(2) = JointState_Data.velocity[1];
    q_dot(3) = JointState_Data.velocity[2];
    q_dot(4) = JointState_Data.velocity[3];
    q_dot(5) = JointState_Data.velocity[4];

    init_mani_joint_flag =1;
}
void dghc_controller::mobile_joint_states_callback(const hw_msgs::msg::Control::SharedPtr JointState_Data)
{       
    if(init_mobile_joint_flag == 0)
    {
        wheel_ro(0) = 0;             //   rotate(f_r,f_l,r_l,r_r),ew(f_r,f_l,r_l,r_r),prismatic(f_r,f_l,r_l,r_r), 
        wheel_ro(1) = 0;                 
        wheel_ro(2) = 0;                
        wheel_ro(3) = 0;                    
        
        wheel_ro_bias(0) =  JointState_Data->steering_pos_fr;
        wheel_ro_bias(1) =  JointState_Data->steering_pos_fl;
        wheel_ro_bias(2) =  JointState_Data->steering_pos_rl;
        wheel_ro_bias(3) =  JointState_Data->steering_pos_rr;
    }
    else
    {
        wheel_ro(0) = JointState_Data->steering_pos_fr - wheel_ro_bias(0);           //   rotate(f_r,f_l,r_l,r_r),ew(f_r,f_l,r_l,r_r),prismatic(f_r,f_l,r_l,r_r), 
        wheel_ro(1) = JointState_Data->steering_pos_fl - wheel_ro_bias(1);               
        wheel_ro(2) = JointState_Data->steering_pos_rl - wheel_ro_bias(2);              
        wheel_ro(3) = JointState_Data->steering_pos_rr - wheel_ro_bias(3);                  
        
    }
    
    // wheel_ew(0) = JointState_Data->wheel_pos_fr;
    // wheel_ew(1) = JointState_Data->wheel_pos_fl;
    // wheel_ew(2) = JointState_Data->wheel_pos_rl;
    // wheel_ew(3) = JointState_Data->wheel_pos_rr;
    wheel_ew<<0,0,0,0;    
    wheel_pr(0) = JointState_Data->lift_pos_fr;    
    wheel_pr(1) = JointState_Data->lift_pos_fl;
    wheel_pr(2) = JointState_Data->lift_pos_rl;    
    wheel_pr(3) = JointState_Data->lift_pos_rr;        
    allq<<wheel_ro,wheel_ew,-wheel_pr,q;


    wheel_ro_dot(0) = JointState_Data->steering_vel_fr; //   rotate(f_r,f_l,r_l,r_r),ew(f_r,f_l,r_l,r_r),prismatic(f_r,f_l,r_l,r_r), 
    wheel_ro_dot(1) = JointState_Data->steering_vel_fl;     
    wheel_ro_dot(2) = JointState_Data->steering_vel_rl;    
    wheel_ro_dot(3) = JointState_Data->steering_vel_rr;        
    
    wheel_ew_dot(0) = JointState_Data->wheel_vel_fr;
    wheel_ew_dot(1) = JointState_Data->wheel_vel_fl;
    wheel_ew_dot(2) = JointState_Data->wheel_vel_rl;
    wheel_ew_dot(3) = JointState_Data->wheel_vel_rr;        
    
    wheel_pr_dot(0) = -JointState_Data->lift_vel_fr;    
    wheel_pr_dot(1) = -JointState_Data->lift_vel_fl;
    wheel_pr_dot(2) = -JointState_Data->lift_vel_rl;    
    wheel_pr_dot(3) = -JointState_Data->lift_vel_rr;    
    
    init_mobile_joint_flag =1;

    allq_dot<<wheel_ro_dot,wheel_ew_dot*wheel_radius,wheel_pr_dot,q_dot;
      
}

void dghc_controller::imu_callback(const sensor_msgs::msg::Imu IMU_Data)
{
    mobile_quat.x() = IMU_Data.orientation.x;
    mobile_quat.y() = IMU_Data.orientation.y;
    mobile_quat.z() = IMU_Data.orientation.z;
    mobile_quat.w() = IMU_Data.orientation.w;
    wRm = mobile_quat.normalized().toRotationMatrix(); // radian
    wRm_e.block(0,0,3,3) = wRm;
    wRm_e.block(3,3,3,3) = wRm;
    mobile_twist(3) = IMU_Data.angular_velocity.x;
    mobile_twist(4) = IMU_Data.angular_velocity.y;
    mobile_twist(5) = IMU_Data.angular_velocity.z;
    
    mobile_acc<<IMU_Data.linear_acceleration.x,IMU_Data.linear_acceleration.y,IMU_Data.linear_acceleration.z;
    init_imu_flag = 1;
}

void dghc_controller::mobile_pose_estimation()
{
 //////////////////////////////////////////////////// ENCODER BASED POSE ESTIMATION //////////////////////////////////////////////////////////////////////////////////////////  
    if(init_step ==0)
    {
        last_estimate_time = rclcpp::Clock{}.now();   
    }

    double dt_estimate = (rclcpp::Clock{}.now() - last_estimate_time).seconds();
    last_estimate_time = rclcpp::Clock{}.now();
    Eigen::VectorXd v1 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd v2 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd v3 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd v4 = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd generalized_v = Eigen::VectorXd::Zero(12);
    
    
    v1(2) = wheel_pr_dot(0)*1000; //mm
    v2(2) = wheel_pr_dot(1)*1000;
    v3(2) = wheel_pr_dot(2)*1000;
    v4(2) = wheel_pr_dot(3)*1000;

    //std::cout<<"wheel_ew_dot: "<<std::endl<<wheel_ew_dot.transpose()<<std::endl;
    //std::cout<<"wheel_ro: "<<std::endl<<wheel_ro.transpose()<<std::endl;
    v1.block(0,0,2,1) << wheel_radius*1000*wheel_ew_dot(0)*cos(wheel_ro(0)),wheel_radius*1000*wheel_ew_dot(0)*sin(wheel_ro(0)); // mm/s
    v2.block(0,0,2,1) << wheel_radius*1000*wheel_ew_dot(1)*cos(wheel_ro(1)),wheel_radius*1000*wheel_ew_dot(1)*sin(wheel_ro(1));
    v3.block(0,0,2,1) << wheel_radius*1000*wheel_ew_dot(2)*cos(wheel_ro(2)),wheel_radius*1000*wheel_ew_dot(2)*sin(wheel_ro(2));
    v4.block(0,0,2,1) << wheel_radius*1000*wheel_ew_dot(3)*cos(wheel_ro(3)),wheel_radius*1000*wheel_ew_dot(3)*sin(wheel_ro(3));

    //std::cout<<"v1: "<<std::endl<<v1(1)<<std::endl;
    //std::cout<<"v2: "<<std::endl<<v2(1)<<std::endl;
    //std::cout<<"v3: "<<std::endl<<v3(1)<<std::endl;
    //std::cout<<"v4: "<<std::endl<<v4(1)<<std::endl;
    
    generalized_v<<v1(0),v1(1),v2(0),v2(1),v3(0),v3(1),v4(0),v4(1),v1(2),v2(2),v3(2),v4(2); // mm/s
    

    estimated_mobile_vel =  wRm* Jacobian_mobile.block(0,0,3,12)*generalized_v; // mm/s
   
    
    for(short i = 0;i<3;i++)
    {
        if(std::isnan(estimated_mobile_vel(i)*dt_estimate)) estimated_mobile_vel(i) = 0;
    }
        
    estimated_mobile_position = estimated_mobile_position + estimated_mobile_vel* dt_estimate; // mm   
    /////////////////////////////////////////////////MODIFIED KALMAN FILTER WITH IMU AND ENCODER /////////////////////////////////////////////////////////////////////////////////////////////
    
    mobile_pose_estimation_kalman(dt_estimate,mobile_acc); 
   
   
}
void dghc_controller::mobile_pose_estimation_kalman(double dt_estimate,Eigen::VectorXd mobile_acc)
{
    Zk << estimated_mobile_position,estimated_mobile_vel; // mm , mm/s
    Fk(0,3) = dt_estimate;
    Fk(1,4) = dt_estimate;
    Fk(2,5) = dt_estimate;
    Bk.block(0,0,3,3) = 0.5*dt_estimate*dt_estimate*Eigen::MatrixXd::Identity(3,3);
    Bk.block(3,0,3,3) = dt_estimate*Eigen::MatrixXd::Identity(3,3);
    Uk << 1000*mobile_acc(0),1000*mobile_acc(1),1000*mobile_acc(2); // mm/s^2
    
    if(init_step == 0)
    {
        Qk = 1000*Qk; //1000
        Rk = 0.0064*Rk;  //0
        P_kk = 0.1*P_kk;//100
    }
   
    X_k_1_k = Fk*X_kk + Bk*Uk;
    P_k_1_k = Fk*P_kk*Fk.transpose() + Qk;
    
    Yk = Zk - Hk*X_k_1_k;
    Sk = Hk*P_k_1_k*Hk.transpose() + Rk;
    Kk = P_k_1_k*Hk.transpose()*Sk.inverse();
    
    X_k_1_k_1 = X_k_1_k + Kk*Yk;
    P_k_1_k_1 = P_k_1_k - Kk*Hk*P_k_1_k;
    
    X_kk = X_k_1_k_1;
    P_kk = P_k_1_k_1;

    //////////////////////////////////////////////////////estimated mobile pose ////////////////////////////////////////////
    mobile_position(0) = X_kk(0)*0.001; // m
    mobile_position(1) = X_kk(1)*0.001;
    mobile_position(2) = X_kk(2)*0.001;
    
    mobile_twist(0) = X_kk(3)*0.001; // m/s
    mobile_twist(1) = X_kk(4)*0.001;
    mobile_twist(2) = X_kk(5)*0.001;

    mobile_TF.block(0,0,3,3) = wRm;
    mobile_TF.block(0,3,3,1) = mobile_position;  
     
    
}



Eigen::MatrixXd dghc_controller::KDLFrameToEigenFrame(const KDL::Frame& Frame)
{
    Eigen::MatrixXd TF(4, 4);
    for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                TF(i,j) = Frame(i,j);
            }
        }

    return TF;
}
void dghc_controller::getModel()
{  
    ///////////////////////////////////////////// get joint states//////////////////////////////////////////////////////
    KDL::JntArray Q(joint_size);
    KDL::JntArray Q_dot(joint_size);
            
    for (int i = 0; i < joint_size; i++) {
        Q(i) = q(i);
        Q_dot(i) = q_dot(i);
    }
    
    ///////////////////////////////////////////// get jacobians ////////////////////////////////////////////////////////
    KDL::ChainJntToJacSolver jac_solver(chain);
    KDL::Jacobian J_arm(joint_size); 
    jac_solver.JntToJac(Q, J_arm);
    Jacobian_arm = wRm_e*J_arm.data;
    Jacobian_mobile_inv <<   1,  0,  0,   0,  0, -Ly,  //ref:mobile_base   
                             0,  1,  0,   0,  0,  Lx,
                             1,  0,  0,   0,  0,  Ly,
                             0,  1,  0,   0,  0,  Lx,
                             1,  0,  0,   0,  0,  Ly,
                             0,  1,  0,   0,  0, -Lx,
                             1,  0,  0,   0,  0, -Ly,
                             0,  1,  0,   0,  0, -Lx,
                             0,  0, -1,  Ly, Lx,   0,   //f_r,f_l,r_l,r_r
                             0,  0, -1,- Ly, Lx,   0,  
                             0,  0, -1,- Ly,-Lx,   0,  
                             0,  0, -1,  Ly,-Lx,   0;  

    
    
    Jacobian_whole.block(0,0,6,12) = wRm_e*Jacobian_mobile;
    Jacobian_whole.block(0,12,6,6) = Jacobian_arm;
    Jacobian_base << Jacobian_mobile.block(3,0,2,12),Eigen::MatrixXd::Zero(2,6);
    Jacobian_obs_tmp << wRm*Jacobian_mobile.block(0,0,3,12), Eigen::MatrixXd::Zero(3,6);
    /////////////////////////////////////////// get end-effector pose & twsit /////////////////////////////////////////////////////
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    fk_solver.JntToCart(Q, end_effector_pose);
    end_effector_tmp_TF = KDLFrameToEigenFrame(end_effector_pose); //end_effector_tmp_TF: mani base to mani end
    end_effector_TF = mobile_TF*end_effector_tmp_TF; // mobile_TF: wordl to mani base
    
    end_position = end_effector_TF.block(0,3,3,1);
    wRe = end_effector_TF.block(0,0,3,3);
    wRe_e.block(0,0,3,3) = wRe;
    wRe_e.block(3,3,3,3) = wRe;
    end_quat = Eigen::Quaterniond(wRe);
    
    end_twist = Jacobian_arm*q_dot + mobile_twist; //unit : m/s
    
    KDL::ChainDynParam dyn_param(chain,KDL::Vector(0.0,0.0,-9.8));

    if(init_step == 0) 
    {
        d_end_position = end_position;
        d_end_quat = end_quat;
        d_mobile_quat = mobile_quat;
       
    }
        
    
}


void dghc_controller::getTwist()
{
    if(init_step ==0)
    {
        last_update_time = rclcpp::Clock{}.now();
    }
            
    dt = (rclcpp::Clock{}.now() - last_update_time).seconds();
    last_update_time = rclcpp::Clock{}.now();
    //std::cout<<"d_end_position: "<<std::endl<<d_end_position<<std::endl;
    //loop 도는데 걸리는 시간 측정
    
    Eigen::VectorXd error = Eigen::VectorXd::Zero(6);
   
    Eigen::VectorXd desire_adm_acc = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(18,18);
    //임의의 Mass(M_,M_ori_), Damping(D_), Stiffness(K_)
    double M_ = 5;
    double M_ori_ = 0.3;
    double D_ = 16;
    double K_ = 10;
    
    

    M.diagonal() << M_, M_, M_,M_ori_,M_ori_,M_ori_;
    D.diagonal() << D_,D_,D_,0.2 *D_,0.2 *D_,0.2 *D_;
    K.diagonal() << K_,K_,K_,0.2 *K_,0.2 *K_,0.2 *K_; 
   
    error.head(3) = end_position - d_end_position;
    if(d_end_quat.coeffs().dot(end_quat.coeffs()) < 0.0)
    {
        end_quat.coeffs() << -end_quat.coeffs();
    }
    Eigen::Quaterniond quat_rot_err (end_quat * d_end_quat.inverse());
    if(quat_rot_err.coeffs().norm() > 1e-3)
    {
      quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
    }
    Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
    error.tail(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle(); 
    Eigen::VectorXd coupling_wrench = Eigen::VectorXd::Zero(6);
    coupling_wrench = D * desire_adm_vel + K * error;
    desire_adm_acc = M.inverse() * (-coupling_wrench + wRe_e*ForceTorque*0.1);
    double a_acc_norm = (desire_adm_acc.segment(0, 3)).norm();
    double arm_max_acc_ = 1.0;
    if (a_acc_norm > arm_max_acc_) 
    {
      desire_adm_acc.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
    }
    desire_adm_vel += desire_adm_acc * dt;
  
    //////////////////////////////// mobile orientation virtual impedance && admitance ////////////////////////////
   
    Eigen::MatrixXd Mass = Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd Damping = Eigen::MatrixXd::Identity(2,2);
    
  
   
    Eigen::Matrix3d mRmd = Eigen::Matrix3d::Identity(3,3);

    mRmd = mobile_quat.toRotationMatrix().transpose()*d_mobile_quat.toRotationMatrix();
    Eigen::Quaterniond mobile_e_quat(mRmd);
    Eigen::VectorXd mobile_wrench = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mobile_error = Eigen::VectorXd::Zero(3);
    
    mobile_error<<mobile_e_quat.x(),mobile_e_quat.y(),mobile_e_quat.z();
    mobile_error = wRm*mobile_error;
    mobile_wrench = k*(mobile_error) - b*mobile_twist.block(3,0,3,1);
    

    mobile_wrench = wRm.transpose()*mobile_wrench; //ref : base frame
  
    Mass << 100,0, //50
         0,100;
    Damping << 10,0,
         0,10;   

    Eigen::MatrixXd temp = Mass + dt*Damping;
    d_mobile_twist = temp.inverse()*(Mass*d_mobile_twist + dt*mobile_wrench.block(0,0,2,1)); // ref : base frame
    
    // //////////////////////////////////////// mobile obstacle avoidance /////////////////////////////////////////
    Eigen::VectorXd obs_position_tmp = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd obs_vel_tmp = Eigen::VectorXd::Zero(2);
    
    if(obs_data.size() != 1 && obs_data(0) == 1)
    {
       
        obs_position.clear();
        obs_vel.clear();
        for(int i = 0 ;i<(obs_data.size()-1)/4 ;i++)
        {
            for(int j = 0; j<2; j++)
            {
                obs_position_tmp(j) = obs_data(i*4+j+1);
                obs_vel_tmp(j) = obs_data(i*4+j+2+1);
            }
            obs_position.push_back(obs_position_tmp);
            obs_vel.push_back(obs_vel_tmp);
    
        }
    
      
        Eigen::MatrixXd Mass_obs = Eigen::MatrixXd::Identity(2,2);
        Eigen::MatrixXd Damping_obs = Eigen::MatrixXd::Identity(2,2);
    
        double dref = 0.5;
        double alpha = 0.5;
        double b_obs = 15;//45 
        double k_obs = 20;//50
        double d_dot = 0;
        double distance = 0;
        mobile_wrench_obs<<0,0;
        for(unsigned int i = 0 ; i<(obs_data.size()-1)/4;i++)
        {   
            Eigen::VectorXd distanceV = mobile_position.block(0,0,2,1) - obs_position[i];
            
            
            distance = distanceV.norm(); //obs_position : world frame 
           
            
            Eigen::VectorXd unit_distance = distanceV/distance;
            
            
            Eigen::VectorXd rel_vel = mobile_twist.block(0,0,2,1) - obs_vel[i];
          
            
            d_dot = unit_distance.transpose()*(rel_vel);
            
            
            if(distance <= dref + alpha)
            {
                mobile_wrench_obs = mobile_wrench_obs + unit_distance*(k_obs*(dref + alpha - distance) - b_obs*d_dot);
            }

        }

       
        
            Mass_obs << 50,0, //50
                        0,50;
            Damping_obs << 100,0,
                            0,100;   
            Eigen::MatrixXd temp_obs = Mass_obs + dt*Damping_obs;
            d_mobile_vel_obs = temp_obs.inverse()*(Mass_obs*d_mobile_vel_obs + dt*mobile_wrench_obs); // ref : world frame
        

        double vel_obs = d_mobile_vel_obs.norm();

        Eigen::VectorXd unit_vel_obs = Eigen::VectorXd::Zero(2);

        if(vel_obs == 0)
        {
            unit_vel_obs <<0,0;
        }
        else
        {
            unit_vel_obs = d_mobile_vel_obs/vel_obs;
        }

        if(isnan(unit_vel_obs(0))||isnan(unit_vel_obs(1))) unit_vel_obs << 0,0;

        Jacobian_obs = unit_vel_obs.transpose()*Jacobian_obs_tmp.block(0,0,2,18);
    
    }
    else if((obs_data.size() == 1  && obs_data(0) == 0 )|| d_mobile_vel_obs.norm()<=0.01) // when obstacles are not detected
    {
        Jacobian_obs = Eigen::MatrixXd::Zero(1,18);
        d_mobile_vel_obs<<0,0;
       
    
    }
  
    
    allx_dot_d.clear();
    allx_dot_d.push_back(desire_adm_vel); // ref: world frame
    allx_dot_d.push_back(d_mobile_twist); // ref: base frame
    allx_dot_d.push_back(d_mobile_vel_obs); // ref: world frame
    
}

void dghc_controller::setPriority()
{
    //choose scalar priorities (between one and zero) for each pair of tasks -> there exists 0.5*(numTasks*numTasks+numTasks) pairs!
    Eigen::VectorXd prioritiesVector;
    prioritiesVector = Eigen::VectorXd::Zero(0.5*(numTasks*numTasks+numTasks));
    //exampleA: strict hierachy with "task1" strict more important that "task0" 
    
    if(d_mobile_vel_obs.norm()<=0.01)
   {
        prioritiesVector[0] = 0.0;                                                    //     0  1   : task num    
        prioritiesVector[1] = 1.0;                                                    //   0 0  1
        prioritiesVector[2] = 0.0;                                                    //   1 0  0
        prioritiesVector[3] = 0.0; 
        prioritiesVector[4] = 0.0; 
        prioritiesVector[5] = 1.0;                                                    
   }
   else
   {
        prioritiesVector[0] = 0.0;                                                    //     0  1   : task num    
        prioritiesVector[1] = 1.0;                                                    //   0 0  1
        prioritiesVector[2] = 1.0;                                                    //   1 0  0
        prioritiesVector[3] = 0.0; 
        prioritiesVector[4] = 0.0; 
        prioritiesVector[5] = 0.0;  
   }

    int counter=0;
    for(unsigned int i=0; i<numTasks; i++){
     for(unsigned int j=i; j<numTasks; j++){
      if(prioritiesVector(counter) < 0){
       std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector(counter) << " < 0" << std::endl;
       
      }
      if(prioritiesVector(counter) > 1){
       std::cerr << " a( " << i << "," << j << ") = " << prioritiesVector(counter) << " > 1" << std::endl;
       
      }
      setAlphaIJ(i,j,prioritiesVector(counter));
      counter++;
     }
    }
    assert(counter==prioritiesVector.size());

}
void dghc_controller::setInertia()
{
    Eigen::MatrixXd I10 = Eigen::MatrixXd::Identity(getDOFsize(),getDOFsize());
    
    setInertiaMatrix(I10);
    // std::cout<<"W: "<<std::endl<<Weighted_wholeM<<std::endl;
}
void dghc_controller::getJacobian()
{   
    
    allJacobians.clear();
    allJacobians.push_back(Jacobian_whole); // ref : world frame
    allJacobians.push_back(Jacobian_base); // ref : mobile frame
    allJacobians.push_back(Jacobian_obs); //ref : world frame
    setJacobianMatrices(allJacobians);
    
//    std::cout<<"alljacobians:"<<std::endl<<allJacobians[0]<<std::endl;
}
void dghc_controller::getProjectionM()
{
    allProjections.clear();
    for(unsigned int i=0; i<numTasks; i++){
        allProjections.push_back(Eigen::MatrixXd::Zero(Dof,Dof));
    }
  
    Eigen::VectorXd ranks;
    ranks = Eigen::VectorXd::Zero(numTasks);
   
    getAllGeneralizedProjectors(allProjections, ranks);

}
void dghc_controller::model2realCmd(Eigen::VectorXd V)
{
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2,2);

    Eigen::VectorXd wfrV = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd wflV = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd wrlV = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd wrrV = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd u1 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u2 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u3 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u4 = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u1_f = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u2_f = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u3_f = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd u4_f = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd wheel_ew_vel_f = Eigen::VectorXd::Zero(4);
    
     for(short i = 0;i<4;i++)
    {   
       
        if(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))<=0.01) 
        {
           
            thetalist(i) = 0;
            wheel_ew_vel_cmd(i) = 0;
            wheel_ro_vel_cmd(i) = 0;
        }
        else
        {   
            R<<cos(0.01),-sin(0.01),
            sin(0.01),cos(0.01);

            u1 << cos(wheel_ro(0)),sin(wheel_ro(0));
            u2 << cos(wheel_ro(1)),sin(wheel_ro(1));
            u3 << cos(wheel_ro(2)),sin(wheel_ro(2));
            u4 << cos(wheel_ro(3)),sin(wheel_ro(3));

            u1_f = R*u1;
            u2_f = R*u2;
            u3_f = R*u3;
            u4_f = R*u4;

            wheel_ew_vel_cmd << u1.transpose()*V.block(0,0,2,1),u2.transpose()*V.block(2,0,2,1),u3.transpose()
                                *V.block(4,0,2,1),u4.transpose()*V.block(6,0,2,1); 
            
            wheel_ew_vel_f <<  u1_f.transpose()*V.block(0,0,2,1),u2_f.transpose()*V.block(2,0,2,1),u3_f.transpose()
                                *V.block(4,0,2,1),u4_f.transpose()*V.block(6,0,2,1); 
            if(wheel_ew_vel_cmd(i)>0)
            {
            
              if( abs(wheel_ew_vel_f(i))>abs(wheel_ew_vel_cmd(i)))
              {
                  thetalist(i)= acos(wheel_ew_vel_cmd(i)/(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))));
                  if(std::isnan(thetalist(i)))  thetalist(i) = 0;
              }
              else
              {
                  thetalist(i)= -acos(wheel_ew_vel_cmd(i)/(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))));
                  if(std::isnan(thetalist(i)))  thetalist(i) = 0;
              }
            }
            else 
            {
               if(abs(wheel_ew_vel_f(i)) > abs(wheel_ew_vel_cmd(i)))
              {
                  thetalist(i)= M_PI - acos(wheel_ew_vel_cmd(i)/(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))));
                  if(std::isnan(thetalist(i)))  thetalist(i) = 0;
              }
              else
              {
                  thetalist(i)= -M_PI + acos(wheel_ew_vel_cmd(i)/(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))));
                  if(std::isnan(thetalist(i)))  thetalist(i) = 0;
              }
            }
        }
        
    }
  

    double p_gain = 3; //1.5
    double d_gain = 0.015;//0.1
    
    wheel_ro_vel_cmd = p_gain*thetalist-d_gain*wheel_ro_dot;
    
    wheel_pr_vel_cmd<<V.block(8,0,4,1);    
  
    mani_q_vel_cmd = V.block(12,0,6,1);
}  
bool dghc_controller::init_topics()
{
    if(init_mobile_joint_flag ==1 && count1 == 0)
    {
        std::cout<<"init mobile"<<std::endl;
        count1 =1;
    }
    if(init_mani_joint_flag ==1&& count2 == 0)
    {
        std::cout<<"init mani"<<std::endl;
        count2 =1;
    }
    if(init_imu_flag ==1&& count3 == 0)
    {
        std::cout<<"init IMU"<<std::endl;
        count3 =1;
    }
    if(init_parameters_flag ==1&& count4 == 0)
    {
        std::cout<<"init parameters"<<std::endl;
        count4 =1;
    }
    if(init_ft_flag ==1&& count5 == 0)
    {
        std::cout<<"init F/T sensor"<<std::endl;
        count5 =1;
    }
    if(init_obs_flag ==1&& count6 == 0)
    {
        std::cout<<"init obs state"<<std::endl;
        count6 =1;
    }
    if(init_mobile_joint_flag ==1 && init_mani_joint_flag ==1 && init_imu_flag == 1 /*&& init_parameters_flag ==1*/ && init_ft_flag ==1 && init_obs_flag ==1) 
    {
        return true;
    }
    else
    {
        return false;
    }

}
int dghc_controller::run()
{   
    tasksize = getTasksize();
    numTasks = getNumTasks();
    Dof = getDOFsize();
    estimated_mobile_position<<0,0,570; //unit: mm ,initial relative position w between orld frame and mobile base frame
     
    std::cout<<"dof: "<<Dof<<std::endl;
    std::cout<<"numTasks: "<<numTasks<<std::endl;
    std::cout<<"tasksize: "<<tasksize<<std::endl;

    rclcpp::Rate loop_rate(1000);
    
    while(rclcpp::ok())
    {  
        if(init_topics())
        {
            mobile_pose_estimation();
            
            getModel(); //update current q
           
            getTwist(); //update x_dot_d
            
            getJacobian(); //update jacobian matrix
            
            setPriority();
           
            setInertia();
           
            getProjectionM(); //update_projection matrix
            
            if(!qp_init_flag)
            {   
                
                qp_init(allq,allx_dot_d,allJacobians,allProjections,numTasks,Dof,tasksize);
                qp_init_flag =1;
               
            }
            else
            {
                qp_updateAllConstraint(allProjections,allJacobians,allx_dot_d,allq);
                
            }

            if(!qp_solve_problem(allProjections))
            {
                return 0;
            }
            else
            {   
               
                model2realCmd(getProjectedJointVel());
            }

            init_step =1;
            fisrt_loop = 1; 
        }
        loop_rate.sleep();   
    }
    return 0; 
}
