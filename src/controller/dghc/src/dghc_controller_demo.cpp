#include "dghc_controller_demo.hpp"
#include "GHCProjections.hpp"
#include "RequiredHeaders.hpp"
using namespace std::literals::chrono_literals;
dghc_controller::dghc_controller() :Node("dghc_controller")
{   
    //chain부분
    std::string base_link = "base_link"; 
    std::string end_link = "tool0";
    urdf_filename = ament_index_cpp::get_package_share_directory("mobis_description") + "/urdf/mobile_ur5e.urdf"; // 윗 부분이 경로 인식이 잘 안되서 상대경로로 지정
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
   
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(1ms, std::bind(&dghc_controller::timer_callback, this));
    joint_states_sub_  = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 100, std::bind(&dghc_controller::joint_states_callback, this, std::placeholders::_1));
    mobile_pos_sub_ = this->create_subscription<linkpose_msgs::msg::LinkPose>("/LinkPose_mobile_ur5e_base_link", 100, std::bind(&dghc_controller::mobile_pose_callback, this, std::placeholders::_1));
    mobile_twist_sub_ = this->create_subscription<linkpose_msgs::msg::LinkTwist>("/LinkTwist_mobile_ur5e_base_link", 100, std::bind(&dghc_controller::mobile_twist_callback, this, std::placeholders::_1));

}

void dghc_controller::timer_callback()
{
    
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! ";
    // //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);
    
}

void dghc_controller::joint_states_callback(const sensor_msgs::msg::JointState::ConstPtr& JointState_Data)
{   
        //q
        q(0) = JointState_Data->position[0];
        q(1) = JointState_Data->position[1];
        q(2) = JointState_Data->position[2];
        q(3) = JointState_Data->position[3];
        q(4) = JointState_Data->position[13];
        q(5) = JointState_Data->position[16];

        //q_dot
        q_dot(0) = JointState_Data->velocity[0];
        q_dot(1) = JointState_Data->velocity[1];
        q_dot(2) = JointState_Data->velocity[2];
        q_dot(3) = JointState_Data->velocity[3];
        q_dot(4) = JointState_Data->velocity[13];
        q_dot(5) = JointState_Data->velocity[16];

        mobile_q(0) = JointState_Data->position[10]; //   rotate(f_r,f_l,r_l,r_r),ew(f_r,f_l,r_l,r_r),prismatic(f_r,f_l,r_l,r_r), 
        mobile_q(1) = JointState_Data->position[12];
        mobile_q(2) = JointState_Data->position[9];
        mobile_q(3) = JointState_Data->position[8];
        mobile_q(4) = JointState_Data->position[11];
        mobile_q(5) = JointState_Data->position[17];
        mobile_q(6) = JointState_Data->position[14];
        mobile_q(7) = JointState_Data->position[5];
        mobile_q(8) = JointState_Data->position[7];
        mobile_q(9) = JointState_Data->position[15];
        mobile_q(10) = JointState_Data->position[6];
        mobile_q(11) = JointState_Data->position[4];


        mobile_q_dot(0) = JointState_Data->velocity[10]; // rotate(f_r,f_l,r_l,r_r),ew(f_r,f_l,r_l,r_r),prismatic(f_r,f_l,r_l,r_r)
        mobile_q_dot(1) = JointState_Data->velocity[12];
        mobile_q_dot(2) = JointState_Data->velocity[9];
        mobile_q_dot(3) = JointState_Data->velocity[8];
        mobile_q_dot(4) = JointState_Data->velocity[11];
        mobile_q_dot(5) = JointState_Data->velocity[17];
        mobile_q_dot(6) = JointState_Data->velocity[14];
        mobile_q_dot(7) = JointState_Data->velocity[5];
        mobile_q_dot(8) = JointState_Data->velocity[7];
        mobile_q_dot(9) = JointState_Data->velocity[15];
        mobile_q_dot(10) = JointState_Data->velocity[6];
        mobile_q_dot(11) = JointState_Data->velocity[4];
        init_joint_flag =1;
        // mobileJointP
        // mobileJointV
}

void dghc_controller::mobile_pose_callback(const linkpose_msgs::msg::LinkPose Pose_Data)
{
        mobile_position(0) = Pose_Data.x;
        mobile_position(1) = Pose_Data.y;
        mobile_position(2) = Pose_Data.z;
        mobile_quat.x() = Pose_Data.qx;
        mobile_quat.y() = Pose_Data.qy;
        mobile_quat.z() = Pose_Data.qz;
        mobile_quat.w() = Pose_Data.qw;

        wRm = mobile_quat.normalized().toRotationMatrix();
        
        wRm_e.block(0,0,3,3) = wRm;
        wRm_e.block(3,3,3,3) = wRm;

        mobile_TF.block(0,0,3,3) = wRm;
        mobile_TF.block(0,3,3,1) = mobile_position;   
        //std::cout<<"mobile pose: "<<std::endl<<mobile_position.transpose()<<std::endl;  
}

void dghc_controller::mobile_twist_callback(const linkpose_msgs::msg::LinkTwist Twist_Data)
{
        mobile_twist(0) = Twist_Data.x;
        mobile_twist(1) = Twist_Data.y;
        mobile_twist(2) = Twist_Data.z;
        mobile_twist(3) = Twist_Data.roll;
        mobile_twist(4) = Twist_Data.pitch;
        mobile_twist(5) = Twist_Data.yaw;        
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
    //std::cout<<"q: "<<std::endl<<q.transpose()<<std::endl;

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

    Jacobian_mobile <<  0.25,0,0.25,0,0.25,0,0.25,0,0,0,0,0, //ref :mobile_base     x
                        0,0.25,0,0.25,0,0.25,0,0.25,0,0,0,0,
                        0,0,0,0,0,0,0,0, -0.25   , -0.25    , -0.25    , -0.25,
                        0,0,0,0,0,0,0,0,1/(4*Ly),-1/(4*Ly),-1/(4*Ly),1/(4*Ly),                    //f_r,f_l,r_l,r_r
                        0,0,0,0,0,0,0,0,1/(4*Lx),1/(4*Lx),-1/(4*Lx),-1/(4*Lx), 
                        1/(4*Ly),0,-1/(4*Ly),0,-1/(4*Ly),0,1/(4*Ly),0,0,0,0,0;
    
    Jacobian_whole.block(0,0,6,12) = wRm_e*Jacobian_mobile;
    Jacobian_whole.block(0,12,6,6) = Jacobian_arm;
    //std::cout<<"jacobian whole: "<<std::endl<<Jacobian_whole<<std::endl;

    /////////////////////////////////////////// get end-effector pose & twsit /////////////////////////////////////////////////////
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    fk_solver.JntToCart(Q, end_effector_pose);
    end_effector_tmp_TF = KDLFrameToEigenFrame(end_effector_pose);
    end_effector_TF = mobile_TF*end_effector_tmp_TF;
    
    end_position = end_effector_TF.block(0,3,3,1);
    wRe = end_effector_TF.block(0,0,3,3);
    wRe_e.block(0,0,3,3) = wRe;
    wRe_e.block(3,3,3,3) = wRe;
    end_quat = Eigen::Quaterniond(wRe);
    
    end_twist = Jacobian_arm*q_dot + mobile_twist;
    //std::cout<<"end_position: "<<std::endl<<end_position.transpose()<<std::endl;
    std::cout<<"end_twist: "<<std::endl<<end_twist<<std::endl;
    KDL::ChainDynParam dyn_param(chain,KDL::Vector(0.0,0.0,-9.8));
}

int dghc_controller::run()
{   
    
    rclcpp::Rate loop_rate(100);
    std::cout<<rclcpp::ok()<<std::endl;

    while(rclcpp::ok())
    {
        
        if(init_joint_flag ==1)
        {
            getModel();

            // getTwist();

            // getJacobian();

            // setPriority();

            // setInertia();

            // getProjectionM();

            // getProjectedJointVel();
        }
        loop_rate.sleep();   
    }
    return 0; 
}
