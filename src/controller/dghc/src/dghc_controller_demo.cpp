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
   
    timer_ = this->create_wall_timer(1ms, std::bind(&dghc_controller::timer_callback, this));
    joint_states_sub_  = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 100, std::bind(&dghc_controller::joint_states_callback, this, std::placeholders::_1));
    mobile_pos_sub_ = this->create_subscription<linkpose_msgs::msg::LinkPose>("/LinkPose_mobile_ur5e_base_link", 100, std::bind(&dghc_controller::mobile_pose_callback, this, std::placeholders::_1));
    mobile_twist_sub_ = this->create_subscription<linkpose_msgs::msg::LinkTwist>("/LinkTwist_mobile_ur5e_base_link", 100, std::bind(&dghc_controller::mobile_twist_callback, this, std::placeholders::_1));
    joint_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ur5e_controller/commands", 100); //mani velocity controller
    lift_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lift_controller/commands", 100); //lift velocity controller
    wheel_vel_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mobile_controller/commands", 100); //lift velocity controller

}

void dghc_controller::timer_callback()
{
    
    
    if(fisrt_loop)
    {
        std_msgs::msg::Float64MultiArray mani_vel_msg;
        std_msgs::msg::Float64MultiArray lift_vel_msg;
        std_msgs::msg::Float64MultiArray wheel_vel_msg;
        
        mani_vel_msg.data = {mani_q_vel_cmd(0),mani_q_vel_cmd(1),mani_q_vel_cmd(2),mani_q_vel_cmd(3),mani_q_vel_cmd(4),mani_q_vel_cmd(5)};
        lift_vel_msg.data = {wheel_pr_vel_cmd(0),wheel_pr_vel_cmd(1),wheel_pr_vel_cmd(2),wheel_pr_vel_cmd(3)}; //f_r,f_l,r_l,r_r
        wheel_vel_msg.data = {-wheel_ew_vel_cmd(0),wheel_ew_vel_cmd(1),-wheel_ew_vel_cmd(2),-wheel_ew_vel_cmd(3),wheel_ro_vel_cmd(0),wheel_ro_vel_cmd(1),wheel_ro_vel_cmd(2),wheel_ro_vel_cmd(3)};
        joint_vel_pub->publish(mani_vel_msg);
        lift_vel_pub->publish(lift_vel_msg);
        wheel_vel_pub->publish(wheel_vel_msg);
    }
}

void dghc_controller::joint_states_callback(const sensor_msgs::msg::JointState& JointState_Data)
{   
       
        //q
        q(0) = JointState_Data.position[0];
        q(1) = JointState_Data.position[1];
        q(2) = JointState_Data.position[2];
        q(3) = JointState_Data.position[3];
        q(4) = JointState_Data.position[13];
        q(5) = JointState_Data.position[16];

        //q_dot
        q_dot(0) = JointState_Data.velocity[0];
        q_dot(1) = JointState_Data.velocity[1];
        q_dot(2) = JointState_Data.velocity[2];
        q_dot(3) = JointState_Data.velocity[3];
        q_dot(4) = JointState_Data.velocity[13];
        q_dot(5) = JointState_Data.velocity[16];

        wheel_ro(0) = JointState_Data.position[10]; //   rotate(f_r,f_l,r_l,r_r),ew(f_r,f_l,r_l,r_r),prismatic(f_r,f_l,r_l,r_r), 
        wheel_ro(1) = JointState_Data.position[12];     
        wheel_ro(2) = JointState_Data.position[9];
        wheel_ro(3) = JointState_Data.position[8];
        
        wheel_ew(0) = JointState_Data.position[11];
        wheel_ew(1) = JointState_Data.position[17];
        wheel_ew(2) = JointState_Data.position[14];
        wheel_ew(3) = JointState_Data.position[5];
        
        wheel_pr(0) = JointState_Data.position[7];
        wheel_pr(1) = JointState_Data.position[15];
        wheel_pr(2) = JointState_Data.position[6];
        wheel_pr(3) = JointState_Data.position[4];
        
        allq<<wheel_ro,wheel_ew,wheel_pr,q;


        wheel_ro_dot(0) = JointState_Data.velocity[10]; //   rotate(f_r,f_l,r_l,r_r),ew(f_r,f_l,r_l,r_r),prismatic(f_r,f_l,r_l,r_r), 
        wheel_ro_dot(1) = JointState_Data.velocity[12];     
        wheel_ro_dot(2) = JointState_Data.velocity[9];
        wheel_ro_dot(3) = JointState_Data.velocity[8];
        
        wheel_ew_dot(0) = JointState_Data.velocity[11];
        wheel_ew_dot(1) = JointState_Data.velocity[17];
        wheel_ew_dot(2) = JointState_Data.velocity[14];
        wheel_ew_dot(3) = JointState_Data.velocity[5];
        
        wheel_pr_dot(0) = JointState_Data.velocity[7];
        wheel_pr_dot(1) = JointState_Data.velocity[15];
        wheel_pr_dot(2) = JointState_Data.velocity[6];
        wheel_pr_dot(3) = JointState_Data.velocity[4];
        init_joint_flag =1;

        allq_dot<<wheel_ro_dot,wheel_ew_dot,wheel_pr_dot,q_dot;
      
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
    
    KDL::ChainDynParam dyn_param(chain,KDL::Vector(0.0,0.0,-9.8));

    if(init_step == 0) 
    {
        d_end_position = end_position;
        d_end_quat = end_quat;
        init_step =1;
    }
}


void dghc_controller::getTwist()
{
    //loop 도는데 걸리는 시간 측정
    dt = (rclcpp::Clock{}.now() - last_update_time).seconds();
    last_update_time = rclcpp::Clock{}.now();
    Eigen::VectorXd error = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd ForceTorque = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd desire_adm_acc = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd D = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd K = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(18,18);
    //임의의 Mass(M_,M_ori_), Damping(D_), Stiffness(K_)
    double M_ = 5;
    double M_ori_ = 0.3;
    double D_ = 16;
    double K_ = 40;
    double W_ = 1;
    
    

    M.diagonal() << M_, M_, M_,M_ori_,M_ori_,M_ori_;
    D.diagonal() << D_,D_,D_,0.2 *D_,0.2 *D_,0.2 *D_;
    K.diagonal() << K_,K_,K_,0.2 *K_,0.2 *K_,0.2 *K_; 
    W.diagonal() << W_,W_,W_,W_,W_,W_,W_,W_,W_,W_,W_,W_,W_,W_,W_,W_,W_,W_; //x1,y1,x2,y2, ... ,z1,z2,z3,z4,q1, ... q6;

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
    desire_adm_acc = M.inverse() * (-coupling_wrench + ForceTorque);
    double a_acc_norm = (desire_adm_acc.segment(0, 3)).norm();
    double arm_max_acc_ = 1.0;
    if (a_acc_norm > arm_max_acc_) 
    {
      desire_adm_acc.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
    }
    desire_adm_vel += desire_adm_acc * dt;
  
    allx_dot_d.clear();
    allx_dot_d.push_back(desire_adm_vel);
    
}

void dghc_controller::setPriority()
{
    //choose scalar priorities (between one and zero) for each pair of tasks -> there exists 0.5*(numTasks*numTasks+numTasks) pairs!
    Eigen::VectorXd prioritiesVector;
    prioritiesVector = Eigen::VectorXd::Zero(0.5*(numTasks*numTasks+numTasks));
    //exampleA: strict hierachy with "task0" strict more important that "task1" and "task1" strict more important that "task2"
    prioritiesVector[0] = 0.0;
   
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
    allJacobians.push_back(Jacobian_whole); 
    
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
 
    Eigen::VectorXd thetalist = Eigen::VectorXd::Zero(4);
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
    const double tolerance = 1e-6;
    if(V.isZero(tolerance))
      {
        thetalist << 0, 0, 0, 0;
      }
      else
      { 
        
        for(short i=0;i<4;i++)
        {
            if(wheel_ew_vel_cmd(i)>0)
            {
            
              if( abs(wheel_ew_vel_f(i))>abs(wheel_ew_vel_cmd(i)))
              {
                  thetalist(i)= acos(wheel_ew_vel_cmd(i)/(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))));
              }
              else
              {
                  thetalist(i)= -acos(wheel_ew_vel_cmd(i)/(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))));
              }
            }
            else 
            {
               if(abs(wheel_ew_vel_f(i)) > abs(wheel_ew_vel_cmd(i)))
              {
                  thetalist(i)= M_PI - acos(wheel_ew_vel_cmd(i)/(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))));
              }
              else
              {
                  thetalist(i)= -M_PI + acos(wheel_ew_vel_cmd(i)/(sqrt(V(2*i)*V(2*i)+V(2*i+1)*V(2*i+1))));
              }
            }
        }
      }

    
    double p_gain = 1.5;
    double d_gain = 0.1;
    std::cout<<"thetalist: "<<thetalist.transpose()<<std::endl;
    wheel_ro_vel_cmd = p_gain*thetalist-d_gain*wheel_ro_dot;
    
    if(isnan(wheel_ro_vel_cmd(0)))
     wheel_ro_vel_cmd <<0,0,0,0;

    wheel_pr_vel_cmd<<V.block(8,0,4,1);    
    //wheel_pr_vel_cmd <<0,0,0,0; 
    
    mani_q_vel_cmd = V.block(12,0,6,1);
}  
int dghc_controller::run()
{   
    tasksize = getTasksize();
    numTasks = getNumTasks();
    Dof = getDOFsize();
    
    std::cout<<"dof: "<<Dof<<std::endl;
    std::cout<<"numTasks: "<<numTasks<<std::endl;
    std::cout<<"tasksize: "<<tasksize<<std::endl;

    rclcpp::Rate loop_rate(1000);
    bool qp_init_flag = 0;
    
    while(rclcpp::ok())
    {
        
        if(init_joint_flag ==1)
        {
            
            getModel(); //update current q
            
            getJacobian(); //update jacobian matrix
           
            getTwist(); //update x_dot_d
            
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

            fisrt_loop = 1; 
        }
        loop_rate.sleep();   
    }
    return 0; 
}
