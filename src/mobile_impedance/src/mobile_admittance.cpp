
#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include "gazebo_msgs/LinkStates.h"
#include <velocity.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <cmath>
class AdmittanceController
{
public:
  AdmittanceController()
  {
    force_sub = n_.subscribe("/published_force", 100, &AdmittanceController::AdmittanceCallback, this);
    state_sub = n_.subscribe("/gazebo/link_states", 100, &AdmittanceController::StateCallback, this);
    
    joint_state_sub = n_.subscribe("/mobile_base/joint_states", 100, &AdmittanceController::JointStateCallback, this);
    fl_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_l_ew_joint_velocity_controller/command", 1000);
    fr_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_r_ew_joint_velocity_controller/command", 1000);
    rl_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_l_ew_joint_velocity_controller/command", 1000);
    rr_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_r_ew_joint_velocity_controller/command", 1000);

    fl_rotation_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_l_rotate_joint_velocity_controller/command", 1000);
    fr_rotation_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_r_rotate_joint_velocity_controller/command", 1000);
    rl_rotation_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_l_rotate_joint_velocity_controller/command", 1000);
    rr_rotation_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_r_rotate_joint_velocity_controller/command", 1000);
    
    fl_prismatic_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_l_prismatic_joint_velocity_controller/command", 1000);
    fr_prismatic_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_r_prismatic_joint_velocity_controller/command", 1000);
    rl_prismatic_pub = n_.advertise<std_msgs::Float64>("//mobile_base/r_l_prismatic_joint_velocity_controller/command", 1000);
    rr_prismatic_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_r_prismatic_joint_velocity_controller/command", 1000);
  }

  void AdmittanceCallback(const geometry_msgs::Wrench::ConstPtr& force_msg)
  {
    
    Force_tmp<< force_msg->force.x, force_msg->force.y, force_msg->force.z, force_msg->torque.x, force_msg->torque.y, force_msg->torque.z;
     
    Force_tmp1 << Rc.transpose()*Force_tmp.block(0,0,3,1),Rc.transpose()*Force_tmp.block(3,0,3,1);
        
    Force_tmp2 <<Force_tmp1[0],Force_tmp1[1],Force_tmp1[5],Force_tmp1[2],Force_tmp1[3],Force_tmp1[4];
   
    double upper = 0;
    double lower = -0.2;
    double alpha = 0.05;

    kjl<<500,500,500,500;
    bjl<<10,10,10,10;
    
    jacobian<<0.25,0,0.25,0,0.25,0,0.25,0,0,0,0,0,
              0,0.25,0,0.25,0,0.25,0,0.25,0,0,0,0,
              1/(4*Ly),0,-1/(4*Ly),0,-1/(4*Ly),0,1/(4*Ly),0,0,0,0,0,
              0,0,0,0,0,0,0,0, -0.25   , -0.25    , -0.25    , -0.25,
              0,0,0,0,0,0,0,0,1/(4*Ly),-1/(4*Ly),-1/(4*Ly),1/(4*Ly),
              0,0,0,0,0,0,0,0,1/(4*Lx),1/(4*Lx),-1/(4*Lx),-1/(4*Lx); 
    
    Eigen::VectorXd torque = Eigen::VectorXd::Zero(12);
    torque = jacobian.transpose()*Force_tmp2;
    for(short i =0;i<4;i++)  //roll pitch z //for prismatic joint limit
    {
      
      if(prismatic[i]>= upper)
      {
        if(torque[8+i]>0)
        {
          a[i] = 0;
        }
        else
        {
          a[i] = 1;
        }
      }
      else if(prismatic[i]< upper && (upper-alpha)< prismatic[i])
      {
        
        if(torque[8+i]>0)
        {
          a[i] = 1- abs(upper - alpha -prismatic[i])/alpha;
        }
        else
        {
          a[i] = 1;
        }
      }
      else if(prismatic[i]>lower && (lower + alpha)>prismatic[i])
      {
        
        if(torque[8+i]>0)
        {
          a[i] = 1;
        }
        else
        {
          a[i] = 1- abs(lower + alpha -prismatic[i])/alpha;
        }
      }
      else if(prismatic[i] <= lower)
      {
        
        if(torque[8+i]>0)
        {
          a[i] = 1;
        }
        else
        {
          a[i] = 0;
        }
      }
      else
      {
        a[i] = 1;
        
      }

      if(upper - alpha < prismatic[i])
      {
        Tjl[i+8] = (kjl[i]*(upper - alpha - prismatic[i]) - bjl[i]*prismatic_dot[i]);
      }
      else if(lower + alpha >prismatic[i])
      {
        Tjl[i+8] = (kjl[i]*(lower + alpha - prismatic[i]) - bjl[i]*prismatic_dot[i]);
 
      }
      else
      {
        Tjl[i+8] = 0;
      }
    }
    
    
    scale_matrix<<1,0,0,0,0,0,0,0,0,0,0,0,
                  0,1,0,0,0,0,0,0,0,0,0,0,
                  0,0,1,0,0,0,0,0,0,0,0,0,
                  0,0,0,1,0,0,0,0,0,0,0,0,
                  0,0,0,0,1,0,0,0,0,0,0,0,
                  0,0,0,0,0,1,0,0,0,0,0,0,
                  0,0,0,0,0,0,1,0,0,0,0,0,
                  0,0,0,0,0,0,0,1,0,0,0,0,
                  0,0,0,0,0,0,0,0,a[0],0,0,0,
                  0,0,0,0,0,0,0,0,0,a[1],0,0,
                  0,0,0,0,0,0,0,0,0,0,a[2],0,
                  0,0,0,0,0,0,0,0,0,0,0,a[3];

    
             
    std::cout<<"scale_matrix: "<<scale_matrix<<std::endl;

    Force_tmp3 = jacobian_pinv.transpose()*(scale_matrix*jacobian.transpose()*Force_tmp2);
    Force_tmp4 = Force_tmp3 + jacobian_pinv.transpose()*Tjl;
    Force_tmp5 << Force_tmp4[0],Force_tmp4[1],Force_tmp4[3],Force_tmp4[4],Force_tmp4[5],Force_tmp4[2];
    Force = Force_tmp5;
    std::cout<<"Force: "<<Force_tmp5<<std::endl;
    std::cout<<"Tjl: "<<Tjl.transpose()<<std::endl;
    
  }

  void StateCallback(const gazebo_msgs::LinkStates::ConstPtr& state_msg)
  {                    
    
    std::string target ="mobile_base::base_link";
    std::cout<<"name size: "<<state_msg->name.size()<<std::endl;
    if(stop == 0)
    {  
      for(short i=0;i<state_msg->name.size();i++)
      {
        std::string tmp_name = state_msg->name[i];
        //std::cout<<"tmp_name: "<<tmp_name<<std::endl;
        if(tmp_name.compare(target)==0)
        {
          I = i;
          stop = 1;
          std::cout<<"I: "<<I<<std::endl;
          break;
        }
      }
    }
    
    Eigen::Quaterniond actual_orientation(
          state_msg->pose[I].orientation.w,
          state_msg->pose[I].orientation.x,
          state_msg->pose[I].orientation.y,
          state_msg->pose[I].orientation.z);
    
    Rc = actual_orientation.toRotationMatrix();
    current_pose << state_msg->pose[I].position.x, state_msg->pose[I].position.y, state_msg->pose[I].position.z, roll_actual,pitch_actual,yaw_actual;
    current_vel << state_msg->twist[I].linear.x, state_msg->twist[I].linear.y, state_msg->twist[I].linear.z, state_msg->twist[I].angular.x, 
                    state_msg->twist[I].angular.y, state_msg->twist[I].angular.z;

    //model position & orientation
    Vx  = state_msg->pose[I].position.x;
    Vy  = state_msg->pose[I].position.y;
    // steer_angle = yaw_actual;
    act_angle = yaw_actual;
  }

  void JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
  {
    // each wheel angle


    wheel_theta_fl = joint_state_msg->position[2];
    wheel_theta_fr = joint_state_msg->position[5];
    wheel_theta_rl = joint_state_msg->position[8];
    wheel_theta_rr = joint_state_msg->position[11];

    wheel_theta<<wheel_theta_fl,wheel_theta_fr,wheel_theta_rr,wheel_theta_rl;
    //std::cout<<"wheel_theta:"<<std::endl<<wheel_theta<<std::endl;
    u1 << cos(wheel_theta_fr),sin(wheel_theta_fr);
    u2 << cos(wheel_theta_fl),sin(wheel_theta_fl);
    u3 << cos(wheel_theta_rl),sin(wheel_theta_rl);
    u4 << cos(wheel_theta_rr),sin(wheel_theta_rr);
    


    wheel_theta_dot_fl_rot = joint_state_msg->velocity[2];
    wheel_theta_dot_fr_rot = joint_state_msg->velocity[5];
    wheel_theta_dot_rl_rot = joint_state_msg->velocity[8];
    wheel_theta_dot_rr_rot = joint_state_msg->velocity[11];

    wheel_theta_dot<<wheel_theta_dot_fr_rot,wheel_theta_dot_fl_rot,wheel_theta_dot_rl_rot,wheel_theta_dot_rr_rot;


   
    prismatic << joint_state_msg->position[4], joint_state_msg->position[1], joint_state_msg->position[7], joint_state_msg->position[10];
    prismatic_dot << joint_state_msg->velocity[4], joint_state_msg->velocity[1], joint_state_msg->velocity[7], joint_state_msg->velocity[10];

  }

  int run(){
    ros::Rate loop_rate(1000);
    while(ros::ok)
    {
      M <<50,0,0,0,0,0,
          0,50,0,0,0,0,
          0,0,50,0,0,0,
          0,0,0,100,0,0,
          0,0,0,0,100,0,
          0,0,0,0,0,50;
      
      B <<40,0,0,0,0,0,
          0,40,0,0,0,0,
          0,0,40,0,0,0,
          0,0,0,10,0,0,
          0,0,0,0,10,0,
          0,0,0,0,0,10;
            
      Eigen::MatrixXd temp = M + dt*B;
      vel = temp.inverse()*(M*past_vel + dt*Force);
      // for(int i = 0;i<vel.size();i++)
      // {
      //     if(vel(i)>1) vel(i) = 0.1;
      // }
      
      //desired_pose << 0,0,0,0,0,0;
       past_vel = vel;
    
      chassis_vel << vel(0),vel(1),vel(5),vel(2),vel(3),vel(4); //x,y,yaw,z,roll,pitch
      
      std::cout<<"chassis_vel: "<< std::endl<<chassis_vel.transpose()<<std::endl;

      jacobian_pinv<<  1,  0,    Ly,  0,   0,  0,                                      
                       0,  1,    Lx,  0,   0,  0,
                       1,  0,   -Ly,  0,   0,  0,
                       0,  1,    Lx,  0,   0,  0,
                       1,  0,   -Ly,  0,   0,  0,
                       0,  1,   -Lx,  0,   0,  0,
                       1,  0,    Ly,  0,   0,  0,
                       0,  1,   -Lx,  0,   0,  0,
                       0,  0,     0,  -1, Ly, Lx,
                       0,  0,     0,  -1,-Ly, Lx,
                       0,  0,     0,  -1,-Ly,-Lx,
                       0,  0,     0,  -1, Ly,-Lx;
      
      
      Eigen::VectorXd V = jacobian_pinv * chassis_vel; 

     
      //std::cout<<"V: "<< std::endl<<V<<std::endl;
     // Eigen::VectorXd V = Eigen::VectorXd::Ones(8);
      
     
      std::cout<<"chassis_Vel: "<<std::endl<<chassis_vel<<std::endl;
      std::cout<<"jacobian*V: "<<std::endl<<jacobian*V<<std::endl; 
      if(V.isZero(tolerance))
      {
        thetalist << 0, 0, 0, 0;
      }
      else
      { 
        
          
      
        
        
      
      

      R<<cos(0.01),-sin(0.01),
         sin(0.01),cos(0.01);
      u_f.block(0,0,2,1) = R*u1;
      u_f.block(2,0,2,1) = R*u2;
      u_f.block(4,0,2,1) = R*u3;
      u_f.block(6,0,2,1) = R*u4;

      wheel_vel_f<< u_f.block(0,0,2,1).transpose()*V.block(0,0,2,1),u_f.block(2,0,2,1).transpose()*V.block(2,0,2,1),u_f.block(4,0,2,1).transpose()*V.block(4,0,2,1),u_f.block(6,0,2,1).transpose()*V.block(6,0,2,1);
      std::cout<<"thetalist:"<<std::endl<<thetalist<<std::endl; 
      
      wheel_vel << u1.transpose()*V.block(0,0,2,1),u2.transpose()*V.block(2,0,2,1),u3.transpose()*V.block(4,0,2,1),u4.transpose()*V.block(6,0,2,1); 
      for(short i=0;i<4;i++)
      {
        if(wheel_vel[i]>0)
        {
          
          if( abs(wheel_vel_f[i])>abs(wheel_vel[i]))
          {
              thetalist[i]= acos(wheel_vel[i]/(sqrt(V[2*i]*V[2*i]+V[2*i+1]*V[2*i+1])));
          }
          else
          {
              thetalist[i]= -acos(wheel_vel[i]/(sqrt(V[2*i]*V[2*i]+V[2*i+1]*V[2*i+1])));
          }
        }
        else
        {
           if(abs(wheel_vel_f[i]) > abs(wheel_vel[i]))
          {
              thetalist[i]= PI - acos(wheel_vel[i]/(sqrt(V[2*i]*V[2*i]+V[2*i+1]*V[2*i+1])));
          }
          else
          {
              thetalist[i]= -PI + acos(wheel_vel[i]/(sqrt(V[2*i]*V[2*i]+V[2*i+1]*V[2*i+1])));
          }
        }
      }
      
      rotation_vel = p_gain*(thetalist)-d_gain*wheel_theta_dot;
      if(isnan(rotation_vel(0))) rotation_vel <<0,0,0,0;
      lift_vel <<V(8),V(9),V(10),V(11);
      
      
      velocity1.data = -wheel_vel(0);
      velocity2.data =  wheel_vel(1);
      velocity3.data = -wheel_vel(2);
      velocity4.data = -wheel_vel(3);

      theta1.data = rotation_vel(0);
      theta2.data = rotation_vel(1);
      theta3.data = rotation_vel(2);
      theta4.data = rotation_vel(3);

      lift_vel1.data  =  lift_vel(0);
      lift_vel2.data  =  lift_vel(1);
      lift_vel3.data  =  lift_vel(2);
      lift_vel4.data  =  lift_vel(3);

      fr_velocity_pub.publish(velocity1);
      fl_velocity_pub.publish(velocity2);
      rl_velocity_pub.publish(velocity3);
      rr_velocity_pub.publish(velocity4);

      fr_rotation_pub.publish(theta1);
      fl_rotation_pub.publish(theta2);
      rl_rotation_pub.publish(theta3);
      rr_rotation_pub.publish(theta4);
      
      fr_prismatic_pub.publish(lift_vel1);
      fl_prismatic_pub.publish(lift_vel2);
      rl_prismatic_pub.publish(lift_vel3);
      rr_prismatic_pub.publish(lift_vel4);
      }
      // ROS_WARN_STREAM( velocity1 << ", " << velocity2 << ", " << velocity3 << ", " << velocity4);
      // ROS_WARN_STREAM( theta1 << ", " << theta2 << ", " << theta3 << ", " << theta4);

      // std::cout << "vel" << std::endl << chassis_vel.transpose() << std::endl;
      // std::cout << "thetalist" << std::endl << thetalist.transpose() << std::endl;
       std::cout << "rotation_vel" << std::endl << rotation_vel.transpose() << std::endl;
      loop_rate.sleep();
      
      
    }
    return 0;
  }



private: 
  ros::NodeHandle n_; 
  ros::Subscriber force_sub;
  ros::Subscriber state_sub;
  ros::Subscriber joint_state_sub;
  
  ros::Publisher fl_velocity_pub;
  ros::Publisher fr_velocity_pub;
  ros::Publisher rl_velocity_pub;
  ros::Publisher rr_velocity_pub;

  ros::Publisher fl_rotation_pub;
  ros::Publisher fr_rotation_pub;
  ros::Publisher rl_rotation_pub;
  ros::Publisher rr_rotation_pub;

  ros::Publisher fl_prismatic_pub;
  ros::Publisher fr_prismatic_pub;
  ros::Publisher rl_prismatic_pub;
  ros::Publisher rr_prismatic_pub;

  Eigen::VectorXd Force = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Force_tmp = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Force_tmp1 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Force_tmp2 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Force_tmp3 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Force_tmp4 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Force_tmp5 = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd current_pose = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd current_vel = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd chassis_vel = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd past_vel = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd thetalist = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd thetalist_past = Eigen::VectorXd::Zero(4);
  
  Eigen::VectorXd rotation_vel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd wheel_vel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd lift_vel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd u1 = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd u2 = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd u3 = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd u4 = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd wheel_theta = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd wheel_theta_dot = Eigen::VectorXd::Zero(4);
  double p_gain = 1.5;
  double d_gain = 0.08;
  Eigen::VectorXd V_tmp = Eigen::VectorXd::Zero(8);
  Eigen::Matrix3d Rc = Eigen::Matrix3d::Zero(3,3);
  Eigen::MatrixXd jacobian_pinv = Eigen::MatrixXd::Zero(12,6);
  Eigen::MatrixXd jacobian= Eigen::MatrixXd::Zero(6,12);;
  double roll_actual, pitch_actual, yaw_actual;
  double dt = 0.001;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6,6);
  bool stop = 0;
  short I=0;
  std_msgs::Float64 velocity1;
  std_msgs::Float64 velocity2;
  std_msgs::Float64 velocity3;
  std_msgs::Float64 velocity4;

  std_msgs::Float64 theta1;
  std_msgs::Float64 theta2;
  std_msgs::Float64 theta3;
  std_msgs::Float64 theta4;

  std_msgs::Float64 lift_vel1;
  std_msgs::Float64 lift_vel2;
  std_msgs::Float64 lift_vel3;
  std_msgs::Float64 lift_vel4;
  Eigen::VectorXd qdd = Eigen::VectorXd::Zero(6);
  const double tolerance = 1e-6;

  Eigen::VectorXd Tjl = Eigen::VectorXd::Zero(12);
  
  Eigen::VectorXd kjl = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd bjl = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd kjl_r = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd bjl_r = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd prismatic = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd prismatic_dot = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd scale_matrix = Eigen::MatrixXd::Zero(12,12);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2,2);
  Eigen::VectorXd u_f =Eigen::VectorXd::Zero(8);
  Eigen::VectorXd wheel_vel_f =Eigen::VectorXd::Zero(4);

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_node");
  ros::AsyncSpinner spinner(0);
  
    spinner.start();
    AdmittanceController admittance; 
    admittance.run();
    spinner.stop();
  
  return 0;
}