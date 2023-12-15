#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/LinkStates.h"
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>

class ImpedanceController
{
public:
  ImpedanceController()
  {
    force_pub = n_.advertise<geometry_msgs::Wrench>("/published_force", 100);
    position_sub = n_.subscribe("/gazebo/link_states", 1, &ImpedanceController::chatterCallback, this);
    dpose_sub = n_.subscribe("/d_psoe", 1, &ImpedanceController::dpose_Callback, this);
    
  }
  
  void dpose_Callback(const geometry_msgs::Pose d_pose)
  {
     
      d_quat.x() = d_pose.orientation.x;
      d_quat.y() = d_pose.orientation.y;
      d_quat.z() = d_pose.orientation.z;
      d_quat.w() = d_pose.orientation.w;
      position_desired<<d_pose.position.x,d_pose.position.y,d_pose.position.z;
  }
  void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
  {
   
   
    std::string target ="mobile_base::base_link";
   //std::cout<<"name size: "<<msg->name.size()<<std::endl;
    if(stop == 0)
    {  
      for(short i=0;i<msg->name.size();i++)
      {
        std::string tmp_name = msg->name[i];
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
     
    
    c_quat.x() = msg->pose[I].orientation.x;
    c_quat.y() = msg->pose[I].orientation.y;
    c_quat.z() = msg->pose[I].orientation.z;
    c_quat.w() = msg->pose[I].orientation.w;
  
    Rd = d_quat.toRotationMatrix();
    Rc = c_quat.toRotationMatrix();
    cRd = Rc.transpose()*Rd;
    Eigen::Quaterniond e_quat(cRd);
    orientation_desired << e_quat.x(),e_quat.y(),e_quat.z();
    
    pos_desired<<position_desired,Rc*orientation_desired;
    
    pos_current <<  msg->pose[I].position.x, msg->pose[I].position.y, msg->pose[I].position.z,0,0,0;

    vel_current <<  msg->twist[I].linear.x, msg->twist[I].linear.y, msg->twist[I].linear.z, msg->twist[I].angular.x, msg->twist[I].angular.y, msg->twist[I].angular.z;
    

  }
void run()
{
   b<<90,0,0,0,0,0,
       0,90,0,0,0,0,
       0,0,60,0,0,0,
       0,0,0,90,0,0,
       0,0,0,0,90,0,
       0,0,0,0,0,60;

    k<<100,0,0,0,0,0,
       0,100,0,0,0,0,
       0,0,100,0,0,0,
       0,0,0,150,0,0,
       0,0,0,0,150,0,
       0,0,0,0,0,150; 
    
     
    
   
   

    vel_desired << 0, 0, 0, 0, 0, 0;

   
    Rc_e<<Rc.transpose(),Z33,
          Z33,I33;
    
    Force = k*(pos_desired-pos_current) - b*vel_current ;
      
    
    std::cout<<"FORCE: "<<std::endl<<Force<<std::endl;
    std::cout<<"pos_d: "<<std::endl<<pos_desired<<std::endl;
    std::cout<<"pos_current: "<<std::endl<<pos_current<<std::endl;
    geometry_msgs::Wrench force_msg;

    force_msg.force.x = Force[0];
    force_msg.force.y = Force[1];
    force_msg.force.z = Force[2];
    force_msg.torque.x = Force[3];
    force_msg.torque.y = Force[4];
    force_msg.torque.z = Force[5];
  force_pub.publish(force_msg);
}
private:
  ros::NodeHandle n_; 
  ros::Publisher force_pub;
  ros::Subscriber position_sub;
  ros::Subscriber dpose_sub;
  geometry_msgs::Pose world;
  Eigen::VectorXd vel_current = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd pos_current = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd vel_desired = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd pos_desired = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd position_desired = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd Force = Eigen::VectorXd::Zero(6);
  Eigen::Quaterniond d_quat;
  Eigen::Quaterniond c_quat;
  
  Eigen::Matrix3d Rd;
  Eigen::Matrix3d Rc;
  Eigen::Matrix3d cRd;
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd k = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd Rc_e = Eigen::MatrixXd::Zero(6,6);
  Eigen::MatrixXd Z33 = Eigen::MatrixXd::Zero(3,3);
  Eigen::MatrixXd I33 = Eigen::MatrixXd::Identity(3,3);
  Eigen::VectorXd orientation_desired = Eigen::VectorXd::Zero(3);
  bool stop = 0;
  short I=0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Impedance_node");
  ImpedanceController impedance; 
  
   ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        impedance.run();
        ros::spinOnce();
        loop_rate.sleep();
       
    }
  return 0;
}



