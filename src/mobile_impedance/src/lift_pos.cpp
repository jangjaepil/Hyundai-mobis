
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "gazebo_msgs/SetLinkState.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>

#include <velocity.h>

#include "math.h"
#include <unistd.h>

std_msgs::Float64 position1;
std_msgs::Float64 position2;
std_msgs::Float64 position3;
std_msgs::Float64 position4;
std_msgs::Float64 state1;
std_msgs::Float64 state2;
std_msgs::Float64 state3;
std_msgs::Float64 state4;

float desired_b; 
double desired_leng[4]={0,0,0,0};
Eigen::VectorXd position = Eigen::VectorXd::Zero(3);
Eigen::Quaterniond quaternion;
Eigen::Matrix3d R_m;
Eigen::Vector3d euler;
void chatterCallback2(const geometry_msgs::QuaternionConstPtr &msg2)
{
  double L14 = 0.6;
  double L12 = 0.6;
  double Lx = 0.3;
  double Ly = 0.3;
  double Lx1 = 0.3;
  double Ly1 = 0.3;
  double alpha = 0;
  double beta = 0;
  double d1 = 0;
  double d2 = 0;
  double d3 = 0;
  double d4 = 0;
  double min =0.5415;
  

  d1 = msg2->x;
  d2 = msg2->y;
  d4 = msg2->z;

  alpha = atan2(d2 - d1, L12);
  beta = atan2(d4 - d1, L14);
 
  
  d3 = (L14*cos(alpha)*sin(beta) + L12*sin(alpha) + d1)/(cos(alpha)*cos(beta));


  desired_leng[0]  = - msg2->x; //d1
  desired_leng[1]  = - msg2->y; //d2
  desired_leng[2]  = - d3; //d3
  desired_leng[3]  = - msg2->z; //d4
  
  std::cout<<"alpha: "<<alpha<<std::endl;
  std::cout<<"beta: "<<beta<<std::endl;
  std::cout<<"d1: "<<d1+min<<std::endl;
  std::cout<<"d2: "<<d2+min<<std::endl;
  std::cout<<"d3: "<<d3+min<<std::endl;
  std::cout<<"d4: "<<d4+min<<std::endl;
  std::cout<<"position: "<<position.transpose()<<std::endl
           <<"euler(xyz): "<<euler.transpose()<<std::endl;

}
void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
     double desired_length1 = desired_leng[0]; //d1
     double desired_length2 = desired_leng[1]; //d2
     double desired_length3 = desired_leng[2]; //d3
     double desired_length4 = desired_leng[3]; //d4
    
// main
     position1.data = desired_length1; //d1
     position2.data = desired_length2; //d2
     position3.data = desired_length3; //d3
     position4.data = desired_length4; //d4
}

void model_states_callback(const gazebo_msgs::ModelStates::ConstPtr &states)
{
     std_msgs::String model_names[states->pose.size()];
     for(int i = 0 ; i<states->pose.size();i++)
     {
       if("mobile_base"==states ->name[i])
       {
         position(0) = states->pose[i].position.x;
         position(1) =states->pose[i].position.y;
         position(2) =states->pose[i].position.z;
         quaternion.x() = states->pose[i].orientation.x;
         quaternion.y() =states->pose[i].orientation.y;
         quaternion.z() =states->pose[i].orientation.z;
         quaternion.w() =states->pose[i].orientation.w;

         R_m = quaternion.normalized().toRotationMatrix();
         euler = R_m.eulerAngles(0,1,2); //xyz
         break; 
       } 
     }
    //  std::cout<<"position: "<<position.transpose()<<std::endl
    //           <<"euler(xyz): "<<euler.transpose()<<std::endl;
            
}  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lift_pos");


  ros::NodeHandle n;
   
  ros::Publisher f_l_prismatic_joint_position_pub = n.advertise<std_msgs::Float64>("/mobile_base/f_l_prismatic_joint_position_controller/command", 1000);
  ros::Publisher f_r_prismatic_joint_position_pub = n.advertise<std_msgs::Float64>("/mobile_base/f_r_prismatic_joint_position_controller/command", 1000);
  ros::Publisher r_l_prismatic_joint_position_pub = n.advertise<std_msgs::Float64>("/mobile_base/r_l_prismatic_joint_position_controller/command", 1000);
  ros::Publisher r_r_prismatic_joint_position_pub = n.advertise<std_msgs::Float64>("/mobile_base/r_r_prismatic_joint_position_controller/command", 1000);

  ros::Publisher f_l_prismatic_joint_state_pub = n.advertise<std_msgs::Float64>("/f_l_prismatic_joint_state", 1000);
  ros::Publisher f_r_prismatic_joint_state_pub = n.advertise<std_msgs::Float64>("/f_r_prismatic_joint_state", 1000);
  ros::Publisher r_l_prismatic_joint_state_pub = n.advertise<std_msgs::Float64>("/r_l_prismatic_joint_state", 1000);
  ros::Publisher r_r_prismatic_joint_state_pub = n.advertise<std_msgs::Float64>("/r_r_prismatic_joint_state", 1000);
  ros::Subscriber sub = n.subscribe("/mobile_base/joint_states", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("/pjoint/command", 1000, chatterCallback2);
  ros::Subscriber sub3 = n.subscribe("/gazebo/model_states", 1000, model_states_callback);
  ros::Rate loop_rate(1000);

 

  while (ros::ok())
  {
    
    f_r_prismatic_joint_position_pub.publish(position1); //d1
    f_l_prismatic_joint_position_pub.publish(position2); //d2
    r_l_prismatic_joint_position_pub.publish(position3); //d3 
    r_r_prismatic_joint_position_pub.publish(position4); //d4

    // f_l_prismatic_joint_state_pub.publish(state1);
    // f_r_prismatic_joint_state_pub.publish(state2);
    // r_l_prismatic_joint_state_pub.publish(state3);
    // r_r_prismatic_joint_state_pub.publish(state4);
   
    ros::spinOnce();
    
    loop_rate.sleep();
    
  }
  return 0;
}
