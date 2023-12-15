
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/LinkStates.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/SetLinkState.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>

#include "math.h"
#include <unistd.h>

std_msgs::Float64 effort1;
std_msgs::Float64 effort2;
std_msgs::Float64 effort3;
std_msgs::Float64 effort4;
std_msgs::Float64 state1;
std_msgs::Float64 state2;
std_msgs::Float64 state3;
std_msgs::Float64 state4;
float desired_b; 
float desired_leng=-0.05;

void chatterCallback2(const std_msgs::Float64::ConstPtr &msg2)
{
  desired_leng  = - msg2->data;
}
void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    
     
     float current_length1 = msg->position[1];
     float current_length2 = msg->position[4];
     float current_length3 = msg->position[7];
     float current_length4 = msg->position[10];

     state1.data = current_length1;
     state2.data = current_length2;
     state3.data = current_length3;
     state4.data = current_length4;



     float desired_length1 = desired_leng;
     float desired_length2 = 0.0;
     float desired_length3 = desired_leng;
     float desired_length4 = 0.0;
     
     float k = 5000; 

     float f_l_p_v = msg ->velocity[1];
     float f_r_p_v = msg ->velocity[4];
     float r_l_p_v = msg ->velocity[7];
     float r_r_p_v = msg ->velocity[10];
     float desired_vel = 0;
     float b = 1000;
// main
     effort1.data = -219.275+k*(desired_length1 - current_length1) - b*f_l_p_v;
     effort2.data = -219.275+k*(desired_length2 - current_length2) - b*f_r_p_v;
     effort3.data = -219.275+k*(desired_length3 - current_length3) - b*r_l_p_v;
     effort4.data = -219.275+k*(desired_length4 - current_length4) - b*r_r_p_v;


    ROS_INFO("[effort1, effort2, effort3, effort4]%f,%f,%f,%f",effort1.data,effort2.data,effort3.data,effort4.data);
}
  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lift");


  ros::NodeHandle n;
   
  ros::Publisher f_l_prismatic_joint_effort_pub = n.advertise<std_msgs::Float64>("/mobile_base/f_l_prismatic_joint_effort_controller/command", 1000);
  ros::Publisher f_r_prismatic_joint_effort_pub = n.advertise<std_msgs::Float64>("/mobile_base/f_r_prismatic_joint_effort_controller/command", 1000);
  ros::Publisher r_l_prismatic_joint_effort_pub = n.advertise<std_msgs::Float64>("/mobile_base/r_l_prismatic_joint_effort_controller/command", 1000);
  ros::Publisher r_r_prismatic_joint_effort_pub = n.advertise<std_msgs::Float64>("/mobile_base/r_r_prismatic_joint_effort_controller/command", 1000);

  ros::Publisher f_l_prismatic_joint_state_pub = n.advertise<std_msgs::Float64>("/f_l_prismatic_joint_state", 1000);
  ros::Publisher f_r_prismatic_joint_state_pub = n.advertise<std_msgs::Float64>("/f_r_prismatic_joint_state", 1000);
  ros::Publisher r_l_prismatic_joint_state_pub = n.advertise<std_msgs::Float64>("/r_l_prismatic_joint_state", 1000);
  ros::Publisher r_r_prismatic_joint_state_pub = n.advertise<std_msgs::Float64>("/r_r_prismatic_joint_state", 1000);
  ros::Subscriber sub = n.subscribe("/mobile_base/joint_states", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("/pjoint/command", 1000, chatterCallback2);
  ros::Rate loop_rate(1000);

 

  while (ros::ok())
  {
    
    f_l_prismatic_joint_effort_pub.publish(effort1);
    f_r_prismatic_joint_effort_pub.publish(effort2);
    r_l_prismatic_joint_effort_pub.publish(effort3);
    r_r_prismatic_joint_effort_pub.publish(effort4);

    f_l_prismatic_joint_state_pub.publish(state1);
    f_r_prismatic_joint_state_pub.publish(state2);
    r_l_prismatic_joint_state_pub.publish(state3);
    r_r_prismatic_joint_state_pub.publish(state4);
   
    ros::spinOnce();
    
    loop_rate.sleep();
    
  }
  return 0;
}
