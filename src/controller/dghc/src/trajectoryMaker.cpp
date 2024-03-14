#include "trajectory.hpp"

using namespace std::chrono_literals;
class trajectory_maker : public rclcpp::Node
{
    public:
        trajectory_maker();
        void run();
        void snap_current(const geometry_msgs::msg::Pose pose,int CurrentSTEP,double current_secs);
        void desired_pose_function(int STEP);
        void caculate_gain_period(); 
        void Maketrajectory(double secs);
        void calculate_pose_error();

        void current_pose_callback(const geometry_msgs::msg::Pose & pose);
        void current_step_callback(const std_msgs::msg::Int16 & step);
    
    private:
    void timer_callback();
   
    geometry_msgs::msg::Pose desired_trajectory;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::Pose desired_pose;
    Eigen::VectorXd desired_poseV = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd current_poseV = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd trajectory_tmp = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd gain = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd period = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd arrived = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd snap_pose = Eigen::VectorXd::Zero(7);
    Eigen::Matrix3d cRd_tmp = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d cRd = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d wRsn = Eigen::Matrix3d::Identity();
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();

    std_msgs::msg::Bool realarrived1;
    std_msgs::msg::Bool realarrived2;
    std_msgs::msg::Bool realarrived5;

    int realarrived_cnt = 0;
    bool flag1=0;
    bool flag2=0;
    int STEP = 0;
    int PastSTEP =-1;
    double snap_secs = 0;
    double d_radian_tmp = 0;
    double desired_radian = 0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr currentpose_Sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr STEP_Sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr trajectory_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr realarrived_pub1;
};

trajectory_maker::trajectory_maker() :Node("trajectory_maker")
{
    timer_ = this->create_wall_timer(1ms, std::bind(&trajectory_maker::timer_callback, this));
    trajectory_pub = this->create_publisher<geometry_msgs::msg::Pose>("/end_desired_pose", 1000);
 
    realarrived_pub1 = this->create_publisher<std_msgs::msg::Bool>("/arrived1", 1);
    
    //realarrivedPub2 = nh.advertise<std_msgs::Bool>("/arrived2", 1);
    //realarrivedPub5 = nh.advertise<std_msgs::Bool>("/arrived5", 1);
    
    currentpose_Sub = this->create_subscription<geometry_msgs::msg::Pose>("/estimated_end_pose", 1000, std::bind(&trajectory_maker::current_pose_callback, this, std::placeholders::_1));  
    STEP_Sub = this->create_subscription<std_msgs::msg::Int16>("/current_step", 1000, std::bind(&trajectory_maker::current_step_callback, this,std::placeholders::_1));
   
}

void::trajectory_maker::timer_callback()
{
   
    if(flag1 ==1 && flag2 ==1)
    {
        trajectory_pub->publish(desired_trajectory);
        realarrived_pub1->publish(realarrived1);
        //realarrivedPub2.publish(realarrived2);
        //realarrivedPub5.publish(realarrived5);
        //std::cout<<"communication connected!"<<std::endl;
    }
}


void trajectory_maker::current_pose_callback(const geometry_msgs::msg::Pose & pose)
{
    current_pose.position.x = pose.position.x;
    current_pose.position.y = pose.position.y;
    current_pose.position.z = pose.position.z;
    current_pose.orientation.x = pose.orientation.x;
    current_pose.orientation.y = pose.orientation.y;
    current_pose.orientation.z = pose.orientation.z;
    current_pose.orientation.w = pose.orientation.w;

    current_poseV(0) = pose.position.x;
    current_poseV(1) = pose.position.y; 
    current_poseV(2) = pose.position.z; 
    current_poseV(3) = pose.orientation.x;
    current_poseV(4) = pose.orientation.y;
    current_poseV(5) = pose.orientation.z;
    current_poseV(6) = pose.orientation.w;
    

    flag1 = 1;
}
void trajectory_maker::current_step_callback(const std_msgs::msg::Int16 & step)
{
    STEP = step.data;
    flag2 = 1;
}

void trajectory_maker::snap_current(const geometry_msgs::msg::Pose pose,int CurrentSTEP,double current_secs)
{
   
    if(PastSTEP != CurrentSTEP)
    {   
        snap_pose(0) = pose.position.x;
        snap_pose(1) = pose.position.y;
        snap_pose(2) = pose.position.z;
        snap_pose(3) = pose.orientation.x;
        snap_pose(4) = pose.orientation.y;
        snap_pose(5) = pose.orientation.z;
        snap_pose(6) = pose.orientation.w;
        PastSTEP = CurrentSTEP;
        snap_secs = current_secs;
        realarrived_cnt = 0;
        realarrived1.data = 0;
        realarrived2.data = 0;
        realarrived5.data = 0;
        arrived<<0,0,0,0;
        trajectory_tmp = snap_pose;
        d_radian_tmp = 0;

    }
     
}

void trajectory_maker::desired_pose_function(int currentSTEP)
{
    if(currentSTEP == 0)
    {
        
        desired_poseV(0)= current_poseV(0);  
        desired_poseV(1)= current_poseV(1);
        desired_poseV(2)= current_poseV(2);
        desired_poseV(3)= current_poseV(3);
        desired_poseV(4)= current_poseV(4);
        desired_poseV(5)= current_poseV(5);
        desired_poseV(6)= current_poseV(6);
        
        
        
        
    }
    else if(currentSTEP==1) // pick tool
    {
        desired_poseV(0) = 3; //ref : world , unit : m 
        desired_poseV(1) = 0.12;
        desired_poseV(2) = 1.0;
        desired_poseV(3) = 0.9268;
        desired_poseV(4) = 0.3756;
        desired_poseV(5) = 0;
        desired_poseV(6) = 0;
        
        //desired_poseV(0) = 2.23; //ref : world , unit : m 
        //desired_poseV(1) = 0.0;
        //desired_poseV(2) = 1.25;
        //desired_poseV(3) = 0.7071;
        //desired_poseV(4) = 0.0;
        //desired_poseV(5) = 0.7071;
        //desired_poseV(6) = 0.0;
    }
    else if(currentSTEP == 2) // to human
    {
        desired_poseV(0) = 3; //ref : world , unit : m 
        desired_poseV(1) = 0.12;
        desired_poseV(2) = 1.3;
        desired_poseV(3) = 0.9268;
        desired_poseV(4) = 0.3756;
        desired_poseV(5) = 0;
        desired_poseV(6) = 0;
    }
    
    else if(currentSTEP == 3) // hold pose 
    {
        desired_poseV(0)= current_poseV(0);  
        desired_poseV(1)= current_poseV(1);
        desired_poseV(2)= current_poseV(2);
        desired_poseV(3)= current_poseV(3);
        desired_poseV(4)= current_poseV(4);
        desired_poseV(5)= current_poseV(5);
        desired_poseV(6)= current_poseV(6);
    }
    else if(currentSTEP == 4) //hand guidance
    {
        desired_poseV(0)= current_poseV(0);  
        desired_poseV(1)= current_poseV(1);
        desired_poseV(2)= current_poseV(2);
        desired_poseV(3)= current_poseV(3);
        desired_poseV(4)= current_poseV(4);
        desired_poseV(5)= current_poseV(5);
        desired_poseV(6)= current_poseV(6);
    }
    else if(currentSTEP == 5) // place tool 
    {
        desired_poseV(0)= current_poseV(0);  
        desired_poseV(1)= current_poseV(1);
        desired_poseV(2)= current_poseV(2);
        desired_poseV(3)= current_poseV(3);
        desired_poseV(4)= current_poseV(4);
        desired_poseV(5)= current_poseV(5);
        desired_poseV(6)= current_poseV(6);
    }

}

void trajectory_maker::caculate_gain_period()
{
    for(int i=0;i<3;i++)
    {
        gain(i) = (desired_poseV(i) - snap_pose(i));
        if(abs(gain(i))<= 0.1)
        {
            period(i) = 5; //sec
        
        }
        else
        {
            period(i) = 25 * abs(gain(i));
        }
    }

    Eigen::Quaterniond wQd(desired_poseV(6),desired_poseV(3),desired_poseV(4),desired_poseV(5));
    Eigen::Quaterniond wQsn(snap_pose(6),snap_pose(3),snap_pose(4),snap_pose(5)); //snapped quaternion
    Eigen::Matrix3d wRd = wQd.normalized().toRotationMatrix();
    wRsn = wQsn.normalized().toRotationMatrix();
        
    cRd = wRsn.transpose()*wRd;
    Eigen::AngleAxisd AngleAxis(cRd);

    desired_radian =  AngleAxis.angle();
    axis = AngleAxis.axis();
 
    gain(3) = (desired_radian);
    if(abs(gain(3))<= 0.1)
    {
        period(3) = 5; //sec
    
    }
    else
    {
        period(3) = 25 * abs(gain(3));
    }
    
}

void trajectory_maker::Maketrajectory(double current_secs)
{
       
        for(short i=0;i<3;i++)
        {   
            if(abs(trajectory_tmp(i) - desired_poseV(i))<= 0.001)
            {   
                
                arrived(i) = 1;   
            }
            //std::cout<<i<<" arrived: "<<arrived(i) <<std::endl; 
            if(arrived(i) == 1)
            {
                trajectory_tmp(i) = desired_poseV(i);
            }
            else if(arrived(i) == 0)
            {
                trajectory_tmp(i) = snap_pose(i) + gain(i)*sin(((2*M_PI)/period(i))*(current_secs - snap_secs));
            }
            
        }
     
        if(abs(d_radian_tmp - desired_radian )<= 0.001)
        {
            arrived(3) = 1;
        }
        //std::cout<<"3 arrived: "<<arrived(3) <<std::endl;
        if(arrived(3) ==1)
        {
            cRd_tmp = cRd;
        }
        else
        {
            d_radian_tmp = gain(3)*sin(((2*M_PI)/period(3))*(current_secs - snap_secs));
            cRd_tmp = Eigen::AngleAxisd(d_radian_tmp,axis);
        }
  
    Eigen::Matrix3d wRd_tmp = wRsn*cRd_tmp;
    Eigen::Quaterniond wQd_tmp(wRd_tmp);

    desired_trajectory.position.x = trajectory_tmp(0);  
    desired_trajectory.position.y = trajectory_tmp(1);
    desired_trajectory.position.z = trajectory_tmp(2);
    desired_trajectory.orientation.x = wQd_tmp.x();
    desired_trajectory.orientation.y = wQd_tmp.y();
    desired_trajectory.orientation.z = wQd_tmp.z();
    desired_trajectory.orientation.w = wQd_tmp.w();
}

void trajectory_maker::calculate_pose_error() 
{
    Eigen::VectorXd error = Eigen::VectorXd::Zero(3);
    error(0) = desired_poseV(0) - current_pose.position.x;
    error(1) = desired_poseV(1) - current_pose.position.y;
    error(2) = desired_poseV(2) - current_pose.position.z;
    
    
    Eigen::Quaterniond wQc(current_poseV(6),current_poseV(3),current_poseV(4),current_poseV(5)); //snapped quaternion
    Eigen::Quaterniond wQd(desired_poseV(6),desired_poseV(3),desired_poseV(4),desired_poseV(5));
    Eigen::Matrix3d wRc = wQc.normalized().toRotationMatrix();
    Eigen::Matrix3d wRd = wQd.normalized().toRotationMatrix();
    Eigen::Matrix3d cRd = wRc.transpose()*wRd;
    Eigen::Quaterniond cQd(cRd);
    
    Eigen::VectorXd orientatin_error = Eigen::VectorXd::Zero(3);
    orientatin_error(0) = cQd.x();
    orientatin_error(1) = cQd.y();
    orientatin_error(2) = cQd.z();

    std::cout<<"error: "<<error.transpose()<<std::endl;
    std::cout<<"orientstion_error: "<<orientatin_error<<std::endl;

    if(sqrt(error(0)*error(0) +error(1)*error(1) +error(2)*error(2)) <= 0.1 && orientatin_error.norm() <= 0.5 )
    {
        realarrived_cnt = realarrived_cnt + 1;
    }
    else
    {
        realarrived_cnt = 0;
    }

    if(realarrived_cnt >= 10)
    {
        if(STEP ==1)
        {
            realarrived1.data = 1;
        }
        else if(STEP ==2)
        {
            realarrived2.data = 1;
        }
        if(STEP ==5)
        {
            realarrived5.data = 1;
        }
    }
    else
    {
        realarrived1.data = 0;
        realarrived2.data = 0;
        realarrived5.data = 0;
    }
}
void trajectory_maker::run()
{
    rclcpp::Rate loop_rate(1000);
    
    bool count1 = 0;
    bool count2 = 0;

    while(rclcpp::ok())
    {   if(flag1 ==1 && count1 == 0)
        {
            std::cout<<"current_pose subscribed"<<std::endl;
            count1 = 1;
        }
        if(flag2 ==1 && count2==0)
        {
            std::cout<<"current_step subscribed"<<std::endl;
            count2 = 1;
        }

         if(flag1 ==1 && flag2 ==1)
        {    
            
            double secs = rclcpp::Clock{}.now().seconds();
            //std::cout<<"currentSTEP: "<<STEP<<" pastStep: "<<PastSTEP<<std::endl;
            
            
            snap_current(current_pose,STEP,secs);
            desired_pose_function(STEP);
            caculate_gain_period(); 
            Maketrajectory(secs);
            calculate_pose_error();
            
        }
       
            
        loop_rate.sleep();   
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto trajectory_node = std::make_shared<trajectory_maker>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(trajectory_node);
    std::thread executor_thread([&executor]() { executor.spin(); });
    
    trajectory_node->run();

    executor_thread.join();
    rclcpp::shutdown();
    
    return 0;
}