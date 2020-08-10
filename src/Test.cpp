#include "ros/ros.h"
#include "Traj_tracking/Command.h"

void messageCallback(const Traj_tracking::Command::ConstPtr& msg){
    //ROS_INFO("I heard: [%s]", msg->Steer_angle);
    std::cout<<msg->Steer_angle<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Test_Sub");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ControlCommand", 1000, messageCallback);
    ros::spin();
    return 0;
}
