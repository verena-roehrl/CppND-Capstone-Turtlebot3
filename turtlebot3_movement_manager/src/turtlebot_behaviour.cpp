#include <ros/ros.h>



class TurtlebotBehaviour{

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlebot_behaviour");
    ros::NodeHandle nh;
    ROS_INFO("TEST");
    ros::spin();
    return 0;
}