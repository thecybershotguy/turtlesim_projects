#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <sstream>

void chatterCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
    ROS_INFO("X: %f" , pose_message->x );
    ROS_INFO("Y: %f" , pose_message->y );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SimplepSubExample");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/turtle1/pose", 1000, chatterCallback);
  ros::spin();
}