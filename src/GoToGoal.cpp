#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

ros::Publisher velocityPublish;
ros::Subscriber positionSubcribe;
turtlesim::Pose currentPose;
geometry_msgs::Twist velocityToPublish;

const float linearVelocityConstant = 1.5;
const float angularVelocityConstant = 4;



float CalculateDistance(float goalPoseX,float goalPoseY , float currentPoseX,float currentPoseY)
{
  float difference_x_axis = goalPoseX - currentPoseX;
  float difference_y_axis = goalPoseY - currentPoseY;
  return sqrt(pow(difference_x_axis,2) + pow(difference_y_axis,2));
}

float GetAngularVelocity(float goalPoseX,float goalPoseY, float currentPoseX,float currentPoseY, float thetha)
{
  float difference_x_axis = goalPoseX - currentPoseX;
  float difference_y_axis = goalPoseY - currentPoseY;
  float steeringAngle = atan2(difference_y_axis,difference_x_axis);
  return angularVelocityConstant * (steeringAngle - thetha);
} 


void subcribedCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
  currentPose.x = pose_message->x;
  currentPose.y = pose_message->y;
  currentPose.theta = pose_message->theta;
}

void moveToGoal(float goalX , float goalY)
{
  turtlesim::Pose goalPose;
  goalPose.x = goalX ;
  goalPose.y = goalY ;
  goalPose.theta = 0;

  ros::Rate loop_rate(10);
  
  do
  {

    float linearVelocity =  linearVelocityConstant * CalculateDistance(goalPose.x,goalPose.y,currentPose.x,currentPose.y );
    float angularVelocity =  GetAngularVelocity(goalPose.x,goalPose.y,currentPose.x,currentPose.y ,currentPose.theta);

    velocityToPublish.linear.x = linearVelocity;
    velocityToPublish.linear.y = 0;
    velocityToPublish.linear.z = 0;

    velocityToPublish.angular.x = 0;
    velocityToPublish.angular.y = 0;
    velocityToPublish.angular.z = angularVelocity;

    velocityPublish.publish(velocityToPublish);

		ros::spinOnce();
		loop_rate.sleep();

  }
  while ( CalculateDistance(goalPose.x,goalPose.y,currentPose.x,currentPose.y ) >= 0.01);
  
  ROS_INFO("Reached Goal = > X: %f , Y: %f" ,currentPose.x , currentPose.y);

  velocityToPublish.linear.x = 0;
  velocityToPublish.angular.z = 0;
  velocityPublish.publish(velocityToPublish);
}


int main(int argc, char **argv)
{

  float x;
  float y;

  ros::init(argc, argv, "TestController");

  ros::NodeHandle nodeHandle("~");

  nodeHandle.getParam("x", x);

  nodeHandle.getParam("y", y);

  if (nodeHandle.hasParam("x") == false) x = 1;
  if (nodeHandle.hasParam("y") == false) y = 1;
  
  positionSubcribe = nodeHandle.subscribe("/turtle1/pose", 1000, subcribedCallback);

  velocityPublish = nodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  moveToGoal(x , y);

  ros::spin();
  ros::shutdown();

	return 0;
}
