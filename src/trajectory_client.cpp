#include "ros/ros.h"
#include "kinova_msgs/AddPoseToCartesianTrajectory.h"
#include "kinova_msgs/ClearTrajectories.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_client");

  ros::NodeHandle n;
  ros::ServiceClient clear_trajectory_client = n.serviceClient<kinova_msgs::ClearTrajectories>("/j2s7s300_driver/in/clear_trajectories");
  kinova_msgs::ClearTrajectories srv;

  if (clear_trajectory_client.call(srv))
  {
    ROS_INFO("Result");
  }
  else
  {
    ROS_ERROR("Failed to call service clear_trajectory_client");
    return 1;
  }

  return 0;
}
