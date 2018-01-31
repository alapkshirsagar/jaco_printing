#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <kinova_driver/kinova_ros_types.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_trajectory");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  // ^^^^^
  //
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // Home position for jaco:
//  currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z]
//  currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072]
  tf::Pose Home;
  Home.setOrigin(tf::Vector3(0.212322831154, -0.257197618484, 0.509646713734));
  Home.setRotation(kinova::EulerXYZ2Quaternion(1.63771402836,1.11316478252, 0.134094119072));
//  Home.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  tf::Pose random_pose;
  random_pose.setOrigin(tf::Vector3(0.54882, -0.30854,  0.65841));
  random_pose.setRotation(tf::Quaternion(0.68463, -0.22436, 0.68808, 0.086576));

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list.
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose start_pose2; // start from Home pose of jaco
  tf::poseTFToMsg(Home, start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;
  target_pose3.position.x += 0.02;
  target_pose3.position.z += 0.02;
  waypoints.push_back(target_pose3);  // up and out

  target_pose3.position.y -= 0.02;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.z -= 0.02;
  target_pose3.position.y += 0.02;
  target_pose3.position.x -= 0.02;
  waypoints.push_back(target_pose3);  // down and right (back to start)

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(15.0);

  ros::shutdown();
  return 0;
}
