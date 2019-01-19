#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send pick_zone requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_pick_zones node
  ros::init(argc, argv, "simple_navigation_pick_zones");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pick_zone;
  pick_zone.target_pose.header.frame_id = "map";
  pick_zone.target_pose.header.stamp = ros::Time::now();
  pick_zone.target_pose.pose.position.x = 1.0;
  pick_zone.target_pose.pose.position.y = 3.0;
  pick_zone.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Moving to pick up zone");
  ac.sendGoal(pick_zone);
  ac.waitForResult();

  // Check if the robot reached its pick_zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Succeeded reaching pick up zone");
  else
    ROS_INFO("Failed to reach pick up zone");

  ros::Duration(5.0).sleep();

  move_base_msgs::MoveBaseGoal drop_zone;
  drop_zone.target_pose.header.frame_id = "map";
  drop_zone.target_pose.header.stamp = ros::Time::now();
  drop_zone.target_pose.pose.position.x = -3.0;
  drop_zone.target_pose.pose.position.y = 3.0;
  drop_zone.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending drop off zone");
  ac.sendGoal(drop_zone);
  ac.waitForResult();

  // Check if the robot reached drop_zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Succeeded reaching drop off zone");
  else
    ROS_INFO("Failed to reach drop off zone");

  return 0;
}
