#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send zone1 requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_zone1s node
  ros::init(argc, argv, "simple_navigation_zone1s");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal zone1;
  zone1.target_pose.header.frame_id = "map";
  zone1.target_pose.header.stamp = ros::Time::now();
  zone1.target_pose.pose.position.x = 1.0;
  zone1.target_pose.pose.position.y = 3.0;
  zone1.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending pick up zone1");
  ac.sendGoal(zone1);
  ac.waitForResult();

  // Check if the robot reached its zone1
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Succeeded reaching zone1");
  else
    ROS_INFO("Failed to reach zone1");

  ros::Duration(5.0).sleep();

  move_base_msgs::MoveBaseGoal zone2;
  zone2.target_pose.header.frame_id = "map";
  zone2.target_pose.header.stamp = ros::Time::now();
  zone2.target_pose.pose.position.x = -3.0;
  zone2.target_pose.pose.position.y = 3.0;
  zone2.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending pick up zone2");
  ac.sendGoal(zone2);
  ac.waitForResult();

  // Check if the robot reached zone2
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Succeeded reaching zone2");
  else
    ROS_INFO("Failed to reach zone2");

  return 0;
}
