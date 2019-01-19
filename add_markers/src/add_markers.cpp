#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>

void show_marker(ros::Publisher& marker_pub, double x, double y)
{
  // Set our initial shape type to be a cube
  const uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
    
  marker_pub.publish(marker);
}


void hide_marker(ros::Publisher& marker_pub)
{
  const uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker = visualization_msgs::Marker();
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
}

static bool goal_reached_g = false;
static double goal_x_g = 0.0;
static double goal_y_g = 0.0;


void goal_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  const geometry_msgs::Pose p = msg->pose.pose;
  const double dx = p.position.x - goal_x_g;
  const double dy = p.position.y - goal_y_g;
  if (dx * dx + dy * dy < 0.4)
  {
    goal_reached_g = true;
  }
}


void wait_for_goal(ros::Subscriber& subscriber, double x, double y)
{
  goal_x_g = x;
  goal_y_g = y;
  goal_reached_g = false;

  while (!goal_reached_g)
  {
    if (!ros::ok())
    {
      exit(0);
    }
    ros::spinOnce();
    sleep(1);
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  show_marker(marker_pub, 1.0, 3.0);
 
  ros::Subscriber subscriber = n.subscribe("odom", 1, goal_callback);
  wait_for_goal(subscriber, 1.0, 3.0);

  hide_marker(marker_pub);

  wait_for_goal(subscriber, -3.0, 3.0);
  show_marker(marker_pub, -3.0, 3.0);
  sleep(5);
}
