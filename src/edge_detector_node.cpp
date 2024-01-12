#include <ros/ros.h>
#include <edge_detector.h>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "edge_detector_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // TODO: Add your code here
  ROS_INFO("[EdgeDetectorNode] Starting node");

  // Spin the ROS node
  ros::spinOnce();

  ROS_INFO("[EdgeDetectorNode] Ending node");

  return 0;
}
