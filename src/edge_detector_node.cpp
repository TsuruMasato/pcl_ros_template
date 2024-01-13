#include <ros/ros.h>
#include <edge_detector.h>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "edge_detector_node");

  // Create a ROS node handle
  ros::NodeHandle nh;
  edge_detector::EdgeDetector edge_detector(nh);

  // TODO: Add your code here
  ROS_INFO("[EdgeDetectorNode] Starting node");

  ros::Rate loop_rate(10);  // 10Hz
  // Spin the ROS node

  while (ros::ok())
  {
    ros::spinOnce();
    // loop_rate.sleep(); // Activate this line if your PC is poor.
  }
  
  ROS_INFO("[EdgeDetectorNode] Ending node");

  return 0;
}
