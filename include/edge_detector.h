#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace edge_detector
{

class EdgeDetector {
public:
  EdgeDetector(ros::NodeHandle& nh);
  // ~EdgeDetector();

  void main_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg);

  void detectEdges(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_marker_;

  std::string frame_id_;

  bool use_voxel_grid_filter_ = true;
  double voxel_resolution_ = 0.01;
};

} // namespace edge_detector