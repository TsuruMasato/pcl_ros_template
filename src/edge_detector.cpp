#include <edge_detector.h>

namespace edge_detector
{

EdgeDetector::EdgeDetector(ros::NodeHandle& nh)
{
  ROS_INFO("[EdgeDetector] Constructor called");
  nh_ = nh;
  sub_point_cloud_ = nh_.subscribe("point_cloud_topic", 1, &EdgeDetector::main_callback, this);
  pub_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/edge_detector/debug_point_cloud", 1);

  nh_.param<bool>("/edge_detector_node/use_voxel_grid_filter", use_voxel_grid_filter_, use_voxel_grid_filter_);
  nh_.param<double>("/edge_detector_node/voxel_resolution", voxel_resolution_, voxel_resolution_);
}

// EdgeDetector::~EdgeDetector()
// {
//   ROS_INFO("[EdgeDetector] Destructor called");
// }

void EdgeDetector::main_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
  ROS_INFO("[EdgeDetector] main_callback called");

  /* convert ros communication message to PCL point cloud */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input_msg, *input_cloud);

  detectEdges(input_cloud);
}

void EdgeDetector::detectEdges(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud)
{
  ROS_INFO("[EdgeDetector] detectEdges called");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  *debug_cloud = *input_cloud;

  if(use_voxel_grid_filter_)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> grid_filter;
    grid_filter.setInputCloud(input_cloud);
    grid_filter.setLeafSize(voxel_resolution_, voxel_resolution_,voxel_resolution_);
    grid_filter.filter(*debug_cloud);
  }

  pub_point_cloud_.publish(debug_cloud);
}

} // namespace edge_detector