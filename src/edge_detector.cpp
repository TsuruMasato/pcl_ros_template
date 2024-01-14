#include <edge_detector.h>

namespace edge_detector
{

EdgeDetector::EdgeDetector(ros::NodeHandle& nh)
{
  ROS_INFO("[EdgeDetector] Constructor called");
  nh_ = nh;
  sub_point_cloud_ = nh_.subscribe("point_cloud_topic", 1, &EdgeDetector::main_callback, this);
  pub_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/edge_detector/debug_point_cloud", 1);
  pub_marker_ = nh_.advertise<visualization_msgs::Marker>("/edge_detector/detected_line", 1);

  nh_.param<bool>("/edge_detector_node/use_voxel_grid_filter", use_voxel_grid_filter_, use_voxel_grid_filter_);
  nh_.param<double>("/edge_detector_node/voxel_resolution", voxel_resolution_, voxel_resolution_);
}

// EdgeDetector::~EdgeDetector()
// {
//   ROS_INFO("[EdgeDetector] Destructor called");
// }

void EdgeDetector::main_callback(const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
  // ROS_INFO("[EdgeDetector] main_callback called");
  ros::Time t_start = ros::Time::now();

  /* convert ros communication message to PCL point cloud */
  frame_id_ = input_msg->header.frame_id;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*input_msg, *input_cloud);

  detectEdges(input_cloud);

  ros::Time t_end = ros::Time::now();
  ROS_INFO_STREAM("Computation time[s] = " << (t_end - t_start).toSec());
}

void EdgeDetector::detectEdges(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud)
{
  // ROS_INFO("[EdgeDetector] detectEdges called");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  *filtered_cloud = *input_cloud;

  if(use_voxel_grid_filter_)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> grid_filter;
    grid_filter.setInputCloud(input_cloud);
    grid_filter.setLeafSize(voxel_resolution_, voxel_resolution_,voxel_resolution_);
    grid_filter.filter(*filtered_cloud);
  }

  // RANSAC
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> RANSAC_segmentation;
  // Optional
  RANSAC_segmentation.setOptimizeCoefficients(true);
  // Mandatory
  RANSAC_segmentation.setModelType(pcl::SACMODEL_PLANE);
  RANSAC_segmentation.setMethodType(pcl::SAC_RANSAC);
  RANSAC_segmentation.setDistanceThreshold(0.01);

  RANSAC_segmentation.setInputCloud(filtered_cloud);
  RANSAC_segmentation.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.setInputCloud(filtered_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*plane_cloud);

  // for(auto coeff = coefficients->values.begin(); coeff != coefficients->values.end(); coeff++)
  // {
  //   ROS_INFO_STREAM("[EdgeDetector] coefficients: " << *coeff);
  // }

  Eigen::Vector4f xyz_centroid;
  if(pcl::compute3DCentroid(*plane_cloud, xyz_centroid))
  {
    ROS_INFO_STREAM("[EdgeDetector] Centroid: " << xyz_centroid.transpose());
  }
  else
  {
    ROS_ERROR("[EdgeDetector] Centroid calculation failed");
    return;
  }

  // Visualize the 3D Line
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.ns = "edge";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::CUBE;
  marker_msg.action = visualization_msgs::Marker::ADD;
  
  marker_msg.pose.position.x = xyz_centroid.x();
  marker_msg.pose.position.y = xyz_centroid.y();
  marker_msg.pose.position.z = xyz_centroid.z();

  /* generate quaterninon from Normal vector of plane */
  Eigen::Vector3f normal_vector(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  normal_vector.normalize();
  Eigen::Vector3f z_axis(0, 0, 1);
  Eigen::Vector3f rotation_axis = normal_vector.cross(z_axis);
  if (rotation_axis.norm() < 1e-6) {
    std::runtime_error("[EdgeDetector] rotation_axis is zero vector");
    return;
  }
  rotation_axis.normalize();
  double rotation_angle = acos(normal_vector.dot(z_axis)); // 正規化されたベクトル間の角度
  Eigen::Quaternionf rotation_quaternion(Eigen::AngleAxisf(-rotation_angle, rotation_axis));
  
  marker_msg.pose.orientation.x = rotation_quaternion.x();
  marker_msg.pose.orientation.y = rotation_quaternion.y();
  marker_msg.pose.orientation.z = rotation_quaternion.z();
  marker_msg.pose.orientation.w = rotation_quaternion.w();

  marker_msg.scale.x = 1;
  marker_msg.scale.y = 1;
  marker_msg.scale.z = 0.03;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;

  // Publish the marker
  pub_marker_.publish(marker_msg);
  pub_point_cloud_.publish(plane_cloud);
}

} // namespace edge_detector