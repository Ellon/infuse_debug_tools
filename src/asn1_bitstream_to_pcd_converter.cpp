#include <memory>

#include <ros/ros.h>
#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/Pointcloud.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>

// Global variables. Ugly, but well... this is just a dirty debug program.
std::unique_ptr<asn1SccPointcloud> asn1PointCloud_ptr;
std::string topic;
size_t pcd_count;
bool pcd_binary_mode;

Eigen::Affine3d convert_asn1_pose_to_eigen(const asn1SccTransformWithCovariance& asn1_pose)
{
  Eigen::Matrix3d m; m = Eigen::Quaterniond(asn1_pose.data.orientation.arr[3],
                                            asn1_pose.data.orientation.arr[0],
                                            asn1_pose.data.orientation.arr[1],
                                            asn1_pose.data.orientation.arr[2]);
  Eigen::Affine3d eigen_pose;
  eigen_pose.linear() = m;
  eigen_pose.translation() << asn1_pose.data.translation.arr[0], asn1_pose.data.translation.arr[1], asn1_pose.data.translation.arr[2];
  return eigen_pose;
}

void callback(const infuse_msgs::asn1_bitstream::Ptr& msg)
{
  // Initialize asn1 point cloud to be sure we have clean object.
  asn1SccPointcloud_Initialize(asn1PointCloud_ptr.get());

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
  res = asn1SccPointcloud_Decode(asn1PointCloud_ptr.get(), &bstream, &errorCode);
  if (not res) {
    ROS_INFO("Error decoding asn1Pointcloud! Error: %d", errorCode);
    return;
  }

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  fromASN1SCC(*asn1PointCloud_ptr, pcl_cloud);

  // Fill viewpoint
  Eigen::Affine3d T_fixed_robot = convert_asn1_pose_to_eigen(asn1PointCloud_ptr->metadata.pose_fixedFrame_robotFrame);
  Eigen::Affine3d T_robot_sensor = convert_asn1_pose_to_eigen(asn1PointCloud_ptr->metadata.pose_robotFrame_sensorFrame);
  Eigen::Affine3f T_fixed_sensor = (T_fixed_robot * T_robot_sensor).cast<float>();

  // TODO: Test if the transformation is rigid
  // Something like the code below could be used
  // const float epsilon = 0.001;
  // Eigen::Matrix3f R = T_fixed_sensor.linear();
  // if(anyabs(1 - R.determinant()) > epsilon)
  //   return NOT_RIGID;
  // else
  //   return RIGID;

  pcl_cloud.sensor_origin_ << T_fixed_sensor.translation(), 0.0;
  pcl_cloud.sensor_orientation_ = Eigen::Quaternionf(T_fixed_sensor.linear());

  // Save pcd
  std::string pcd_file_prefix = topic;
  std::replace( pcd_file_prefix.begin(), pcd_file_prefix.end(), '/', '_');
  std::stringstream ss;
  ss << pcd_file_prefix << "_" << pcd_count << ".pcd";
  pcl::io::savePCDFile( ss.str(), pcl_cloud, pcd_binary_mode );

  // Increment pcd counter
  pcd_count++;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "asn1_bitstream_to_pcd_converter");

  ros::NodeHandle n;

  // Initialize globals
  asn1PointCloud_ptr = std::make_unique<asn1SccPointcloud>();
  topic = "/velodyne/point_cloud";
  pcd_count = 0;
  pcd_binary_mode = true;

  ros::Subscriber sub = n.subscribe(topic, 1000, callback);

  ros::spin();

  return 0;
}