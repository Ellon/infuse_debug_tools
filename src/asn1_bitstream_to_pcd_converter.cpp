#include <memory>

#include <ros/ros.h>
#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/Pointcloud.h>

// #include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <pcl/io/pcd_io.h>


// Global variables. Ugly, but well... this is just a dirty debug program.
std::unique_ptr<asn1SccPointcloud> asn1PointCloud_ptr;
std::string topic;
size_t pcd_count;
bool pcd_binary_mode;


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