#ifndef POINT_CLOUD_EXTRACTOR_HPP
#define POINT_CLOUD_EXTRACTOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/Pointcloud.h>

#include <Eigen/Geometry>

#include <boost/filesystem.hpp>

#include <pcl/visualization/pcl_visualizer.h>

namespace infuse_debug_tools {

class PointCloudExtractor {
public:
  typedef pcl::PointXYZI Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef pcl::visualization::PointCloudColorHandler<Point> ColorHandler;
  typedef ColorHandler::Ptr ColorHandlerPtr;
  typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

public:
  PointCloudExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &point_cloud_topic, bool extract_pngs = false);
  void Extract();

private:
  void ProcessPointCloud(const infuse_msgs::asn1_bitstream::Ptr& msg);
  Eigen::Affine3d ConvertAsn1PoseToEigen(const asn1SccTransformWithCovariance& asn1_pose);

private:
  //! Directory where to put the dataset
  boost::filesystem::path output_dir_;
  //! Strings with paths to the bags to be processed
  std::vector<std::string> bag_paths_;
  //! Name of the topic with the point clouds
  std::string point_cloud_topic_;
  //! Directory where to put the pcd files (set on Extract())
  boost::filesystem::path data_dir_;
  //! Variable used to decode the ASN1 message into.
  std::unique_ptr<asn1SccPointcloud> asn1_pointcloud_ptr_;
  //! Counter for the number of pcd files written. Used to create the pcd filename
  size_t pcd_count_;
  //! Number of characters to be used when creating pcd filename
  unsigned int length_pcd_filename_;
  //! Stream used to write the metadata file
  std::ofstream metadata_ofs_;
  //! Store if we want to extract pngs using PCLVisualizer
  bool extract_pngs_;
  //! Viewer used to render pngs to be dumped
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer_;
  //! Variable used to color pointclouds during png extraction
  ColorHandlerPtr color_handler_;
  //! Point size for PNG extraction
  double point_size_;
  //! Directory where to put the png files (set on Extract())
  boost::filesystem::path png_dir_;
};

} // infuse_debug_tools

#endif // POINT_CLOUD_EXTRACTOR_HPP
