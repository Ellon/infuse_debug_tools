#ifndef GPS_EXTRACTOR_HPP
#define GPS_EXTRACTOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_novatel_gps_msgs/UtmInfo.h>
#include <infuse_asn1_types/TransformWithCovariance.h>

#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

namespace infuse_debug_tools {

class GpsExtractor{
  public:
  GpsExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &pose_topic,  const std::string &info_topic, const std::string &output_name);
  void Extract();

  private:
  void ProcessGpsData(const infuse_msgs::asn1_bitstream::ConstPtr &pose_msg, const infuse_novatel_gps_msgs::UtmInfo::ConstPtr &info_msg);

  private:
  boost::filesystem::path output_dir_;
  std::vector<std::string> bag_paths_;
  std::string pose_topic_;
  std::string info_topic_;
  std::string output_name_;
  std::unique_ptr<asn1SccTransformWithCovariance> asn1_pose_ptr_;
  std::ofstream data_ofs_;
  uint32_t queue_size_;
  std::unique_ptr<boost::progress_display> show_progress_ptr_;
};

}
#endif // GPS_EXTRACTOR_HPP
