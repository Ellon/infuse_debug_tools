#ifndef ASN1_BITSTREAM_LOGGER_DEFINED
#define ASN1_BITSTREAM_LOGGER_DEFINED

#include <vector>
#include <string>
#include <fstream>

#include <infuse_asn1_types/TransformWithCovariance.h>
#include <infuse_asn1_types/Pointcloud.h>


#include <Eigen/Geometry>

namespace infuse_debug_tools {

class ASN1BitstreamLogger
{
public:
  static void LogTransformWithCovariance(const asn1SccTransformWithCovariance & transform, std::ofstream & ofs);
  static std::vector<std::string> GetTransformWithCovarianceLogEntries(std::string prefix = "");

  // void LogPointcloud(const asn1SccPointcloud & cloud, std::ofstream & ofs);
  // std::vector<std::string> GetPointcloudLogHeader(std::string prefix = "");

private:
  static double Yaw(const Eigen::Quaterniond & quat);
  static double Pitch(const Eigen::Quaterniond & quat);
  static double Roll(const Eigen::Quaterniond & quat);

};

} // infuse_debug_tools

#endif // ASN1_BITSTREAM_LOGGER_DEFINED
