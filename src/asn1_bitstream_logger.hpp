#ifndef ASN1_BITSTREAM_LOGGER_DEFINED
#define ASN1_BITSTREAM_LOGGER_DEFINED

#include <vector>
#include <string>
#include <fstream>

#include <infuse_asn1_types/TransformWithCovariance.h>
#include <infuse_asn1_types/Pointcloud.h>
#include <infuse_asn1_types/Frame.h>

#include <infuse_novatel_gps_msgs/UtmInfo.h>

#include <Eigen/Geometry>

namespace infuse_debug_tools {

class ASN1BitstreamLogger
{
public:
  static void LogTransformWithCovariance(const asn1SccTransformWithCovariance & transform, std::ofstream & ofs);
  static std::vector<std::string> GetTransformWithCovarianceLogEntries(std::string prefix = "");

  static void LogPointcloud(const asn1SccPointcloud & cloud, std::ofstream & ofs);
  static std::vector<std::string> GetPointcloudLogEntries(std::string prefix = "");

  static void LogGpsPoseWithInfo(const asn1SccTransformWithCovariance & transform, const infuse_novatel_gps_msgs::UtmInfo & info, std::ofstream & ofs);
  static std::vector<std::string> GetGpsPoseWithInfoLogEntries(std::string prefix = "");

  static void LogGpsInfo(const infuse_novatel_gps_msgs::UtmInfo & info, std::ofstream & ofs);
  static std::vector<std::string> GetGpsInfoLogEntries(std::string prefix = "");

  static void LogFrame(const asn1SccFrame & frame, std::ofstream & ofs);
  static std::vector<std::string> GetFrameLogEntries(std::string prefix = "");

  static void LogFramePair(const asn1SccFramePair & frame_pair, std::ofstream & ofs);
  static std::vector<std::string> GetFramePairLogEntries(std::string prefix = "");

  template<typename T>
  static T Yaw(const Eigen::Quaternion<T> & quat)
  {
    T yaw;
    // yaw (z-axis rotation)
    T siny = +2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
    T cosy = +1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z());  
    yaw = atan2(siny, cosy);
    return yaw;
  }

  template<typename T>
  static T Pitch(const Eigen::Quaternion<T> & quat)
  {
    T pitch;
    // pitch (y-axis rotation)
    T sinp = +2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
    if (fabs(sinp) >= 1)
      pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
      pitch = asin(sinp);
    return pitch;
  }

  template<typename T>
  static T Roll(const Eigen::Quaternion<T> & quat)
  {
    T roll;
    // roll (x-axis rotation)
    T sinr = +2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
    T cosr = +1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
    roll = atan2(sinr, cosr);
    return roll;
  }

};

} // infuse_debug_tools

#endif // ASN1_BITSTREAM_LOGGER_DEFINED
