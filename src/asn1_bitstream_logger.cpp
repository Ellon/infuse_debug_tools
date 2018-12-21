#include "asn1_bitstream_logger.hpp"

namespace infuse_debug_tools {

void ASN1BitstreamLogger::LogTransformWithCovariance(const asn1SccTransformWithCovariance & transform, std::ofstream & ofs)
{
  Eigen::Quaterniond quat(transform.data.orientation.arr[3],
                          transform.data.orientation.arr[0],
                          transform.data.orientation.arr[1],
                          transform.data.orientation.arr[2]);

  ofs.precision(10);
  ofs << transform.metadata.parentTime.microseconds << " "
      << transform.metadata.childTime.microseconds << " "
      << transform.data.translation.arr[0] << " "
      << transform.data.translation.arr[1] << " "
      << transform.data.translation.arr[2] << " "
      << quat.w() << " "
      << quat.x() << " "
      << quat.y() << " "
      << quat.z() << " " 
      << quat.norm()  << " " 
      << Roll(quat) << " "
      << Pitch(quat) << " "
      << Yaw(quat) << " ";
}

std::vector<std::string> ASN1BitstreamLogger::GetTransformWithCovarianceLogEntries(std::string prefix)
{
  std::vector<std::string> entries = {"parent_time", "child_time", "x", "y", "z", "qw", "qx", "qy", "qz", "q_norm", "roll", "pitch", "yaw"};
  if (not prefix.empty())
    for (auto & entry : entries)
      entry = prefix + entry;
  return std::move(entries);
}

void ASN1BitstreamLogger::LogPointcloud(const asn1SccPointcloud & cloud, std::ofstream & ofs)
{
  ofs << cloud.metadata.timeStamp.microseconds << " ";
  LogTransformWithCovariance(cloud.metadata.pose_fixedFrame_robotFrame, ofs);
  LogTransformWithCovariance(cloud.metadata.pose_robotFrame_sensorFrame, ofs);
}

std::vector<std::string> ASN1BitstreamLogger::GetPointcloudLogEntries(std::string prefix)
{
  std::vector<std::string> entries = {"cloud_time"};
  {
    std::vector<std::string> tmp = GetTransformWithCovarianceLogEntries("pose_fixed_robot__");
    entries.insert(entries.end(), tmp.begin(), tmp.end());
  }
  {
    std::vector<std::string> tmp = GetTransformWithCovarianceLogEntries("pose_robot_sensor__");
    entries.insert(entries.end(), tmp.begin(), tmp.end());
  }
  if (not prefix.empty())
    for (auto & entry : entries)
      entry = prefix + entry;
  return std::move(entries);
}

void ASN1BitstreamLogger::LogGps(const asn1SccTransformWithCovariance & transform, const infuse_novatel_gps_msgs::UtmInfo & info, std::ofstream & ofs)
{
  LogTransformWithCovariance(transform, ofs);
  ofs << info.solution_status << " "
      << info.position_type   << " "
      << info.easting_sigma   << " "
      << info.northing_sigma  << " "
      << info.height_sigma    << " ";
}

std::vector<std::string> ASN1BitstreamLogger::GetGpsLogEntries(std::string prefix)
{
  std::vector<std::string> entries;
  {
    std::vector<std::string> tmp = GetTransformWithCovarianceLogEntries();
    entries.insert(entries.end(), tmp.begin(), tmp.end());
  }
  entries.push_back("solution_status");
  entries.push_back("position_type");
  entries.push_back("easting_sigma");
  entries.push_back("northing_sigma");
  entries.push_back("height_sigma");

  if (not prefix.empty())
    for (auto & entry : entries)
      entry = prefix + entry;

  return std::move(entries);
}


} // infuse_debug_tools
