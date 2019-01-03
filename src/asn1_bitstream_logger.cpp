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

void ASN1BitstreamLogger::LogGpsPoseWithInfo(const asn1SccTransformWithCovariance & transform, const infuse_novatel_gps_msgs::UtmInfo & info, std::ofstream & ofs)
{
  LogTransformWithCovariance(transform, ofs);
  LogGpsInfo(info, ofs);
}

std::vector<std::string> ASN1BitstreamLogger::GetGpsPoseWithInfoLogEntries(std::string prefix)
{
  std::vector<std::string> entries;
  {
    std::vector<std::string> tmp = GetTransformWithCovarianceLogEntries();
    entries.insert(entries.end(), tmp.begin(), tmp.end());
  }
  {
    std::vector<std::string> tmp = GetGpsInfoLogEntries();
    entries.insert(entries.end(), tmp.begin(), tmp.end());
  }

  if (not prefix.empty())
    for (auto & entry : entries)
      entry = prefix + entry;

  return std::move(entries);
}

void ASN1BitstreamLogger::LogGpsInfo(const infuse_novatel_gps_msgs::UtmInfo & info, std::ofstream & ofs)
{
  // Convert ROS timestamp to usecs
  time_t time_usecs = info.header.stamp.sec * 1000000ull + info.header.stamp.nsec / 1000ull;

  ofs << time_usecs           << " "
      << info.solution_status << " "
      << info.position_type   << " "
      << info.easting_sigma   << " "
      << info.northing_sigma  << " "
      << info.height_sigma    << " ";
}

std::vector<std::string> ASN1BitstreamLogger::GetGpsInfoLogEntries(std::string prefix)
{
  std::vector<std::string> entries = {
    "gps_info_timestamp",
    "solution_status",
    "position_type",
    "easting_sigma",
    "northing_sigma",
    "height_sigma"
  };

  if (not prefix.empty())
    for (auto & entry : entries)
      entry = prefix + entry;

  return std::move(entries);
}

void ASN1BitstreamLogger::LogFrame(const asn1SccFrame & frame, std::ofstream & ofs)
{
  ofs << frame.metadata.timeStamp.microseconds << " "
      << frame.metadata.receivedTime.microseconds << " ";

  LogTransformWithCovariance(frame.extrinsic.pose_fixedFrame_robotFrame, ofs);
  LogTransformWithCovariance(frame.extrinsic.pose_robotFrame_sensorFrame, ofs);
}

std::vector<std::string> ASN1BitstreamLogger::GetFrameLogEntries(std::string prefix)
{
  std::vector<std::string> entries = {
    "timestamp",
    "received_time"
  };

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

void ASN1BitstreamLogger::LogFramePair(const asn1SccFramePair & frame_pair, std::ofstream & ofs)
{
  ofs << frame_pair.baseline << " ";
  LogFrame(frame_pair.left, ofs);
  LogFrame(frame_pair.right, ofs);
}

std::vector<std::string> ASN1BitstreamLogger::GetFramePairLogEntries(std::string prefix)
{
  std::vector<std::string> entries = {"baseline"};
  {
    std::vector<std::string> tmp = GetFrameLogEntries("left__");
    entries.insert(entries.end(), tmp.begin(), tmp.end());
  }
  {
    std::vector<std::string> tmp = GetFrameLogEntries("right__");
    entries.insert(entries.end(), tmp.begin(), tmp.end());
  }

  if (not prefix.empty())
    for (auto & entry : entries)
      entry = prefix + entry;

  return std::move(entries);
} 




} // infuse_debug_tools
