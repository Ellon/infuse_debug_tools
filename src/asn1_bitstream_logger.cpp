#include "asn1_bitstream_logger.hpp"

namespace infuse_debug_tools {

void ASN1BitstreamLogger::LogTransformWithCovariance(const asn1SccTransformWithCovariance & transform, std::ofstream & ofs)
{
  Eigen::Quaterniond quat(transform.data.orientation.arr[3],
                          transform.data.orientation.arr[0],
                          transform.data.orientation.arr[1],
                          transform.data.orientation.arr[2]);

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
      << Yaw(quat) << " " 
      << "\n";
}

std::vector<std::string> ASN1BitstreamLogger::GetTransformWithCovarianceLogEntries(std::string prefix)
{
  std::vector<std::string> entries = {"parent_time", "child_time", "x", "y", "z", "qw", "qx", "qy", "qz", "q_norm", "roll", "pitch", "yaw"};
  for (auto & entry : entries)
    entry = prefix + entry;
  return std::move(entries);
}

double ASN1BitstreamLogger::Yaw(const Eigen::Quaterniond & quat)
{
  double yaw;
  // yaw (z-axis rotation)
  double siny = +2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
  double cosy = +1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z());  
  yaw = atan2(siny, cosy);
  return yaw;
}

double ASN1BitstreamLogger::Pitch(const Eigen::Quaterniond & quat)
{
  double pitch;
  // pitch (y-axis rotation)
  double sinp = +2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);
  return pitch;
}

double ASN1BitstreamLogger::Roll(const Eigen::Quaterniond & quat)
{
  double roll;
  // roll (x-axis rotation)
  double sinr = +2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
  double cosr = +1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
  roll = atan2(sinr, cosr);
  return roll;
}

} // infuse_debug_tools
