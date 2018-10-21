#include "asn1_bitstream_transform_processer.hpp"

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>

namespace infuse_debug_tools
{

geometry_msgs::TransformStamped ASN1BitstreamTransformProcesser::process_pose(const asn1SccTransformWithCovariance& asn1_pose) const
{
  geometry_msgs::TransformStamped ros_msg;
  if (publish_asn1_time_) {
    ros_msg.header.stamp = ros::Time::now();
  } else {
    // Using child time. Not a problem because it should be equal to parentTime for a normal pose
    ros_msg.header.stamp.fromNSec((uint64_t)asn1_pose.metadata.childTime.microseconds * 1000ull);
  }
  fromASN1SCC(asn1_pose.metadata.parentFrameId, ros_msg.header.frame_id);
  fromASN1SCC(asn1_pose.metadata.childFrameId, ros_msg.child_frame_id);
  ros_msg.transform.translation.x = asn1_pose.data.translation.arr[0];
  ros_msg.transform.translation.y = asn1_pose.data.translation.arr[1];
  ros_msg.transform.translation.z = asn1_pose.data.translation.arr[2];
  ros_msg.transform.rotation.x = asn1_pose.data.orientation.arr[0];
  ros_msg.transform.rotation.y = asn1_pose.data.orientation.arr[1];
  ros_msg.transform.rotation.z = asn1_pose.data.orientation.arr[2];
  ros_msg.transform.rotation.w = asn1_pose.data.orientation.arr[3];

  return std::move(ros_msg);
}

geometry_msgs::TransformStamped ASN1BitstreamTransformProcesser::process_delta_pose(const asn1SccTransformWithCovariance& asn1_pose) const
{
  // This is ugly,
  throw "Received a delta pose but delta pose processing is not yet implemented";

  geometry_msgs::TransformStamped ros_msg;
  return std::move(ros_msg);
}

} // namespace infuse_debug_tools