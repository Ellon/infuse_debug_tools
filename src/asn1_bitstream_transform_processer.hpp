#ifndef ASN1_BITSTREAM_TRANSFORM_PROCESSER_DEFINED
#define ASN1_BITSTREAM_TRANSFORM_PROCESSER_DEFINED

#include <geometry_msgs/TransformStamped.h>
#include <infuse_asn1_types/TransformWithCovariance.h>

namespace infuse_debug_tools
{

class ASN1BitstreamTransformProcesser
{
protected:
  geometry_msgs::TransformStamped process_pose(const asn1SccTransformWithCovariance& asn1Transform) const;
  geometry_msgs::TransformStamped process_delta_pose(const asn1SccTransformWithCovariance& asn1Transform) const;

  bool publish_asn1_time_ = {false};
};

} // namespace infuse_debug_tools

#endif // ASN1_BITSTREAM_TRANSFORM_PROCESSER_DEFINED
