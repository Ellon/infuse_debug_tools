#include <map>
#include <memory>

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/TransformWithCovariance.h>
#include <infuse_asn1_conversions/asn1_base_conversions.hpp>

#include <infuse_debug_tools/ConnectTopic.h>
#include <infuse_debug_tools/AddVirtualFrame.h>

#include "asn1_bitstream_transform_processer.hpp"


namespace infuse_debug_tools
{

class ASN1BitstreamToTf : ASN1BitstreamTransformProcesser
{
public:
  ASN1BitstreamToTf();

  bool connect_pose(infuse_debug_tools::ConnectTopic::Request  &req,
                    infuse_debug_tools::ConnectTopic::Response &res);

  void pose_callback(const infuse_msgs::asn1_bitstream::Ptr& msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer connect_pose_srv_;
  std::map<std::string,ros::Subscriber> sub_map_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};


ASN1BitstreamToTf::ASN1BitstreamToTf() :
  private_nh_{"~"},
  connect_pose_srv_{private_nh_.advertiseService("connect_pose", &ASN1BitstreamToTf::connect_pose, this)}
{
  private_nh_.param<bool>("publish_asn1_time", publish_asn1_time_, false);

  std::string topics_to_connect;
  if (private_nh_.getParam("topics_to_connect", topics_to_connect)) {
    // Split strings into a vector
    std::vector<std::string> topics;
    {
      std::stringstream ss(topics_to_connect);
      std::istream_iterator<std::string> begin(ss), eos; // end-of-stream
      topics.assign(begin, eos);
    }
    // Connect to topics
    for (const auto & topic : topics) {
      try {
        sub_map_[topic] = nh_.subscribe(topic, 1000, &ASN1BitstreamToTf::pose_callback, this);
        ROS_INFO_STREAM("Connected to topic " << topic);
      } catch (...) {
        ROS_INFO_STREAM("ERROR: Could not connect to topic " << topic);
      }
    }
  }
}

bool ASN1BitstreamToTf::connect_pose(infuse_debug_tools::ConnectTopic::Request  &req,
                                     infuse_debug_tools::ConnectTopic::Response &res)
{
  try {
    sub_map_[req.topic] = nh_.subscribe(req.topic, 1000, &ASN1BitstreamToTf::pose_callback, this);
    res.success = true;
    return true;
  } catch (...) {
    std::stringstream ss;
    ss << "Unknown exception when subscribing to topic " << req.topic;
    ROS_INFO_STREAM(ss.str());
    res.success = false;
    res.message = ss.str();
    return false;
  }
}

void ASN1BitstreamToTf::pose_callback(const infuse_msgs::asn1_bitstream::Ptr& msg)
{
  // Initialize
  asn1SccTransformWithCovariance asn1Transform;
  asn1SccTransformWithCovariance_Initialize(&asn1Transform);

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
  res = asn1SccTransformWithCovariance_Decode(&asn1Transform, &bstream, &errorCode);
  if (not res) {
    ROS_INFO("Error decoding asn1SccTransformWithCovariance! Error: %d", errorCode);
    return;
  }

  // Fill TF transform
  auto is_asn1_time_equal = [](const asn1SccTime& t1, const asn1SccTime& t2){
    return (t1.microseconds == t2.microseconds) and (t1.usecPerSec == t2.usecPerSec);
  };

  // Process pose or delta pose
  geometry_msgs::TransformStamped transformStamped;
  try {
    if(is_asn1_time_equal(asn1Transform.metadata.parentTime, asn1Transform.metadata.childTime)) {
      transformStamped = process_pose(asn1Transform);
    } else {
      transformStamped = process_delta_pose(asn1Transform);
    }
  } catch (const char *exception) {
    ROS_INFO("%s", exception);
    return;
  }

  // Publish transform
  tf_broadcaster_.sendTransform(transformStamped);
}

} // namespace infuse_debug_tools

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asn1_bitstream_to_tf");

  infuse_debug_tools::ASN1BitstreamToTf asn1_bitstream_to_tf;

  ros::spin();

  return 0;
}