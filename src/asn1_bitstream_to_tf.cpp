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

class ASN1BitstreamToTf
{
public:
  ASN1BitstreamToTf();

  bool connect_pose(infuse_debug_tools::ConnectTopic::Request  &req,
                    infuse_debug_tools::ConnectTopic::Response &res);

  bool add_virtual_frame(infuse_debug_tools::AddVirtualFrame::Request  &req,
                         infuse_debug_tools::AddVirtualFrame::Response &res);

  void pose_callback(const infuse_msgs::asn1_bitstream::Ptr& msg);

private:
  geometry_msgs::TransformStamped process_pose(const asn1SccTransformWithCovariance& asn1Transform);
  geometry_msgs::TransformStamped process_delta_pose(const asn1SccTransformWithCovariance& asn1Transform);
  geometry_msgs::TransformStamped parse_frame(const std::vector<std::string> &vstrings);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer connect_pose_srv_;
  ros::ServiceServer add_virtual_frame_srv_;
  std::map<std::string,ros::Subscriber> sub_map_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  bool publish_asn1_time_;
};


ASN1BitstreamToTf::ASN1BitstreamToTf() :
  private_nh_{"~"},
  connect_pose_srv_{private_nh_.advertiseService("connect_pose", &ASN1BitstreamToTf::connect_pose, this)},
  add_virtual_frame_srv_{private_nh_.advertiseService("add_virtual_frame", &ASN1BitstreamToTf::add_virtual_frame, this)},
  publish_asn1_time_{private_nh_.param<bool>("publish_asn1_time", false)}
{
  {
    std::vector<std::string> virtual_frames;
    if (private_nh_.getParam("virtual_frames", virtual_frames)) {
      for(const auto & virtual_frame : virtual_frames ) {
        // Split strings into a vector
        std::vector<std::string> vstrings;
        {
          std::stringstream ss(virtual_frame);
          std::istream_iterator<std::string> begin(ss), eos; // end-of-stream
          vstrings.assign(begin, eos);
        }

        // Test against wrong size
        if (vstrings.size() != 8 and vstrings.size() != 9) {
          std::stringstream ss;
          ss << "Error: parameter \"" << private_nh_.resolveName("virtual_frame") << "\" has " << vstrings.size()
             << " elements, but it must have 8 or 9 elements!\n" 
             << "Please use one of the following formats:\n"
             << "       parent_frame_id child_fram_id x y z roll pitch yaw\n"
             << "       parent_frame_id child_fram_id x y z qw qx qy qz";
          ROS_INFO_STREAM(ss.str());
          // throw std::runtime_error(ss.str());
        } else {
          geometry_msgs::TransformStamped msg{parse_frame(vstrings)};
          static_tf_broadcaster_.sendTransform(msg);
          ROS_INFO_STREAM("Publishing static virtual_frame \"" << virtual_frame << "\"");
        }        
      }
    }
  }
  {
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
}

geometry_msgs::TransformStamped ASN1BitstreamToTf::parse_frame(const std::vector<std::string> &vstrings) {
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = vstrings[0];
  msg.child_frame_id = vstrings[1];

  std::vector<double> vdoubles;
  std::transform(vstrings.begin()+2, vstrings.end(), std::back_inserter(vdoubles), [](const auto& val){
    return std::stod(val);
  });

  msg.transform.translation.x = vdoubles[0];
  msg.transform.translation.y = vdoubles[1];
  msg.transform.translation.z = vdoubles[2];
  if (vdoubles.size() == 6) {
    tf2::Quaternion quat;
    quat.setRPY(vdoubles[3], vdoubles[4], vdoubles[5]);
    msg.transform.rotation.w = quat.w();
    msg.transform.rotation.x = quat.x();
    msg.transform.rotation.y = quat.y();
    msg.transform.rotation.z = quat.z();
  } else {
    msg.transform.rotation.w = vdoubles[3];
    msg.transform.rotation.x = vdoubles[4];
    msg.transform.rotation.y = vdoubles[5];
    msg.transform.rotation.z = vdoubles[6];
  }

  return msg;
}

bool ASN1BitstreamToTf::add_virtual_frame(infuse_debug_tools::AddVirtualFrame::Request  &req,
                                          infuse_debug_tools::AddVirtualFrame::Response &res)
{
  try {
    // Split strings into a vector
    std::vector<std::string> vstrings;
    {
      std::stringstream ss(req.frame_string);
      std::istream_iterator<std::string> begin(ss), eos; // end-of-stream
      vstrings.assign(begin, eos);
    }

    // Test against wrong size
    if (vstrings.empty() or (vstrings.size() != 8 and vstrings.size() != 9)) {
      std::stringstream ss;
      ss << "Invalid frame string \"" << req.frame_string << "\"\n"
         << "Please use one of the following formats: \n"
         << "  parent_frame_id child_fram_id x y z roll pitch yaw\n"
         << "  parent_frame_id child_fram_id x y z qw qx qy qz";
      ROS_INFO_STREAM(ss.str());
      res.success = false;
      res.message = ss.str();
      return false;
    }

    geometry_msgs::TransformStamped msg{parse_frame(vstrings)};

    static_tf_broadcaster_.sendTransform(msg);

    res.success = true;
    return true;

  } catch (...) {
    std::stringstream ss;
    ss << "Unknown exception when adding a virtual frame";
    ROS_INFO_STREAM(ss.str());
    res.success = false;
    res.message = ss.str();
    return false;
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

geometry_msgs::TransformStamped ASN1BitstreamToTf::process_pose(const asn1SccTransformWithCovariance& asn1_pose)
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

geometry_msgs::TransformStamped ASN1BitstreamToTf::process_delta_pose(const asn1SccTransformWithCovariance& asn1_pose)
{
  throw "Received a delta pose but delta pose processing is not yet implemented";

  geometry_msgs::TransformStamped ros_msg;
  return std::move(ros_msg);

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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "asn1_bitstream_to_tf");

  ASN1BitstreamToTf asn1_bitstream_to_tf;

  ros::spin();

  return 0;
}