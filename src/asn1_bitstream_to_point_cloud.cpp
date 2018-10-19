#include <map>
#include <memory>

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/Pointcloud.h>
// #include <infuse_asn1_types/TransformWithCovariance.h>
// #include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <infuse_debug_tools/ConnectTopic.h>
// #include <infuse_debug_tools/AddVirtualFrame.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class ASN1BitstreamToPointCloud
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

public:
  ASN1BitstreamToPointCloud();

  bool connect_point_cloud(infuse_debug_tools::ConnectTopic::Request  &req,
                           infuse_debug_tools::ConnectTopic::Response &res);

  void point_cloud_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub);

private:
  geometry_msgs::TransformStamped process_point_cloud(const asn1SccTransformWithCovariance& asn1Transform);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer connect_point_cloud_srv_;
  std::map<std::string,ros::Subscriber> sub_map_;
  std::map<std::string,ros::Publisher> pub_map_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  bool publish_asn1_time_;
  std::unique_ptr<asn1SccPointcloud> asn1_pointcloud_ptr_;
};


ASN1BitstreamToPointCloud::ASN1BitstreamToPointCloud() :
  private_nh_{"~"},
  connect_point_cloud_srv_{private_nh_.advertiseService("connect_point_cloud", &ASN1BitstreamToPointCloud::connect_point_cloud, this)},
  publish_asn1_time_{private_nh_.param<bool>("publish_asn1_time", false)},
  asn1_pointcloud_ptr_{std::make_unique<asn1SccPointcloud>()}
{
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
          std::string output_topic = topic + "_ros";
          // Create publisher
          pub_map_[topic] = nh_.advertise<PointCloud>(output_topic, 100);
          // Bind publisher to the callback
          boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback = 
            boost::bind(&ASN1BitstreamToPointCloud::point_cloud_callback, this, _1, boost::cref(pub_map_[topic]));
          // Create subscriber
          sub_map_[topic] = nh_.subscribe<PointCloud>(topic, 100, callback);
          ROS_INFO_STREAM("Connected to topic " << topic << ". Publishing clouds on " << output_topic);
        } catch (...) {
          ROS_INFO_STREAM("ERROR: Could not connect to topic " << topic);
        }
      }
    }
  }
}

bool ASN1BitstreamToPointCloud::connect_point_cloud(infuse_debug_tools::ConnectTopic::Request  &req,
                                                    infuse_debug_tools::ConnectTopic::Response &res)
{
  try {
    std::string output_topic = req.topic + "_ros";
    // Connect to topics
    pub_map_[req.topic] = nh_.advertise<PointCloud>(output_topic, 100);
    // Bind publisher to the callback
    boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback = 
      boost::bind(&ASN1BitstreamToPointCloud::point_cloud_callback, this, _1, boost::cref(pub_map_[req.topic]));
    // Create subscriber
    sub_map_[req.topic] = nh_.subscribe<PointCloud>(req.topic, 100, callback);
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

void ASN1BitstreamToPointCloud::point_cloud_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub)
{
  // Get time at the begining of the callback
  auto cb_time = ros::Time::now();

  // Initialize
  asn1SccPointcloud_Initialize(asn1_pointcloud_ptr_.get());

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
  res = asn1SccPointcloud_Decode(asn1_pointcloud_ptr_.get(), &bstream, &errorCode);
  if (not res) {
    ROS_INFO("Error decoding asn1Pointcloud! Error: %d", errorCode);
    return;
  }

  // Convert to PCL
  static PointCloud msg_cloud;
  fromASN1SCC(*asn1_pointcloud_ptr_, msg_cloud);

  // Publish cloud
  pcl_conversions::toPCL(cb_time, msg_cloud.header.stamp);
  pub.publish(msg_cloud.makeShared());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "asn1_bitstream_to_tf");

  ASN1BitstreamToPointCloud asn1_bitstream_to_point_cloud;

  ros::spin();

  return 0;
}