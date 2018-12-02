#include <map>
#include <memory>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_conversions/asn1_ros_conversions.hpp>
#include <infuse_asn1_types/Frame.h>

#include <infuse_debug_tools/ConnectTopic.h>

#include <sensor_msgs/Image.h>

#include "asn1_bitstream_transform_processer.hpp"


void fromASN1SCC(const asn1SccFramePair& image, sensor_msgs::Image& msg_image){

	//Not sure this is gonna work
	msg_image.header.stamp.fromNSec((uint64_t)image.left.metadata.timeStamp.microseconds * 1000ull);

	//not sure anything below is gonna work neither
	//fromASN1SCC(image.msgVersion, msg_image_left.header.frame_id);

	msg_image.width = image.left.data.cols+image.right.data.cols;
	msg_image.height = image.left.data.rows;

	msg_image.encoding = "mono8";

	msg_image.is_bigendian = true; //or false?

	msg_image.step = 8*msg_image.height;

	msg_image.data.resize(image.left.data.data.nCount+image.right.data.data.nCount);

	int i = 0;
	int line_idx = 0;
	while(i<msg_image.data.size()){
		for(int j = 0; j < image.left.data.cols; j++){
			msg_image.data[i] = image.left.data.data.arr[line_idx*image.left.data.cols+j];
			i++;
		}
		for(int j = 0; j < image.right.data.cols; j++){
			msg_image.data[i] = image.right.data.data.arr[line_idx*image.right.data.cols+j];
			i++;
		}
		line_idx++;
	}
}

namespace infuse_debug_tools
{

class ASN1BitstreamToImage : ASN1BitstreamTransformProcesser
{
public:
	ASN1BitstreamToImage();

	bool connect_image(infuse_debug_tools::ConnectTopic::Request  &req,
					   infuse_debug_tools::ConnectTopic::Response &res);

	void image_callback (const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub);

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::ServiceServer connect_image_srv_;
	std::map<std::string,ros::Subscriber> sub_map_;
	std::map<std::string,ros::Publisher> pub_map_;
	tf2_ros::TransformBroadcaster tf_broadcaster_;

	bool publish_poses_;

	std::unique_ptr<asn1SccFramePair> asn1_image_ptr_;
};


ASN1BitstreamToImage::ASN1BitstreamToImage() :
	private_nh_{"~"},
	connect_image_srv_{private_nh_.advertiseService("connect_image", &ASN1BitstreamToImage::connect_image, this)},
	publish_poses_{private_nh_.param<bool>("publish_poses", false)},
	asn1_image_ptr_{std::make_unique<asn1SccFramePair>()}
{
	private_nh_.param<bool>("publish_asn1_time", publish_asn1_time_, false);

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
					pub_map_[topic] = nh_.advertise<sensor_msgs::Image>(output_topic, 100);
					// Bind publisher to the callback
					boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback =
							boost::bind(&ASN1BitstreamToImage::image_callback, this, _1, boost::cref(pub_map_[topic]));
					// Create subscriber
					sub_map_[topic] = nh_.subscribe<sensor_msgs::Image>(topic, 100, callback);
					ROS_INFO_STREAM("Connected to topic " << topic << ". Publishing images on " << output_topic);
				} catch (...) {
					ROS_INFO_STREAM("ERROR: Could not connect to topic " << topic);
				}
			}
		}
	}
}

bool ASN1BitstreamToImage::connect_image(infuse_debug_tools::ConnectTopic::Request  &req,
										 infuse_debug_tools::ConnectTopic::Response &res)
{
	try {
		std::string output_topic = req.topic + "_ros";
		// Connect to topics
		pub_map_[req.topic] = nh_.advertise<sensor_msgs::Image>(output_topic, 100);
		// Bind publisher to the callback
		boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback =
				boost::bind(&ASN1BitstreamToImage::image_callback, this, _1, boost::cref(pub_map_[req.topic]));
		// Create subscriber
		sub_map_[req.topic] = nh_.subscribe<sensor_msgs::Image>(req.topic, 100, callback);
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

void ASN1BitstreamToImage::image_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub)
{
	// Get time at the begining of the callback -- unused now...
	auto cb_time = ros::Time::now();

	// Initialize
	asn1SccFramePair_Initialize(asn1_image_ptr_.get());

	// Decode
	flag res;
	int errorCode;
	BitStream bstream;
	BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
	res = asn1SccFramePair_Decode(asn1_image_ptr_.get(), &bstream, &errorCode);
	if (not res) {
		ROS_INFO("Error decoding asn1Image! Error: %d", errorCode);
		return;
	}

	// Convert to sensor_msgs/Image
	static sensor_msgs::Image msg_image;
	fromASN1SCC(*asn1_image_ptr_, msg_image);
	pub.publish(msg_image);
}


} // namespace infuse_debug_tools

int main(int argc, char **argv)
{
	ros::init(argc, argv, "asn1_bitstream_to_tf");

	infuse_debug_tools::ASN1BitstreamToImage asn1_bitstream_to_image;

	ros::spin();

	return 0;
}