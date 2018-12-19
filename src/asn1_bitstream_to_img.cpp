#include <map>
#include <memory>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_conversions/asn1_ros_conversions.hpp>
#include <infuse_asn1_types/Frame.h>

#include <infuse_debug_tools/ConnectTopic.h>

#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include "asn1_bitstream_transform_processer.hpp"


void fromASN1SCC(const asn1SccFramePair& image, sensor_msgs::Image& msg_image_left, sensor_msgs::Image& msg_image_right){

	//Not sure this is gonna work
	msg_image_left.header.stamp.fromNSec((uint64_t)image.left.metadata.timeStamp.microseconds * 1000ull);

	//not sure anything below is gonna work neither
	//fromASN1SCC(image.msgVersion, msg_image_left.header.frame_id);
	std::cout<<"LEFT MATRIX";
	for(int i = 0; i<3; i++){
		for(int j = 0; j<3; j++){
			std::cout<<image.left.intrinsic.cameraMatrix.arr[i].arr[j]<< " ";
		}
	}
	std::cout<<std::endl;
	std::cout<<"RIGHT MATRIX\n";
	for(int i = 0; i<3; i++){
		for(int j = 0; j<3; j++){
			std::cout<<image.right.intrinsic.cameraMatrix.arr[i].arr[j]<< " ";
		}
		std::cout<<std::endl;
	}
	std::cout<<"BASELINE\n";
	std::cout<<image.baseline<<std::endl;
	msg_image_left.width = image.left.data.cols;
	msg_image_left.height = image.left.data.rows;

	msg_image_left.encoding = "mono8";

	msg_image_left.is_bigendian = true; //or false?

	msg_image_left.step = msg_image_left.width;

	msg_image_left.data.resize(image.left.data.data.nCount);

//	int line_idx = 0;
//	while(i<msg_image_left.data.size()){
//		for(int j = 0; j < image.left.data.cols; j++){
//			msg_image_left.data[i] = image.left.data.data.arr[line_idx*image.left.data.cols+j];
//			i++;
//		}
//		for(int j = 0; j < image.right.data.cols; j++){
//			msg_image_left.data[i] = image.right.data.data.arr[line_idx*image.right.data.cols+j];
//			i++;
//		}
//		line_idx++;
//	}

	int i = 0;
	while(i<msg_image_left.data.size()){
		msg_image_left.data[i] = image.left.data.data.arr[i];
		i++;
	}

	msg_image_right.header.stamp.fromNSec((uint64_t)image.right.metadata.timeStamp.microseconds * 1000ull);

	//not sure anything below is gonna work neither
	//fromASN1SCC(image.msgVersion, msg_image_left.header.frame_id);

	msg_image_right.width = image.right.data.cols;
	msg_image_right.height = image.right.data.rows;

	msg_image_right.encoding = "mono8";

	msg_image_right.is_bigendian = true; //or false?

	msg_image_right.step = msg_image_right.width;

	msg_image_right.data.resize(image.right.data.data.nCount);

	i = 0;
	while(i<msg_image_right.data.size()){
		msg_image_right.data[i] = image.right.data.data.arr[i];
		i++;
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

	void image_callback (const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub_left, const ros::Publisher &pub_right);

private:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::ServiceServer connect_image_srv_;
	std::map<std::string,ros::Subscriber> sub_map_;
	std::map<std::string,std::pair<ros::Publisher, ros::Publisher>> pub_map_;
	tf2_ros::TransformBroadcaster tf_broadcaster_;

	bool publish_poses_;

	std::unique_ptr<asn1SccFramePair> asn1_image_ptr_;
	void bind_subToPubs(std::string topic_in);
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
					bind_subToPubs(topic);
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
		bind_subToPubs(req.topic);
//		std::string output_topic = req.topic + "_ros";
//		// Connect to topics
//		pub_map_[req.topic] = nh_.advertise<sensor_msgs::Image>(output_topic, 100);
//		// Bind publisher to the callback
//		boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback =
//				boost::bind(&ASN1BitstreamToImage::image_callback, this, _1, boost::cref(pub_map_[req.topic]));
//		// Create subscriber
//		sub_map_[req.topic] = nh_.subscribe<sensor_msgs::Image>(req.topic, 100, callback);
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

void ASN1BitstreamToImage::bind_subToPubs(std::string topic_in){

	std::string output_topic_left = topic_in + "_rosLeft";
	std::string output_topic_right= topic_in + "_rosRight";
	// Create publisher
	pub_map_[topic_in] = std::make_pair(nh_.advertise<sensor_msgs::Image>(output_topic_left, 100), nh_.advertise<sensor_msgs::Image>(output_topic_right, 100));
	// Bind publisher to the callback
	boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback =
			boost::bind(&ASN1BitstreamToImage::image_callback, this, _1, boost::cref(pub_map_[topic_in].first), boost::cref(pub_map_[topic_in].second));
	// Create subscriber
	sub_map_[topic_in] = nh_.subscribe<sensor_msgs::Image>(topic_in, 100, callback);
	ROS_INFO_STREAM("Connected to topic " << topic_in << ". Publishing images on " << output_topic_left<<" and to "<<output_topic_right);

}

void ASN1BitstreamToImage::image_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub_left, const ros::Publisher &pub_right )
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
	static sensor_msgs::Image msg_image_left;
	static sensor_msgs::Image msg_image_right;
	fromASN1SCC(*asn1_image_ptr_, msg_image_left, msg_image_right);
	pub_left.publish(msg_image_left);
	pub_right.publish(msg_image_right);
}


} // namespace infuse_debug_tools

int main(int argc, char **argv)
{
	ros::init(argc, argv, "asn1_bitstream_to_tf");

	infuse_debug_tools::ASN1BitstreamToImage asn1_bitstream_to_image;

	ros::spin();

	return 0;
}
