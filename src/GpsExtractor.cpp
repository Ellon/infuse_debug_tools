#include "GpsExtractor.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>

#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools {

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    this->signalMessage(msg);
  }
};


GpsExtractor::GpsExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &pose_topic,  const std::string &info_topic)
: output_dir_{output_dir},
  bag_paths_{bag_paths},
  pose_topic_{pose_topic},
  info_topic_{info_topic},
  queue_size_{100000}
{
  // This is needed to use message_filters::TimeSynchronizer out of a ROS node
  ros::Time::init();
}

void GpsExtractor::Extract()
{
  // Count all messages on pose and info topics
  size_t n_poses = 0, n_info = 0;
  for (auto bag_path : bag_paths_) {
    rosbag::Bag bag(bag_path);
    rosbag::View view_poses(bag, rosbag::TopicQuery({pose_topic_}));
    rosbag::View view_infos(bag, rosbag::TopicQuery({info_topic_}));
    n_poses += view_poses.size();
    n_info += view_infos.size();
    bag.close();
  }

  // Stop here if there's nothing on the topic
  if (n_poses == 0) {
    std::cout << "Warning: Nothing to extract on topic " << pose_topic_ << std::endl;
    return;
  }

  // Both topics are supposed to have the same number of messages (inless
  // rosbag was started/stopped in-between the publish of both messages, what
  // is very unlikely)
  if(n_poses != n_info)
    std::cout << "WARNING: Number of GPS poses differs from number of GPS info (" << n_poses << " poses vs. " << n_info << " infos)" << std::endl;

  // Makes sure the output dir does not already exists
  if (bfs::exists(output_dir_)) {
    std::stringstream ss;
    if (bfs::is_directory(output_dir_))
      ss << "A directory named \"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory to output GPS data.";
    else if (bfs::is_regular_file(output_dir_))
      ss << "A regular file named \"" << output_dir_.string() << "\" already exists. Please remove this file or choose another directory name to output GPS data.";
    else
      ss << "\"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory name to output GPS data.";
    throw std::runtime_error(ss.str());
  }

  // Lambda function that creates a directory (or subdir inside dir if specified).
  auto lambda_create_subdir = [](bfs::path dir, std::string subdir = "") -> bfs::path {
    bfs::path dirpath = dir / subdir;
    bool dir_created = bfs::create_directory(dirpath);
    if (not dir_created) {
      std::stringstream ss;
      ss << "Could not create \"" << dirpath.string() << "\" directory.";
      throw std::runtime_error(ss.str());
    }
    return dirpath;
  };
  // Create output dir
  lambda_create_subdir(output_dir_);

  // Write dataformat file. The rationalle of keeping the dataformat separated
  // from the metadata is that this way it is possible to associate the cloud
  // number with the line in the metadata file.
  // We have three outputs files, so we write three datafiles.
  auto write_dataformat_file = [this](const std::string &filename, const std::vector<std::string> & entries) -> void {
    std::ofstream dataformat_ofs((this->output_dir_ / filename).string());
    unsigned int index = 1;
    for (auto entry : entries) {
      dataformat_ofs << "# " << std::setw(2) << index << " - " << entry << '\n';
      index++;
    }
    dataformat_ofs.close();
  };
  write_dataformat_file("gps_pose_dataformat.txt", ASN1BitstreamLogger::GetTransformWithCovarianceLogEntries());
  write_dataformat_file("gps_info_dataformat.txt", ASN1BitstreamLogger::GetGpsInfoLogEntries());
  write_dataformat_file("gps_pose_info_dataformat.txt", ASN1BitstreamLogger::GetGpsPoseWithInfoLogEntries());

  // Set up fake subscribers to capture synchronized GPS data
  BagSubscriber<infuse_msgs::asn1_bitstream> pose_sub;
  BagSubscriber<infuse_novatel_gps_msgs::UtmInfo> info_sub;

  // Use time synchronizer to make sure we get properly synchronized data
  message_filters::TimeSynchronizer<infuse_msgs::asn1_bitstream, infuse_novatel_gps_msgs::UtmInfo> sync(pose_sub, info_sub, queue_size_);
  sync.registerCallback(boost::bind(&GpsExtractor::ProcessPoseWithInfo, this, _1, _2));

  // Setup output data files
  pose_ofs_.open((output_dir_ / "gps_pose.txt").string());
  info_ofs_.open((output_dir_ / "gps_info.txt").string());
  syncd_ofs_.open((output_dir_ / "gps_pose_info.txt").string());

  // Setup progress display
  std::cout << "Extracting " << n_poses << " GPS entries to " << output_dir_.string();
  boost::progress_display show_progress(n_poses);

  // Vector used to create a view on the bag with both topics
  std::vector<std::string> topics = {pose_topic_, info_topic_};

  // Reset count before main loop
  syncd_count_ = 0;

  // Loop over bags
  for (auto bag_path : bag_paths_) {
    rosbag::Bag bag(bag_path); // bagmode::Read by default
    // Create a view of the bag with the selected topics only
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    try {
      // Loop over messages in each view
      for (rosbag::MessageInstance const m: view) {
        if (m.getTopic() == pose_topic_) {
          infuse_msgs::asn1_bitstream::Ptr pose_msg = m.instantiate<infuse_msgs::asn1_bitstream>();
          if (pose_msg != nullptr) {
            ProcessPose(pose_msg);
            pose_sub.newMessage(pose_msg);
            // Update progress display
            ++show_progress;
          } else
            throw std::runtime_error("Could not instantiate an infuse_msgs::asn1_bitstream message!");
        }

        if (m.getTopic() == info_topic_) {
          infuse_novatel_gps_msgs::UtmInfo::Ptr info_msg = m.instantiate<infuse_novatel_gps_msgs::UtmInfo>();
          if (info_msg != nullptr) {
            ProcessInfo(info_msg);
            info_sub.newMessage(info_msg);
          } else
            throw std::runtime_error("Could not instantiate an infuse_novatel_gps_msgs::UtmInfo message!");
        }
      } // for msgs in view
    } catch (...) {
      // Assure files are closed if something goes wrong and re-trhow
      bag.close();
      pose_ofs_.close();
      info_ofs_.close();
      syncd_ofs_.close();
      throw;
    }

    bag.close();
  } // for bags

  if(syncd_count_ != n_poses)
    std::cout << "WARNING: Number of synchronized data differs from number of poses (" << syncd_count_ << " processed data vs. " << n_poses << " poses)" << std::endl;

  pose_ofs_.close();
  info_ofs_.close();
  syncd_ofs_.close();
}

void GpsExtractor::ProcessPose(const infuse_msgs::asn1_bitstream::ConstPtr &pose_msg)
{
  // Initialize asn1 pose to be sure we have a clean object.
  asn1SccTransformWithCovariance asn1_pose;
  asn1SccTransformWithCovariance_Initialize(&asn1_pose);

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, const_cast<unsigned char*>(pose_msg->data.data()), pose_msg->data.size());
  res = asn1SccTransformWithCovariance_Decode(&asn1_pose, &bstream, &errorCode);
  if (not res) {
    std::stringstream ss;
    ss << "Error decode asn1TransformWithCovariance! Error: " << errorCode << "\n";
    throw std::runtime_error(ss.str());
  }

  ASN1BitstreamLogger::LogTransformWithCovariance(asn1_pose, pose_ofs_);
  pose_ofs_ << '\n';

}

void GpsExtractor::ProcessInfo(const infuse_novatel_gps_msgs::UtmInfo::ConstPtr &info_msg)
{
  ASN1BitstreamLogger::LogGpsInfo(*info_msg, info_ofs_);
  info_ofs_ << '\n';
}

void GpsExtractor::ProcessPoseWithInfo(const infuse_msgs::asn1_bitstream::ConstPtr &pose_msg, const infuse_novatel_gps_msgs::UtmInfo::ConstPtr &info_msg)
{
  // Initialize asn1 pose to be sure we have a clean object.
  asn1SccTransformWithCovariance asn1_pose;
  asn1SccTransformWithCovariance_Initialize(&asn1_pose);

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, const_cast<unsigned char*>(pose_msg->data.data()), pose_msg->data.size());
  res = asn1SccTransformWithCovariance_Decode(&asn1_pose, &bstream, &errorCode);
  if (not res) {
    std::stringstream ss;
    ss << "Error decode asn1TransformWithCovariance! Error: " << errorCode << "\n";
    throw std::runtime_error(ss.str());
  }

  ASN1BitstreamLogger::LogTransformWithCovariance(asn1_pose, syncd_ofs_);
  ASN1BitstreamLogger::LogGpsInfo(*info_msg, syncd_ofs_);
  syncd_ofs_ << '\n';

  syncd_count_++;
}

}
