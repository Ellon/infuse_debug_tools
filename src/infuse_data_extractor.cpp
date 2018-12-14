#include <vector>
#include <string>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/Pointcloud.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>

#include <boost/filesystem.hpp>
#include <boost/progress.hpp>

#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools
{

class PointCloudExtractor {
public:
	PointCloudExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &point_cloud_topic);
  void Extract();

private:
  void ProcessPointCloud(const infuse_msgs::asn1_bitstream::Ptr& msg);
  Eigen::Affine3d ConvertAsn1PoseToEigen(const asn1SccTransformWithCovariance& asn1_pose);

private:
  //! Directory where to put the dataset
  bfs::path output_dir_;
  //! Strings with paths to the bags to be processed
  std::vector<std::string> bag_paths_;
  //! Name of the topic with the point clouds
  std::string point_cloud_topic_;
  //! Directory where to put the pcd files (set on Extract())
  bfs::path data_dir_;
  //! Variable used to decode the ASN1 message into.
  std::unique_ptr<asn1SccPointcloud> asn1_pointcloud_ptr_;
  //! Counter for the number of pcd files written. Used to create the pcd filename
  size_t pcd_count_;
  //! Number of characters to be used when creating pcd filename
  unsigned int length_pcd_filename_;
  //! Stream used to write the metadata file
  std::ofstream metadata_ofs_;
};

PointCloudExtractor::PointCloudExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &point_cloud_topic)
  : output_dir_{output_dir},
    bag_paths_{bag_paths},
    point_cloud_topic_{point_cloud_topic},
    asn1_pointcloud_ptr_{std::make_unique<asn1SccPointcloud>()},
    pcd_count_{0},
    length_pcd_filename_{5}
{}


void PointCloudExtractor::Extract()
{
  // Makes sure the output dir does not already exists
  if (bfs::exists(output_dir_)) {
    std::stringstream ss;
    if (bfs::is_directory(output_dir_))
      ss << "A directory named \"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory to output the point clouds.";
    else if (bfs::is_regular_file(output_dir_))
      ss << "A regular file named \"" << output_dir_.string() << "\" already exists. Please remove this file or choose another directory name to output the point clouds.";
    else
      ss << "\"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory name to output the point clouds.";
    throw std::runtime_error(ss.str());
  }

  // Create output dir
  {
    bool dir_created = bfs::create_directory(output_dir_);
    if (not dir_created) {
      std::stringstream ss;
      ss << "Could not create \"" << output_dir_.string() << "\" directory.";
      throw std::runtime_error(ss.str());
    }
  }

  // Create data dir, used to put the pcd files
  {
    data_dir_ = output_dir_ / "data";
    bool dir_created = bfs::create_directory(data_dir_);
    if (not dir_created) {
      std::stringstream ss;
      ss << "Could not create \"" << data_dir_.string() << "\" directory.";
      throw std::runtime_error(ss.str());
    }
  }

  // Write dataformat file. The rationalle of keeping the dataformat separated
  // from the metadata is that this way it is possible to associate the cloud
  // number with the line in the metadata file.
  std::ofstream dataformat_ofs((output_dir_ / "dataformat.txt").string());
  {
    std::vector<std::string> entries{ASN1BitstreamLogger::GetPointcloudLogEntries()};
    unsigned int index = 1;
    for (auto entry : entries) {
      dataformat_ofs << "# " << std::setw(2) << index << " - " << entry << '\n';
      index++;
    }
  }
  dataformat_ofs.close();

  // Setup metadata file
  metadata_ofs_.open((output_dir_ / "metadata.txt").string());

  // Vector of topics used to create a view on the bag
  std::vector<std::string> topics = {point_cloud_topic_};

  // Get the number of clouds to process (only used to create the progress display)
  size_t n_point_clouds = 0;
  for (auto bag_path : bag_paths_) {
    rosbag::Bag bag(bag_path); // bagmode::Read by default
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    n_point_clouds += view.size();
    bag.close();
  }

  // Setup progress display
  std::cout << "Extracting " << n_point_clouds << " point clouds...";
  boost::progress_display show_progress( n_point_clouds );

  // Loop over bags
  for (auto bag_path : bag_paths_) {
    rosbag::Bag bag(bag_path); // bagmode::Read by default
    // Create a view of the bag with the selected topics only
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    try {
      // Loop over messages in each view
      for (rosbag::MessageInstance const m: view) {
        infuse_msgs::asn1_bitstream::Ptr i = m.instantiate<infuse_msgs::asn1_bitstream>();
        if (i != nullptr) {
          ProcessPointCloud(i);
          ++show_progress; // Update progress display
        } else throw std::runtime_error("Could not instantiate an infuse_msgs::asn1_bitstream message!");
      } // for msgs in view
    } catch (...) {
      // Assure the bags are closed if something goes wrong and re-trhow
      bag.close();
      throw;
    }

    bag.close();
  } // for bags

  metadata_ofs_.close();
}

void PointCloudExtractor::ProcessPointCloud(const infuse_msgs::asn1_bitstream::Ptr& msg)
{
  // Initialize asn1 point cloud to be sure we have clean object.
  asn1SccPointcloud_Initialize(asn1_pointcloud_ptr_.get());

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
  res = asn1SccPointcloud_Decode(asn1_pointcloud_ptr_.get(), &bstream, &errorCode);
  if (not res) {
    std::stringstream ss;
    ss << "Error decoding asn1Pointcloud! Error: " << errorCode << "\n";
    throw std::runtime_error(ss.str());
  }

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  fromASN1SCC(*asn1_pointcloud_ptr_, pcl_cloud);

  // Fill sensor pose information. This info ends up in the VIEWPOINT field of
  // the pcd file, and it's used to position the cloud on pcl_viewer with it
  // is used with multiple input pcds. Note that it is stored using float
  // values.
  Eigen::Affine3d T_fixed_robot = ConvertAsn1PoseToEigen(asn1_pointcloud_ptr_->metadata.pose_fixedFrame_robotFrame);
  Eigen::Affine3d T_robot_sensor = ConvertAsn1PoseToEigen(asn1_pointcloud_ptr_->metadata.pose_robotFrame_sensorFrame);
  Eigen::Affine3f T_fixed_sensor = (T_fixed_robot * T_robot_sensor).cast<float>();
  pcl_cloud.sensor_origin_ << T_fixed_sensor.translation(), 0.0;
  pcl_cloud.sensor_orientation_ = Eigen::Quaternionf(T_fixed_sensor.rotation());   

  // TODO: Test if the transformation is rigid. Something like the commented-
  // out code below could be used

  // const float epsilon = 0.001;
  // Eigen::Matrix3f R = T_fixed_sensor.rotation();
  // if(anyabs(1 - R.determinant()) > epsilon)
  //   NOT_RIGID;
  // else
  //   RIGID;

  // Compose pcd name
  std::string pcd_filename = std::to_string(pcd_count_);
  pcd_filename = std::string(length_pcd_filename_ - pcd_filename.length(), '0') + pcd_filename + ".pcd";
  bfs::path pcd_path = data_dir_ / pcd_filename;

  // Save pcd
  bool pcd_binary_mode = true;
  pcl::io::savePCDFile( pcd_path.string(), pcl_cloud, pcd_binary_mode );

  // Increment pcd counter
  pcd_count_++;

  // Log metadata
  ASN1BitstreamLogger::LogPointcloud(*asn1_pointcloud_ptr_, metadata_ofs_);
  metadata_ofs_ << '\n';

}

Eigen::Affine3d PointCloudExtractor::ConvertAsn1PoseToEigen(const asn1SccTransformWithCovariance& asn1_pose)
{
  Eigen::Matrix3d m; m = Eigen::Quaterniond(asn1_pose.data.orientation.arr[3],
                                            asn1_pose.data.orientation.arr[0],
                                            asn1_pose.data.orientation.arr[1],
                                            asn1_pose.data.orientation.arr[2]);
  Eigen::Affine3d eigen_pose;
  eigen_pose.linear() = m;
  eigen_pose.translation() << asn1_pose.data.translation.arr[0], asn1_pose.data.translation.arr[1], asn1_pose.data.translation.arr[2];
  return eigen_pose;
}

} // namespace infuse_debug_tools


#include <boost/program_options.hpp>

namespace bpo = boost::program_options;

int main(int argc, char **argv)
{
  try {
    // Parse program options
    bpo::options_description desc{"Options"};
    desc.add_options()
      ("help,h", "Display help")
      ("output-dir,o", bpo::value<std::string>(), "Directory where to put the extracted dataset")
      ("bags,b", bpo::value<std::vector<std::string>>()->multitoken()->composing(), "Bags composing the dataset")
      ("topic,t", bpo::value<std::string>()->default_value("/velodyne/point_cloud"), "Point cloud topic");

    bpo::positional_options_description pos_desc;
    pos_desc.add("output-dir", 1).add("bags", -1);

    bpo::command_line_parser parser{argc, argv};
    parser.options(desc).positional(pos_desc);

    bpo::variables_map vm;
    bpo::store(parser.run(), vm);
    bpo::notify(vm);

    if (vm.count("help")) {
      std::cout << "Usage:" << '\n';
      std::cout << "  " << argv[0] << " <output-dir> <bag1> ... <bagN>" << '\n';
      std::cout << desc << '\n';
      return 0;
    }

    if (not vm.count("output-dir") or not vm.count("bags")) {
      if (not vm.count("output-dir"))
        std::cout << "Error: Output directory is missing." << '\n';
      if (not vm.count("bags")) 
        std::cout << "Error: Bag files are missing." << '\n';

      std::cout << "Usage:" << '\n';
      std::cout << "  " << argv[0] << " <output-dir> <bag1> ... <bagN>" << '\n';
      std::cout << desc << '\n';
      return 1;
    }

    // Create the extractor and extract clouds.
    infuse_debug_tools::PointCloudExtractor cloud_extractor{
      vm["output-dir"].as<std::string>(),
      vm["bags"].as<std::vector<std::string>>(),
      vm["topic"].as<std::string>()};

    cloud_extractor.Extract();

  } catch (const bpo::error &ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  } catch (const std::runtime_error& ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  }

  return 0;
}