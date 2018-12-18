#include "PointCloudExtractor.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <pcl/io/pcd_io.h>

#include <boost/progress.hpp>

#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools {

PointCloudExtractor::PointCloudExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &point_cloud_topic, bool extract_pngs)
  : output_dir_{output_dir},
    bag_paths_{bag_paths},
    point_cloud_topic_{point_cloud_topic},
    asn1_pointcloud_ptr_{std::make_unique<asn1SccPointcloud>()},
    pcd_count_{0},
    length_pcd_filename_{5},
    extract_pngs_{extract_pngs},
    pcl_viewer_{nullptr},
    color_handler_{nullptr},
    point_size_{1}
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

  // Create png dir
  if (extract_pngs_) {
    png_dir_ = output_dir_ / "pngs";
    bool dir_created = bfs::create_directory(png_dir_);
    if (not dir_created) {
      std::stringstream ss;
      ss << "Could not create \"" << png_dir_.string() << "\" directory.";
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

  // Setup png extraction if needed
  if (extract_pngs_) {
    pcl_viewer_.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl_viewer_->setBackgroundColor (0, 0, 0);
    pcl_viewer_->initCameraParameters ();
  }

  // Setup progress display
  std::cout << "Extracting " << n_point_clouds << " point clouds" << (extract_pngs_? " and PNG views" : "") << "...";
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
      // Assure the bags are closed if something goes wrong and re-throw
      bag.close();
      if (extract_pngs_)  pcl_viewer_->close();
      throw;
    }

    bag.close();
  } // for bags

  metadata_ofs_.close();

  if (extract_pngs_) {
    pcl_viewer_->close();
  }

}

void PointCloudExtractor::ProcessPointCloud(const infuse_msgs::asn1_bitstream::Ptr& msg)
{
  // Guard against overflow on the filename numbers
  if (pcd_count_ >= std::pow(10, length_pcd_filename_))
    throw std::runtime_error("Overflow on the filename counter. Please increase the number of characters to be used to compose the filename");

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
  PointCloud::Ptr pcl_cloud_ptr(new PointCloud());
  fromASN1SCC(*asn1_pointcloud_ptr_, *pcl_cloud_ptr);

  // Fill sensor pose information. This info ends up in the VIEWPOINT field of
  // the pcd file, and it's used to position the cloud on pcl_viewer with it
  // is used with multiple input pcds. Note that it is stored using float
  // values.
  Eigen::Affine3d T_fixed_robot = ConvertAsn1PoseToEigen(asn1_pointcloud_ptr_->metadata.pose_fixedFrame_robotFrame);
  Eigen::Affine3d T_robot_sensor = ConvertAsn1PoseToEigen(asn1_pointcloud_ptr_->metadata.pose_robotFrame_sensorFrame);
  Eigen::Affine3f T_fixed_sensor = (T_fixed_robot * T_robot_sensor).cast<float>();
  pcl_cloud_ptr->sensor_origin_ << T_fixed_sensor.translation(), 0.0;
  pcl_cloud_ptr->sensor_orientation_ = Eigen::Quaternionf(T_fixed_sensor.rotation());   

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
  pcl::io::savePCDFile( pcd_path.string(), *pcl_cloud_ptr, pcd_binary_mode );

  // Handle PNG extraction
  if (extract_pngs_) {
    // Compute useful transformations
    // Sensor frame
    Eigen::Affine3f T_world_sensor;
    T_world_sensor = pcl_cloud_ptr->sensor_orientation_;
    T_world_sensor.translation() = pcl_cloud_ptr->sensor_origin_.head<3>();
    // Sensor frame that considers only yaw
    Eigen::Affine3f T_world_sensoryaw;
    T_world_sensoryaw = Eigen::AngleAxis<float>(ASN1BitstreamLogger::Yaw(pcl_cloud_ptr->sensor_orientation_), Eigen::Vector3f::UnitZ());
    T_world_sensoryaw.translation() = pcl_cloud_ptr->sensor_origin_.head<3>();
    // Camera position, behind the robot and considering only sensor yaw
    // (makes camera less shaky). Note that since the velodyne is mounted
    // facing backwards, we preform a positive translation on sensor's X axis
    Eigen::Affine3f T_world_camera = T_world_sensoryaw * Eigen::Translation<float,3>(45,0,20);

    // Add a new at current sensor pose. This creates a trail of frames
    pcl_viewer_->addCoordinateSystem (1.0, T_world_sensor, "sensor_frame");
    // Or alternativaly we can update the coordinate system, w/o leaving the trail
    // pcl_viewer_->updateCoordinateSystemPose ("sensor_frame", T_world_sensor);

    // Remove previous point cloud
    if(pcd_count_ != 0) {
      pcl_viewer_->removePointCloud("sample cloud");
    }

    // Add cloud
    color_handler_.reset (new pcl::visualization::PointCloudColorHandlerGenericField<Point> (pcl_cloud_ptr, "z"));
    pcl_viewer_->addPointCloud<Point> (pcl_cloud_ptr, *color_handler_, "sample cloud");
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "sample cloud");

    // Put the camera behind the robot, looking at the sensor origin, and upwards
    pcl_viewer_->setCameraPosition (T_world_camera.translation()[0], // pos_x
                                    T_world_camera.translation()[1], // pos_y
                                    T_world_camera.translation()[2], // pos_z
                                    T_world_sensoryaw.translation()[0], // view_x
                                    T_world_sensoryaw.translation()[1], // view_y
                                    T_world_sensoryaw.translation()[2], // view_z
                                    0,  // up_x
                                    0,  // up_y
                                    1); // up_z

    // Render
    bool force_redraw = true;
    pcl_viewer_->spinOnce (1, force_redraw);

    // Save png
    bfs::path png_path = png_dir_ / bfs::path(pcd_path).filename().replace_extension(".png");
    pcl_viewer_->saveScreenshot(png_path.string());
  }

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

} // infuse_debug_tools
