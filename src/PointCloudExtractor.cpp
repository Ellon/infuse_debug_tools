#include "PointCloudExtractor.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <boost/progress.hpp>

#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools {

PointCloudExtractor::PointCloudExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &point_cloud_topic, bool extract_pngs, ColorMode color_mode)
  : output_dir_{output_dir},
    bag_paths_{bag_paths},
    point_cloud_topic_{point_cloud_topic},
    asn1_pointcloud_ptr_{std::make_unique<asn1SccPointcloud>()},
    length_pcd_filename_{5},
    pcd_count_{0},
    pcd_max_{std::stoul(std::string("1") + std::string(length_pcd_filename_, '0')) - 1},
    extract_pngs_{extract_pngs},
    pcl_viewer_{nullptr},
    point_size_{1},
    compute_min_max_z_{true},
    min_z_{0},
    max_z_{0},
    color_mode_{color_mode}
{}

PointCloudExtractor::PointCloudExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &point_cloud_topic, double min_z, double max_z, bool extract_pngs, ColorMode color_mode)
  : output_dir_{output_dir},
    bag_paths_{bag_paths},
    point_cloud_topic_{point_cloud_topic},
    asn1_pointcloud_ptr_{std::make_unique<asn1SccPointcloud>()},
    length_pcd_filename_{5},
    pcd_count_{0},
    pcd_max_{std::stoul(std::string("1") + std::string(length_pcd_filename_, '0')) - 1},
    extract_pngs_{extract_pngs},
    pcl_viewer_{nullptr},
    point_size_{1},
    compute_min_max_z_{false},
    min_z_{min_z},
    max_z_{max_z},
    color_mode_{color_mode}
{}


void PointCloudExtractor::Extract()
{
  // Vector of topics used to create a view on the bag
  std::vector<std::string> topics = {point_cloud_topic_};

  // Get the number of clouds to process
  size_t n_point_clouds = 0;
  for (auto bag_path : bag_paths_) {
    rosbag::Bag bag(bag_path); // bagmode::Read by default
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    n_point_clouds += view.size();
    bag.close();
  }

  // Stop here if there's nothing on the topic
  if (n_point_clouds == 0) {
    std::cout << "Warning: Nothing to extract on topic " << point_cloud_topic_ << std::endl;
    return;
  }

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
  // Create subdirs
  data_dir_     = lambda_create_subdir(output_dir_, "data");
  metadata_dir_ = lambda_create_subdir(output_dir_, "metadata");
  if (extract_pngs_)
    png_dir_      = lambda_create_subdir(output_dir_, "pngs");

  // Write dataformat file. The rationalle of keeping the dataformat separated
  // from the metadata is that this way it is possible to associate the cloud
  // number with the line in the metadata file.
  std::ofstream dataformat_ofs((output_dir_ / "dataformat.txt").string());
  {
    std::vector<std::string> entries{ASN1BitstreamLogger::GetPointcloudLogEntries()};
    // Add coordinates min max to the entries
    entries.insert(entries.end(), {"min_x", "max_x", "min_y", "max_y", "min_z", "max_z"});
    unsigned int index = 1;
    for (auto entry : entries) {
      dataformat_ofs << "# " << std::setw(2) << index << " - " << entry << '\n';
      index++;
    }
  }
  dataformat_ofs.close();

  // Setup metadata file
  metadata_ofs_.open((output_dir_ / "all_metadata.txt").string());

  // // Find min and max Z (that will be used to create a color lookup table) if needed
  if (extract_pngs_ and compute_min_max_z_) {
    std::cout << "Computing min and max Z for all clouds... ";
    boost::progress_display show_progress( n_point_clouds );
    // Loop over bags
    bool first_cloud = true;
    for (auto bag_path : bag_paths_) {
      rosbag::Bag bag(bag_path); // bagmode::Read by default
      // Create a view of the bag with the selected topics only
      rosbag::View view(bag, rosbag::TopicQuery(topics));

      try {
        for (rosbag::MessageInstance const m: view) {
          infuse_msgs::asn1_bitstream::Ptr i = m.instantiate<infuse_msgs::asn1_bitstream>();
          if (i != nullptr) {
            if (first_cloud) {
              std::tie(std::ignore, std::ignore, std::ignore, std::ignore, min_z_, max_z_) = FindMinMax(i);
              first_cloud = false;
            }
            else {
              float min = min_z_, max = max_z_;
              std::tie(std::ignore, std::ignore, std::ignore, std::ignore, min, max) = FindMinMax(i);
              if (min < min_z_)
                min_z_ = min;
              if (max > max_z_)
                max_z_ = max;
            }
            ++show_progress; // Update progress display
          } else throw std::runtime_error("Could not instantiate an infuse_msgs::asn1_bitstream message!");
        } // for msgs in view
      } catch (...) {
        bag.close();
        metadata_ofs_.close();
        throw;
      }
    }

    std::cout << "Found : min_z = " << min_z_ << ", max_z = " << max_z_ << std::endl;
  }

  // Create the pcl viewer instance if needed
  if (extract_pngs_) {
    pcl_viewer_.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl_viewer_->setBackgroundColor (0, 0, 0);
    pcl_viewer_->initCameraParameters ();
  }

  // Setup progress display
  std::cout << "Extracting " << n_point_clouds << " point clouds " << (extract_pngs_? "and PNG views " : "") << "to " << output_dir_.string() << "/...";
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
      // Assure files are closed if something goes wrong and re-throw
      bag.close();
      metadata_ofs_.close();
      if (extract_pngs_) {
        pcl_viewer_->close();
      }
      throw;
    }

    bag.close();
  } // for bags

  metadata_ofs_.close();

  if (extract_pngs_) {
    pcl_viewer_->close();
  }

}

void PointCloudExtractor::DecodeBitstream(const infuse_msgs::asn1_bitstream::Ptr& msg)
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
}

Eigen::Affine3f PointCloudExtractor::ComputeSensorPoseInFixedFrame(const asn1SccPointcloud & asn1_cloud)
{
  Eigen::Affine3d T_fixed_robot = ConvertAsn1PoseToEigen(asn1_cloud.metadata.pose_fixedFrame_robotFrame);
  Eigen::Affine3d T_robot_sensor = ConvertAsn1PoseToEigen(asn1_cloud.metadata.pose_robotFrame_sensorFrame);
  return (T_fixed_robot * T_robot_sensor).cast<float>();
}

void PointCloudExtractor::ProcessPointCloud(const infuse_msgs::asn1_bitstream::Ptr& msg)
{
  // Guard against overflow on the filename numbers
  if (pcd_count_ > pcd_max_)
    throw std::runtime_error("Overflow on the pcd filename counter. Please increase the number of characters to be used to compose the filename");

  // Decode bitstream
  DecodeBitstream(msg);

  // Convert to PCL
  PointCloud::Ptr pcl_cloud_ptr(new PointCloud());
  fromASN1SCC(*asn1_pointcloud_ptr_, *pcl_cloud_ptr);

  // Extract sensor frame in fixed frame from metadata
  Eigen::Affine3f T_fixed_sensor = ComputeSensorPoseInFixedFrame(*asn1_pointcloud_ptr_);
  // Fill sensor pose information. This info ends up in the VIEWPOINT field of
  // the pcd file, and it's used to position the cloud on pcl_viewer with it
  // is used with multiple input pcds. Note that it is stored using float
  // values.
  SetCloudSensorPose(T_fixed_sensor, *pcl_cloud_ptr);

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

  // Put the cloud in the fixed frame, so min and max will be absolute. Note
  // that this is done AFTER saving the .pcd file, so it has the point cloud
  // as it was stored in the message.
  pcl::transformPointCloud(*pcl_cloud_ptr, *pcl_cloud_ptr, T_fixed_sensor);
  SetCloudSensorPose(Eigen::Affine3f::Identity(), *pcl_cloud_ptr);

  // Log metadata in the common file
  ASN1BitstreamLogger::LogPointcloud(*asn1_pointcloud_ptr_, metadata_ofs_);
  // Log min-max coordinates in the metadata as well
  float min_x = 0, max_x = 0, min_y = 0, max_y = 0, min_z = 0, max_z = 0;
  try {
    std::tie(min_x, max_x, min_y, max_y, min_z, max_z) = FindMinMax(*pcl_cloud_ptr);
  } catch (std::out_of_range) {
    std::cerr << "Warning: Found empty cloud (number " << pcd_count_ << ") when computing min max coordinates\n";
  }
  metadata_ofs_ << min_x << " " << max_x << " " << min_y << " " << max_y << " " << min_z << " " << max_z;
  metadata_ofs_ << '\n';

  // Log the same metadata of this cloud in a separated file
  bfs::path metadata_path = metadata_dir_ / bfs::path(pcd_path).filename().replace_extension(".txt");
  std::ofstream pcd_metadata_ofs(metadata_path.string());
  ASN1BitstreamLogger::LogPointcloud(*asn1_pointcloud_ptr_, pcd_metadata_ofs);
  pcd_metadata_ofs << min_x << " " << max_x << " " << min_y << " " << max_y << " " << min_z << " " << max_z;
  pcd_metadata_ofs.close();


  // Handle PNG extraction
  if (extract_pngs_) {
    // Get a version of the point cloud that can be colored
    ColoredPointCloud::Ptr pcl_colored_cloud_ptr(new ColoredPointCloud());
    pcl::copyPointCloud(*pcl_cloud_ptr, *pcl_colored_cloud_ptr);

    // Compute useful transformations
    // Sensor frame that considers only yaw
    Eigen::Affine3f T_fixed_sensoryaw;
    T_fixed_sensoryaw = Eigen::AngleAxis<float>(ASN1BitstreamLogger::Yaw(Eigen::Quaternionf(T_fixed_sensor.rotation())), Eigen::Vector3f::UnitZ());
    T_fixed_sensoryaw.translation() = T_fixed_sensor.translation();
    // Camera pose, behind the robot and considering only sensor yaw
    // (makes camera less shaky). Note that since the velodyne is mounted
    // facing backwards, we preform a positive translation on sensor's X axis
    Eigen::Affine3f T_fixed_camera = T_fixed_sensoryaw * Eigen::Translation<float,3>(45,0,20);

    // Add a new at current sensor pose. This creates a trail of frames
    pcl_viewer_->addCoordinateSystem (1.0, T_fixed_sensor, "sensor_frame");
    // Or alternativaly we can update the coordinate system, w/o leaving the trail
    // pcl_viewer_->updateCoordinateSystemPose ("sensor_frame", T_fixed_sensor);

    // Fill RGB values for each point
    ColorPointCloud(*pcl_colored_cloud_ptr);

    // Remove previous point cloud
    if(pcd_count_ != 0) {
      pcl_viewer_->removePointCloud("sample cloud");
    }

    // Add cloud
    pcl_viewer_->addPointCloud (pcl_colored_cloud_ptr, "sample cloud");
    pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "sample cloud");

    // Put the camera behind the robot, looking at the sensor origin, and upwards
    pcl_viewer_->setCameraPosition (T_fixed_camera.translation()[0], // pos_x
                                    T_fixed_camera.translation()[1], // pos_y
                                    T_fixed_camera.translation()[2], // pos_z
                                    T_fixed_sensoryaw.translation()[0], // view_x
                                    T_fixed_sensoryaw.translation()[1], // view_y
                                    T_fixed_sensoryaw.translation()[2], // view_z
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

std::tuple<float,float,float,float,float,float> PointCloudExtractor::FindMinMax(const PointCloud & cloud)
{
  float min_x = cloud.points.at(0).x,
        max_x = cloud.points.at(0).x,
        min_y = cloud.points.at(0).y,
        max_y = cloud.points.at(0).y,
        min_z = cloud.points.at(0).z,
        max_z = cloud.points.at(0).z;

  for (auto & point : cloud.points)
  {
    if (min_x > point.x)
      min_x = point.x;

    if (max_x < point.x)
      max_x = point.x;

    if (min_y > point.y)
      min_y = point.y;

    if (max_y < point.y)
      max_y = point.y;

    if (min_z > point.z)
      min_z = point.z;

    if (max_z < point.z)
      max_z = point.z;
  }

  return std::make_tuple(min_x, max_x, min_y, max_y, min_z, max_z);
}

std::tuple<float,float,float,float,float,float> PointCloudExtractor::FindMinMax(const infuse_msgs::asn1_bitstream::Ptr& msg)
{
  // Decode bitstream
  DecodeBitstream(msg);

  // Convert to PCL
  PointCloud::Ptr pcl_cloud_ptr(new PointCloud());
  fromASN1SCC(*asn1_pointcloud_ptr_, *pcl_cloud_ptr);

  // Put the cloud in the world frame, so min and max will be absolute
  Eigen::Affine3f T_fixed_sensor = ComputeSensorPoseInFixedFrame(*asn1_pointcloud_ptr_);
  pcl::transformPointCloud(*pcl_cloud_ptr, *pcl_cloud_ptr, T_fixed_sensor);

  return FindMinMax(*pcl_cloud_ptr);
}

void PointCloudExtractor::ColorPointCloud(ColoredPointCloud & colored_cloud)
{
  double lut_scale = 255.0 / (max_z_ - min_z_);  // max is 255, min is 0

  if (min_z_ == max_z_)  // In case the cloud is flat
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (auto & point : colored_cloud.points)
  {
    int value = boost::math::iround ( (point.z - min_z_) * lut_scale); // Round the number to the closest integer

    // Guard against outliers
    if (value > 255)
      value = 255;
    if (value < 0)
      value = 0;

    // Color the point
    switch (color_mode_)
    {
      case ColorMode::kBlueToRed:
        // Blue (= min) -> Red (= max)
        point.r = value;
        point.g = 0;
        point.b = 255 - value;
        break;
      case ColorMode::kGreenToMagenta:
        // Green (= min) -> Magenta (= max)
        point.r = value;
        point.g = 255 - value;
        point.b = value;
        break;
      case ColorMode::kWhiteToRed:
        // White (= min) -> Red (= max)
        point.r = 255;
        point.g = 255 - value;
        point.b = 255 - value;
        break;
      case ColorMode::kGrayOrRed:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          point.r = 255;
          point.g = 0;
          point.b = 0;
        }
        else
        {
          point.r = 128;
          point.g = 128;
          point.b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        point.r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        point.g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        point.b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    } 
  }
}

std::istream& operator>>(std::istream& in, PointCloudExtractor::ColorMode& color_mode)
{
  using ColorMode = PointCloudExtractor::ColorMode;

  std::string token;
  in >> token;
  if (token == "blue2red")
    color_mode = ColorMode::kBlueToRed;
  else if (token == "green2magenta")
    color_mode = ColorMode::kGreenToMagenta;
  else if (token == "white2red")
    color_mode = ColorMode::kWhiteToRed;
  else if (token == "gray/red")
    color_mode = ColorMode::kGrayOrRed;
  else if (token == "rainbow")
    color_mode = ColorMode::kRainbow;
  else
      in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream& operator<<(std::ostream& out, const PointCloudExtractor::ColorMode& color_mode)
{
  using ColorMode = PointCloudExtractor::ColorMode;

  if (color_mode == ColorMode::kBlueToRed)
    out << "blue2red";
  else if (color_mode == ColorMode::kGreenToMagenta)
    out << "green2magenta";
  else if (color_mode == ColorMode::kWhiteToRed)
    out << "white2red";
  else if (color_mode == ColorMode::kGrayOrRed)
    out << "gray/red";
  else if (color_mode == ColorMode::kRainbow)
    out << "rainbow";
  else
    out.setstate(std::ios_base::failbit);

  return out;
}

} // infuse_debug_tools
