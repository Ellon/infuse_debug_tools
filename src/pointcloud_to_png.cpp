#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "asn1_bitstream_logger.hpp"

namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;
namespace idt = infuse_debug_tools;

int main(int argc, char **argv)
{
  try {
    // Parse program options
    bpo::options_description desc{"Options"};
    desc.add_options()
      ("help,h", "Display help")
      ("clouds,c", bpo::value<std::vector<std::string>>()->multitoken()->composing(), "PCD clouds");

    bpo::positional_options_description pos_desc;
    pos_desc.add("clouds", -1);

    bpo::command_line_parser parser{argc, argv};
    parser.options(desc).positional(pos_desc);

    bpo::variables_map vm;
    bpo::store(parser.run(), vm);
    bpo::notify(vm);

    if (vm.count("help")) {
      std::cout << "Usage:" << '\n';
      std::cout << "  " << argv[0] << " <pcd1> ... <pcdN>" << '\n';
      std::cout << desc << '\n';
      return 0;
    }

    if (not vm.count("clouds")) {
        std::cout << "Error: PCD files are missing." << '\n';

      std::cout << "Usage:" << '\n';
      std::cout << "  " << argv[0] << " <pcd1> ... <pcdN>" << '\n';
      std::cout << desc << '\n';
      return 1;
    }


    // Be sure directory does not exist already
    bfs::path png_dir("pngs");
    if (bfs::exists(png_dir)) {
      std::stringstream ss;
      if (bfs::is_directory(png_dir))
        ss << "A directory named \"" << png_dir.string() << "\" already exists. Please remove it or choose another directory to output the point clouds.";
      else if (bfs::is_regular_file(png_dir))
        ss << "A regular file named \"" << png_dir.string() << "\" already exists. Please remove this file or choose another directory name to output the point clouds.";
      else
        ss << "\"" << png_dir.string() << "\" already exists. Please remove it or choose another directory name to output the point clouds.";
      throw std::runtime_error(ss.str());
    }

    // Create png dir
    {
      bool dir_created = bfs::create_directory(png_dir);
      if (not dir_created) {
        std::stringstream ss;
        ss << "Could not create \"" << png_dir.string() << "\" directory.";
        throw std::runtime_error(ss.str());
      }
    }

    pcl::PCDReader pcd;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters ();

    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

    typedef pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> ColorHandler;
    typedef ColorHandler::Ptr ColorHandlerPtr;
    typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

    ColorHandlerPtr color_handler;

    std::vector<std::string> pcd_filenames = vm["clouds"].as<std::vector<std::string>>();
    auto pcd_it = pcd_filenames.begin();
    while (!viewer->wasStopped ()) {
      if (pcd_it != pcd_filenames.end()) {
        // Read cloud
        bfs::path pcd_path = *pcd_it;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcd.read(pcd_path.string(), *cloud);

        // Compute useful transformations
        // sensor frame
        Eigen::Affine3f T_world_sensor;
        T_world_sensor = cloud->sensor_orientation_;
        T_world_sensor.translation() = cloud->sensor_origin_.head<3>();
        // sensor frame but considering only yaw
        Eigen::Affine3f T_world_sensoryaw;
        T_world_sensoryaw = Eigen::AngleAxis<float>(idt::ASN1BitstreamLogger::Yaw(cloud->sensor_orientation_), Eigen::Vector3f::UnitZ());
        T_world_sensoryaw.translation() = cloud->sensor_origin_.head<3>();
        // camera position, put behind the robot when we consider only sensor yaw
        Eigen::Affine3f T_world_camera = T_world_sensoryaw * Eigen::Translation<float,3>(45,0,20);

        // Add a new at current sensor pose
        viewer->addCoordinateSystem (1.0, T_world_sensor, "sensor_frame");
        // Or alternativaly we can update old one
        // viewer->updateCoordinateSystemPose ("sensor_frame", T_world_sensor);

        // Remove previous point cloud
        if(pcd_it != pcd_filenames.begin()) {
          viewer->removePointCloud("sample cloud");
        }

        // Add cloud
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> (cloud, "z"));
        // geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<PointCloud> (cloud));

        viewer->addPointCloud<pcl::PointXYZI> (cloud, *color_handler, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

        // Put the camera behind the robot, looking at the sensor origin, and upwards
        viewer->setCameraPosition (T_world_camera.translation()[0], // pos_x
                                   T_world_camera.translation()[1], // pos_y
                                   T_world_camera.translation()[2], // pos_z
                                   T_world_sensoryaw.translation()[0],  // view_x
                                   T_world_sensoryaw.translation()[1],  // view_y
                                   T_world_sensoryaw.translation()[2],  // view_z
                                   0,                         // up_x
                                   0,                         // up_y
                                   1);                        // up_z

        // Render
        bool force_redraw = true;
        viewer->spinOnce (1, force_redraw);
  
        // Save png
        bfs::path png_path = pcd_path.parent_path() / png_dir / bfs::path(pcd_path).replace_extension(".png");
        // std::cout << png_path.string() << std::endl;
        viewer->saveScreenshot(png_path.string());

        // Move to next file
        pcd_it++;

      } else {
        // No more pcd files! Spin here until we stop the viewer.
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }

    } 

  } catch (const bpo::error &ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  }

}