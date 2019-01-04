#include "PointCloudExtractor.hpp"
#include "ImagePairExtractor.hpp"
#include "PoseExtractor.hpp"
#include "GpsExtractor.hpp"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

void print_usage(int argc, char **argv, const bpo::options_description &desc)
{
  std::cout << "Usage:" << '\n';
  std::cout << "  " << argv[0] << " {-avfnrotg} ... <output-dir> <bag1> ... <bagN>" << "\n\n";
  std::cout << desc << '\n';
}

namespace std
{
  std::ostream& operator<<(std::ostream &os, const std::vector<std::string> &vec) 
  {    
    for (auto item : vec) 
    { 
      os << item << " "; 
    } 
    return os; 
  }
} 

int main(int argc, char **argv)
{
  try {
    // Parse program options
    bpo::options_description extraction{"Extraction options"};
    extraction.add_options()
      ("all,a", bpo::bool_switch(), "Extract all data")
      ("velodyne,v", bpo::bool_switch(), "Extract velodyne point clouds")
      ("front,f", bpo::bool_switch(), "Extract front cam images")
      ("nav,n", bpo::bool_switch(), "Extract nav cam images")
      ("rear,r", bpo::bool_switch(), "Extract rear cam images")
      ("odom,o", bpo::bool_switch(), "Extract odometry (RMP) pose data")
      ("tokamak,t", bpo::bool_switch(), "Extract tokamak pose data")
      ("gps,g", bpo::bool_switch(), "Extract GPS pose data")
      ;

    bpo::options_description velodyne{"Velodyne specific options"};
    velodyne.add_options()
      ("velodyne-topic", bpo::value<std::string>()->default_value("/velodyne/point_cloud"), "Velodyne point cloud topic")
      ("velodyne-png", bpo::bool_switch(), "Extract point cloud views as pngs (Warning: this launches a PCLViewer window during extraction)")
      ;

    bpo::options_description cam{"Camera specific options"};
    cam.add_options()
      ("front-topic", bpo::value<std::string>()->default_value("/FrontCam/Stereo"), "Front cam stereo pair topic")
      ("front-ext", bpo::value<std::string>()->default_value("pgm"), "File extension for the Front cam data")
      ("nav-topic", bpo::value<std::string>()->default_value("/NavCam/Stereo"), "Nav cam stereo pair topic")
      ("nav-ext", bpo::value<std::string>()->default_value("pgm"), "File extension for the Nav cam data")
      ("rear-topic", bpo::value<std::string>()->default_value("/RearCam/Stereo"), "Rear cam stereo pair topic")
      ("rear-ext", bpo::value<std::string>()->default_value("pgm"), "File extension for the Rear cam data")
      ;

    bpo::options_description odom{"Odometry specific options"};
    odom.add_options()
      ("odom-topic", bpo::value<std::vector<std::string>>()->default_value({"/rmp400/PoseInfuse","/rmp440/PoseInfuse"}), "Odometry topic")
      ;

    bpo::options_description tokamak{"Tokamak specific options"};
    tokamak.add_options()
      ("tokamak-topic", bpo::value<std::string>()->default_value("/pose_robot_pom"), "Tokamak topic")
      ;

    bpo::options_description gps{"GPS specific options"};
    gps.add_options()
      ("gps-pose-topic", bpo::value<std::string>()->default_value("/bestutm_infuse"), "GPS pose topic")
      ("gps-info-topic", bpo::value<std::string>()->default_value("/bestutm_info"), "GPS info topic")
      ;

    // Backend options will be hidden from the user
    bpo::options_description backend{"Backend Options"};
    backend.add_options()
      ("output-dir", bpo::value<std::string>(), "Directory where to put the extracted dataset")
      ("bags", bpo::value<std::vector<std::string>>()->multitoken()->composing(), "Bags composing the dataset")
      ;
    // Positional options to capture backend arguments
    bpo::positional_options_description pos_backend;
    pos_backend.add("output-dir", 1).add("bags", -1);

    // All options, used for parsing
    bpo::options_description all("General options");
    all.add(extraction).add(velodyne).add(cam).add(odom).add(tokamak).add(gps).add(backend);
    all.add_options()
      ("help,h", "Display help")
      ("debug,d", bpo::bool_switch(), "Run in debug mode (extract debug data, print debug messages, etc...)")
      ;

    // Only the options that should be visible to the user
    bpo::options_description visible("General options");
    visible.add(extraction).add(velodyne).add(cam).add(odom).add(tokamak).add(gps);
    visible.add_options()
      ("help,h", "Display help")
      ;

    bpo::command_line_parser parser{argc, argv};
    parser.options(all).positional(pos_backend);

    bpo::variables_map vm;
    bpo::store(parser.run(), vm);
    bpo::notify(vm);

    if (vm.count("help")) {
      print_usage(argc, argv, visible);
      return 0;
    }

    if (not vm.count("output-dir") or not vm.count("bags")) {
      if (not vm.count("output-dir"))
        std::cout << "Error: Output directory is missing." << '\n';
      if (not vm.count("bags")) 
        std::cout << "Error: Bag files are missing." << '\n';
      print_usage(argc, argv, visible);
      return 1;
    }

    // Makes sure we have at least one extraction informed
    if (not vm["all"].as<bool>() and
        not vm["velodyne"].as<bool>() and
        not vm["front"].as<bool>() and
        not vm["nav"].as<bool>() and
        not vm["rear"].as<bool>() and
        not vm["odom"].as<bool>() and
        not vm["tokamak"].as<bool>() and
        not vm["gps"].as<bool>()) {
     std::cout << "Error: Extraction option not informed. Please use at least one extraction option." << '\n';
      print_usage(argc, argv, visible);
      return 1;
    }

    // Makes sure the output dir does not already exists
    bfs::path output_dir = vm["output-dir"].as<std::string>();
    if (bfs::exists(output_dir)) {
      std::stringstream ss;
      if (bfs::is_directory(output_dir))
        ss << "A directory named \"" << output_dir.string() << "\" already exists. Please remove it or choose another directory to output the dataset.";
      else if (bfs::is_regular_file(output_dir))
        ss << "A regular file named \"" << output_dir.string() << "\" already exists. Please remove this file or choose another directory name to output the dataset.";
      else
        ss << "\"" << output_dir.string() << "\" already exists. Please remove it or choose another directory name to output the dataset.";
      throw std::runtime_error(ss.str());
    }

    // Create output dir
    {
      bool dir_created = bfs::create_directory(output_dir);
      if (not dir_created) {
        std::stringstream ss;
        ss << "Could not create \"" << output_dir.string() << "\" directory.";
        throw std::runtime_error(ss.str());
      }
    }

    if (vm["all"].as<bool>() or vm["velodyne"].as<bool>()) {
      // Create the extractor and extract clouds.
      bfs::path velodyne_output_dir = output_dir / "velodyne";
      infuse_debug_tools::PointCloudExtractor cloud_extractor{
        velodyne_output_dir.string(),
        vm["bags"].as<std::vector<std::string>>(),
        vm["velodyne-topic"].as<std::string>(),
        vm["velodyne-png"].as<bool>(),
        vm["debug"].as<bool>(),
      };
      cloud_extractor.Extract();
    }

    std::array<std::string, 3> cam_names = {"front", "nav", "rear"};
    for (const auto & cam_name : cam_names) {
      if (vm["all"].as<bool>() or vm[cam_name].as<bool>()) {
        // Create the cam extractor and extract images.
        bfs::path cam_output_dir = output_dir / (cam_name + "_cam");
        infuse_debug_tools::ImagePairExtractor cam_extractor{
          cam_output_dir.string(),
          vm["bags"].as<std::vector<std::string>>(),
          vm[cam_name + "-topic"].as<std::string>(),
          vm[cam_name + "-ext"].as<std::string>()
        };
        cam_extractor.Extract();
      }
    }

    if (vm["all"].as<bool>() or vm["odom"].as<bool>())
    {
      // Create the extractor and extract odometry poses.
      bfs::path odom_output_dir = output_dir / "odometry";
      infuse_debug_tools::PoseExtractor pose_extractor{
          odom_output_dir.string(),
          vm["bags"].as<std::vector<std::string>>(),
          vm["odom-topic"].as<std::vector<std::string>>(),
          "odometry"
      };
      pose_extractor.Extract();
    }

    if (vm["all"].as<bool>() or vm["tokamak"].as<bool>())
    {
      // Create the extractor and extract tokamak poses.
      bfs::path tokamak_output_dir = output_dir / "tokamak";
      infuse_debug_tools::PoseExtractor pose_extractor{
          tokamak_output_dir.string(),
          vm["bags"].as<std::vector<std::string>>(),
          {vm["tokamak-topic"].as<std::string>()},
          "tokamak"
      };
      pose_extractor.Extract();
    }

    if (vm["all"].as<bool>() or vm["gps"].as<bool>())
    {
      // Create the extractor and extract gps poses.
      bfs::path gps_output_dir = output_dir / "gps";
      infuse_debug_tools::GpsExtractor gps_extractor{
          gps_output_dir.string(),
          vm["bags"].as<std::vector<std::string>>(),
          vm["gps-pose-topic"].as<std::string>(),
          vm["gps-info-topic"].as<std::string>()
      };
      gps_extractor.Extract();
    }

  } catch (const bpo::error &ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  } catch (const std::runtime_error& ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  }

  return 0;
}
