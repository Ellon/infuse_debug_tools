#include "PointCloudExtractor.hpp"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace bpo = boost::program_options;
namespace bfs = boost::filesystem;

void print_usage(int argc, char **argv, const bpo::options_description &desc)
{
  std::cout << "Usage:" << '\n';
  std::cout << "  " << argv[0] << " {--velodyne} ... <output-dir> <bag1> ... <bagN>" << '\n';
  std::cout << desc << '\n';
}

int main(int argc, char **argv)
{
  try {
    // Parse program options
    bpo::options_description desc{"Options"};
    desc.add_options()
      ("help,h", "Display help")
      ("output-dir,o", bpo::value<std::string>(), "Directory where to put the extracted dataset")
      ("bags,b", bpo::value<std::vector<std::string>>()->multitoken()->composing(), "Bags composing the dataset")
      ("velodyne,v", bpo::bool_switch(), "Extract velodyne point clouds")
      ("velodyne-topic", bpo::value<std::string>()->default_value("/velodyne/point_cloud"), "Velodyne point cloud topic")
      ("velodyne-png", bpo::bool_switch(), "Extract point cloud views as pngs (Warning: this launches a PCLViewer window during extraction)")
      ;

    bpo::positional_options_description pos_desc;
    pos_desc.add("output-dir", 1).add("bags", -1);

    bpo::command_line_parser parser{argc, argv};
    parser.options(desc).positional(pos_desc);

    bpo::variables_map vm;
    bpo::store(parser.run(), vm);
    bpo::notify(vm);

    if (vm.count("help")) {
      print_usage(argc, argv, desc);
      return 0;
    }

    if (not vm.count("output-dir") or not vm.count("bags")) {
      if (not vm.count("output-dir"))
        std::cout << "Error: Output directory is missing." << '\n';
      if (not vm.count("bags")) 
        std::cout << "Error: Bag files are missing." << '\n';
      print_usage(argc, argv, desc);
      return 1;
    }

    // Makes sure we have at least one extraction informed
    if (not vm.count("velodyne")) {
      std::cout << "Error: Extraction not informed. Please inform at least one data type to extact." << '\n';
      print_usage(argc, argv, desc);
      return 1;
    }

    // Makes sure the output dir does not already exists
    bfs::path output_dir = vm["output-dir"].as<std::string>();
    if (bfs::exists(output_dir)) {
      std::stringstream ss;
      if (bfs::is_directory(output_dir))
        ss << "A directory named \"" << output_dir.string() << "\" already exists. Please remove it or choose another directory to output the point clouds.";
      else if (bfs::is_regular_file(output_dir))
        ss << "A regular file named \"" << output_dir.string() << "\" already exists. Please remove this file or choose another directory name to output the point clouds.";
      else
        ss << "\"" << output_dir.string() << "\" already exists. Please remove it or choose another directory name to output the point clouds.";
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


    if (vm.count("velodyne")) {
      // Create the extractor and extract clouds.
      bfs::path velodyne_output_dir = output_dir / "velodyne";
      infuse_debug_tools::PointCloudExtractor cloud_extractor{
        velodyne_output_dir.string(),
        vm["bags"].as<std::vector<std::string>>(),
        vm["velodyne-topic"].as<std::string>(),
        vm["velodyne-png"].as<bool>()
      };

      cloud_extractor.Extract();
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