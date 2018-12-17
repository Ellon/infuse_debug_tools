#include "PointCloudExtractor.hpp"

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