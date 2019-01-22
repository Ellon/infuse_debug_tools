#include "ImagePairMatcher.hpp"

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
    bpo::options_description data{"Data options"};
    data.add_options()
      ("all,a", bpo::bool_switch(), "Process matching on all data")
      ("front,f", bpo::bool_switch(), "Process matching on front cam images")
      ("nav,n", bpo::bool_switch(), "Process matching on nav cam images")
      ("rear,r", bpo::bool_switch(), "Process matching on rear cam images")
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

    bpo::options_description rect{"Stereo rectification specific options"};
    rect.add_options()
      ("xratio", bpo::value<std::int16_t>()->default_value(3), "Degradation ratio to be applied over the x-axis")
      ("yratio", bpo::value<std::int16_t>()->default_value(3), "Degradation ratio to be applied over the y-axis")
      ("scaling", bpo::value<std::double_t>()->default_value(0), "Free scaling parameter")
      ("calibration_file_path", bpo::value<std::string>()->default_value(" "), "Path to the calibration file")
      ;

    bpo::options_description matcher{"Stereo matching specific options"};
    matcher.add_options()
        ("algorithm", bpo::value<std::int16_t>()->default_value(1), "Algorithm to be used")
        ("min_disparity", bpo::value<std::int16_t>()->default_value(0), "Minimum possible disparity value")
        ("num_disparities", bpo::value<std::int16_t>()->default_value(64), "Maximum disparity minus minimum disparity")
        ("block_size", bpo::value<std::int16_t>()->default_value(1), "Matched block size")
        ("speckle_window_size", bpo::value<std::int16_t>()->default_value(50), "Maximum size of smooth disparity regions to consider their noise speckles and invalidate")
        ("speckle_range", bpo::value<std::int16_t>()->default_value(1), "Maximum disparity variation within each connected component")
        ("disp12_max_diff", bpo::value<std::int16_t>()->default_value(-1), "Maximum allowed difference (in integer pixel units) in the left-right disparity check")
        ("pre_filter_cap", bpo::value<std::int16_t>()->default_value(31), "Truncation value for the prefiltered image pixels")
        ("uniqueness_ratio", bpo::value<std::int16_t>()->default_value(15), "Margin in percentage by which the best (minimum) computed cost function value should win the second best value to consider the found match correct")

        ("pre_filter_type", bpo::value<std::int16_t>()->default_value(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE), "Pre-Filter Type")
        ("pre_filter_size", bpo::value<std::int16_t>()->default_value(9), "Size of the Pre-Filter")
        ("texture_threshold", bpo::value<std::int16_t>()->default_value(10), "Texture Threshold")

        ("P1", bpo::value<std::int16_t>()->default_value(0), "The first parameter controlling the disparity smoothness")
        ("P2", bpo::value<std::int16_t>()->default_value(0), "The second parameter controlling the disparity smoothness")
        ("mode", bpo::value<std::int16_t>()->default_value(cv::StereoSGBM::MODE_SGBM), "Algorithm mode")

        ("use_filter", bpo::value<bool>()->default_value(false), "Set to true to filter the disparity map")
        ("use_confidence", bpo::value<bool>()->default_value(false), "Filtering with confidence requires two disparity maps (for the left and right views) and is approximately two times slower")
        ("depth_discontinuity_radius", bpo::value<std::int16_t>()->default_value(0), "DepthDiscontinuityRadius is a parameter used in confidence computation")
        ("lambda", bpo::value<std::double_t>()->default_value(8000.0), "Lambda is a parameter defining the amount of regularization during filtering")
        ("lrc_thresh", bpo::value<std::int16_t>()->default_value(24), "LRCthresh is a threshold of disparity difference used in left-right-consistency check during confidence map computation")
        ("sigma_color", bpo::value<std::double_t>()->default_value(1.5), "SigmaColor is a parameter defining how sensitive the filtering process is to source image edges")
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
    all.add(data).add(cam).add(rect).add(matcher).add(backend);
    all.add_options()
      ("help,h", "Display help")
      ;

    // Only the options that should be visible to the user
    bpo::options_description visible("General options");
    visible.add(data).add(cam).add(rect).add(matcher);
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

    // Makes sure we have at least one data option informed
    if (not vm["all"].as<bool>() and
        not vm["front"].as<bool>() and
        not vm["nav"].as<bool>() and
        not vm["rear"].as<bool>()) {
     std::cout << "Error: Data option not informed. Please use at least one data option." << '\n';
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

    // Fill the stereo matching parameters struct
    infuse_debug_tools::ImagePairMatcher::StereoMatchingParams matching_parameters;
    matching_parameters.stereoMatcher.algorithm = vm["algorithm"].as<std::int16_t>();
    matching_parameters.stereoMatcher.min_disparity = vm["min_disparity"].as<std::int16_t>();
    matching_parameters.stereoMatcher.num_disparities = vm["num_disparities"].as<std::int16_t>();
    matching_parameters.stereoMatcher.block_size = vm["block_size"].as<std::int16_t>();
    matching_parameters.stereoMatcher.speckle_window_size = vm["speckle_window_size"].as<std::int16_t>();
    matching_parameters.stereoMatcher.speckle_range = vm["speckle_range"].as<std::int16_t>();
    matching_parameters.stereoMatcher.disp12_max_diff = vm["disp12_max_diff"].as<std::int16_t>();
    matching_parameters.stereoMatcher.pre_filter_cap = vm["pre_filter_cap"].as<std::int16_t>();
    matching_parameters.stereoMatcher.uniqueness_ratio = vm["uniqueness_ratio"].as<std::int16_t>();

    matching_parameters.stereoMatcher.bm_params.pre_filter_type = vm["pre_filter_type"].as<std::int16_t>();
    matching_parameters.stereoMatcher.bm_params.pre_filter_size = vm["pre_filter_size"].as<std::int16_t>();
    matching_parameters.stereoMatcher.bm_params.texture_threshold = vm["texture_threshold"].as<std::int16_t>();

    matching_parameters.stereoMatcher.sgbm_params.P1 = vm["P1"].as<std::int16_t>();
    matching_parameters.stereoMatcher.sgbm_params.P2 = vm["P2"].as<std::int16_t>();
    matching_parameters.stereoMatcher.sgbm_params.mode = vm["mode"].as<std::int16_t>();

#if WITH_XIMGPROC
    matching_parameters.filter.use_filter = vm["use_filter"].as<bool>();
    matching_parameters.filter.use_confidence = vm["use_confidence"].as<bool>();
    matching_parameters.filter.depth_discontinuity_radius = vm["depth_discontinuity_radius"].as<std::int16_t>();
    matching_parameters.filter.lambda = vm["lambda"].as<std::double_t>();
    matching_parameters.filter.lrc_thresh = vm["lrc_thresh"].as<std::int16_t>();
    matching_parameters.filter.sigma_color = vm["sigma_color"].as<std::double_t>();
#endif

    // Fill the rectification stereo parameters struct
    infuse_debug_tools::ImagePairMatcher::StereoRectificationParams rect_parameters;
    rect_parameters.xratio = vm["xratio"].as<std::int16_t>();
    rect_parameters.yratio = vm["yratio"].as<std::int16_t>();
    rect_parameters.scaling = vm["scaling"].as<std::double_t>();
    rect_parameters.calibration_file_path = vm["calibration_file_path"].as<std::string>();

    // Process camera data
    std::array<std::string, 3> cam_names = {"front", "nav", "rear"};
    for (const auto & cam_name : cam_names) {
      if (vm["all"].as<bool>() or vm[cam_name].as<bool>()) {
        // Create the cam matcher and process stereo matching.
        bfs::path disparity_output_dir = output_dir / (cam_name + "_disparity");
        infuse_debug_tools::ImagePairMatcher cam_matcher{
          disparity_output_dir.string(),
          vm["bags"].as<std::vector<std::string>>(),
          vm[cam_name + "-topic"].as<std::string>(),
          vm[cam_name + "-ext"].as<std::string>(),
          matching_parameters,
          rect_parameters
        };
        cam_matcher.Match();
      }
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
