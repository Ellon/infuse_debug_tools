#include "ImagePairExtractor.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <boost/progress.hpp>

#include <opencv2/opencv.hpp>

#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools {

ImagePairExtractor::ImagePairExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &image_topic, const std::string & img_extension)
  : output_dir_{output_dir},
    bag_paths_{bag_paths},
    image_topic_{image_topic},
    asn1_frame_pair_ptr_{std::make_unique<asn1SccFramePair>()},
    length_img_filename_{5},
    image_count_{0},
    image_max_{std::stoul(std::string("1") + std::string(length_img_filename_, '0')) - 1},
    img_extension_{img_extension}
{
}


void ImagePairExtractor::Extract()
{
  // Vector of topics used to create a view on the bag
  std::vector<std::string> topics = {image_topic_};

  // Get the number of messages to process
  size_t n_images = 0;
  for (auto bag_path : bag_paths_) {
    rosbag::Bag bag(bag_path); // bagmode::Read by default
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    n_images += view.size();
    bag.close();
  }

  // Stop here if there's nothing on the topic
  if (n_images == 0) {
    std::cout << "Warning: Nothing to extract on topic " << image_topic_ << std::endl;
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
  left_dir_       = lambda_create_subdir(output_dir_, "left");
  left_data_dir_  = lambda_create_subdir(left_dir_,   "data");
  right_dir_      = lambda_create_subdir(output_dir_, "right");
  right_data_dir_ = lambda_create_subdir(right_dir_,  "data");

  // // Write dataformat file. The rationalle of keeping the dataformat separated
  // // from the metadata is that this way it is possible to associate the cloud
  // // number with the line in the metadata file.
  // std::ofstream dataformat_ofs((output_dir_ / "dataformat.txt").string());
  // {
  //   std::vector<std::string> entries{ASN1BitstreamLogger::GetPointcloudLogEntries()};
  //   unsigned int index = 1;
  //   for (auto entry : entries) {
  //     dataformat_ofs << "# " << std::setw(2) << index << " - " << entry << '\n';
  //     index++;
  //   }
  // }
  // dataformat_ofs.close();

  // // Setup metadata file
  // metadata_ofs_.open((output_dir_ / "metadata.txt").string());

  // Setup progress display
  std::cout << "Extracting " << n_images << " image pairs to " << output_dir_.string() << "...";
  boost::progress_display show_progress( n_images );

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
          ProcessImagePair(i);
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

  // metadata_ofs_.close();

}

void ImagePairExtractor::ProcessImagePair(const infuse_msgs::asn1_bitstream::Ptr& msg)
{
  // Guard against overflow on the filename numbers
  if (image_count_ > image_max_)
    throw std::runtime_error("Overflow on the image filename counter. Please increase the number of characters to be used to compose the filename");

  // Initialize asn1 point cloud to be sure we have clean object.
  asn1SccFramePair_Initialize(asn1_frame_pair_ptr_.get());

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
  res = asn1SccFramePair_Decode(asn1_frame_pair_ptr_.get(), &bstream, &errorCode);
  if (not res) {
    std::stringstream ss;
    ss << "Error decoding asn1SccFramePair! Error: " << errorCode << "\n";
    throw std::runtime_error(ss.str());
  }

  ProcessImage(asn1_frame_pair_ptr_->left, left_data_dir_);
  ProcessImage(asn1_frame_pair_ptr_->right, right_data_dir_);

  image_count_++;
}

void ImagePairExtractor::ProcessImage(asn1SccFrame & asn1_frame, boost::filesystem::path data_dir)
{
  // Bind to the pointer inside the asn1 variable
  cv::Mat img( asn1_frame.data.rows, asn1_frame.data.cols,
    CV_MAKETYPE((int)(asn1_frame.data.depth), asn1_frame.data.channels),
    asn1_frame.data.data.arr, asn1_frame.data.rowSize);

  // Compose output filename
  std::string img_filename = std::to_string(image_count_);
  img_filename = std::string(length_img_filename_ - img_filename.length(), '0') + img_filename + "." + img_extension_;
  bfs::path img_path = data_dir / img_filename;

  // save the file
  cv::imwrite(img_path.string(), img);
}

} // infuse_debug_tools
