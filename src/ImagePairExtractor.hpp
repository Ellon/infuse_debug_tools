#ifndef IMAGE_PAIR_EXTRACTOR_HPP
#define IMAGE_PAIR_EXTRACTOR_HPP

#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/Frame.h>

#include <boost/filesystem.hpp>

namespace infuse_debug_tools {

class ImagePairExtractor {
public:
  ImagePairExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &image_topic, const std::string & img_extension);
  void Extract();

private:
  void ProcessImagePair(const infuse_msgs::asn1_bitstream::Ptr& msg);
  void ProcessImage(asn1SccFrame & asn1_frame, boost::filesystem::path data_dir);

private:
  //! Directory where to put the dataset
  boost::filesystem::path output_dir_;
  //! Strings with paths to the bags to be processed
  std::vector<std::string> bag_paths_;
  //! Name of the topics with the images
  std::string image_topic_;
  //! Directories where to put the image files (set on Extract())
  boost::filesystem::path left_dir_;
  boost::filesystem::path left_data_dir_;
  boost::filesystem::path right_dir_;
  boost::filesystem::path right_data_dir_;
  //! Variable used to decode the ASN1 message into.
  std::unique_ptr<asn1SccFramePair> asn1_frame_pair_ptr_;
  //! Counter for the number of images files written. Used to create the pcd filename
  size_t image_count_;
  //! Number of characters to be used when creating pcd filename
  unsigned int length_img_filename_;
  std::string img_extension_;
  //! Stream used to write the metadata file
  // std::ofstream metadata_ofs_;
};

} // infuse_debug_tools

#endif // IMAGE_PAIR_EXTRACTOR_HPP
