#include "ImagePairMatcher.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <boost/progress.hpp>

#include <opencv2/opencv.hpp>

#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools {

ImagePairMatcher::ImagePairMatcher(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &image_topic, const std::string & img_extension, infuse_debug_tools::ImagePairMatcher::StereoMatchingParams & matching_parameters, infuse_debug_tools::ImagePairMatcher::StereoRectificationParams & rect_parameters)
  : output_dir_{output_dir},
    bag_paths_{bag_paths},
    image_topic_{image_topic},
    img_extension_{img_extension},
    matching_parameters_{matching_parameters},
    rect_parameters_{rect_parameters},
    asn1_frame_pair_ptr_{std::make_unique<asn1SccFramePair>()},
    asn1_rect_out_frame_pair_ptr_{std::make_unique<asn1SccFramePair>()},
    out_raw_disparity_ptr_{std::make_unique<asn1SccFrame>()},
    out_color_disparity_ptr_{std::make_unique<asn1SccFrame>()},
    length_img_filename_{5},
    image_count_{0},
    image_max_{std::stoul(std::string("1") + std::string(length_img_filename_, '0')) - 1}
{
}


void ImagePairMatcher::Match()
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
  disparity_data_dir_       = lambda_create_subdir(output_dir_,   "data");
  disparity_metadata_dir_   = lambda_create_subdir(output_dir_,   "metadata");

  // Fill metadata names vector
  std::vector<std::string> metadata_names;
  metadata_names.push_back("left_timestamp");
  metadata_names.push_back("right_timestamp");
  metadata_names.push_back("number_paired_pixels");
  metadata_names.push_back("percentage_of_paired_pixels");

  // Write dataformat files. The rationalle of keeping the dataformat
  // separated from the metadata is that this way it is possible to associate
  // the cloud number with the line in the metadata file.
  // We do it using a lambda function.
  auto lambda_create_dataformat_file = [](bfs::path dir, std::string file_prefix, const std::vector<std::string> & entries) -> void {
    std::ofstream dataformat_ofs((dir / (file_prefix + "dataformat.txt")).string());
    unsigned int index = 1;
    for (auto entry : entries) {
      dataformat_ofs << "# " << std::setw(2) << index << " - " << entry << '\n';
      index++;
    }
    dataformat_ofs.close();
  };
  lambda_create_dataformat_file(output_dir_, "disparity_", metadata_names);

  // Setup metadata file
  disparity_metadata_ofs_.open((output_dir_ / "disparity_all_metadata.txt").string());

  // Setup progress display
  std::cout << "Extracting " << n_images << " image pairs to " << output_dir_.string() << "/...";
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

void ImagePairMatcher::ProcessImagePair(const infuse_msgs::asn1_bitstream::Ptr& msg)
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

  // Process stereo matching
  std::vector<std::string> metadata_values;
  ProcessStereoMatching(*asn1_frame_pair_ptr_, *out_raw_disparity_ptr_, *out_color_disparity_ptr_, metadata_values);

  // Write images
//  ProcessImage(*out_raw_disparity_ptr_, disparity_data_dir_);
  ProcessImage(*out_color_disparity_ptr_, disparity_data_dir_);

  // Write all_metadata files
  for(std::vector<int>::size_type i = 0; i != metadata_values.size(); i++) {
      disparity_metadata_ofs_ << metadata_values.at(i) << " ";
  }
  disparity_metadata_ofs_ << '\n';

  // Lambda function to dump metadata from map
  auto lambda_log_metadata_on_file = [this](bfs::path metadata_dir, std::vector<std::string> metadata_values) -> void {
    // Compose output filename
    std::string filename = std::to_string(this->image_count_);
    filename = std::string(this->length_img_filename_ - filename.length(), '0') + filename + ".txt";
    bfs::path metadata_path = metadata_dir / filename;

    // Write the metadata file
    std::ofstream img_metadata_ofs(metadata_path.string());
    for(std::vector<int>::size_type i = 0; i != metadata_values.size(); i++) {
        img_metadata_ofs << metadata_values.at(i) << " ";
    }
    img_metadata_ofs.close();

  };
  // Dump metadata of frames in a separated files
  lambda_log_metadata_on_file(disparity_metadata_dir_, metadata_values);

  image_count_++;
}

void ImagePairMatcher::ProcessImage(asn1SccFrame & asn1_frame, boost::filesystem::path data_dir)
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

void ImagePairMatcher::ProcessStereoMatching(asn1SccFramePair& in_frame_pair, asn1SccFrame& out_raw_disparity, asn1SccFrame& out_color_disparity, std::vector<std::string> & metadata_values)
{
    cv::Mat rect_left, rect_right;
    cv::Mat disparity;

    ProcessStereoRectification(in_frame_pair, *asn1_rect_out_frame_pair_ptr_, rect_left, rect_right);

    // Using Algorithm StereoBM
    if(matching_parameters_.stereoMatcher.algorithm == 0)
    {
        if(_bm.empty())
        {
            _bm = cv::StereoBM::create(matching_parameters_.stereoMatcher.num_disparities, matching_parameters_.stereoMatcher.block_size);
        }

        _bm->setBlockSize(matching_parameters_.stereoMatcher.block_size);
        _bm->setDisp12MaxDiff(matching_parameters_.stereoMatcher.disp12_max_diff);
        _bm->setMinDisparity(matching_parameters_.stereoMatcher.min_disparity);
        _bm->setNumDisparities(matching_parameters_.stereoMatcher.num_disparities);
        _bm->setPreFilterCap(matching_parameters_.stereoMatcher.pre_filter_cap);
        _bm->setPreFilterSize(matching_parameters_.stereoMatcher.bm_params.pre_filter_size);
        _bm->setPreFilterType(matching_parameters_.stereoMatcher.bm_params.pre_filter_type);
        _bm->setSpeckleRange(matching_parameters_.stereoMatcher.speckle_range);
        _bm->setSpeckleWindowSize(matching_parameters_.stereoMatcher.speckle_window_size);
        _bm->setTextureThreshold(matching_parameters_.stereoMatcher.bm_params.texture_threshold);
        _bm->setUniquenessRatio(matching_parameters_.stereoMatcher.uniqueness_ratio);

        _bm->compute(rect_left, rect_right, disparity);



    }
    // Using Algorithm StereoSGBM
    else if(matching_parameters_.stereoMatcher.algorithm == 1)
    {
        if(_sgbm.empty())
        {
            _sgbm = cv::StereoSGBM::create(matching_parameters_.stereoMatcher.min_disparity, matching_parameters_.stereoMatcher.num_disparities, matching_parameters_.stereoMatcher.block_size, matching_parameters_.stereoMatcher.sgbm_params.P1, matching_parameters_.stereoMatcher.sgbm_params.P2, matching_parameters_.stereoMatcher.disp12_max_diff, matching_parameters_.stereoMatcher.pre_filter_cap, matching_parameters_.stereoMatcher.uniqueness_ratio, matching_parameters_.stereoMatcher.speckle_window_size, matching_parameters_.stereoMatcher.speckle_range, matching_parameters_.stereoMatcher.sgbm_params.mode);
        }

        _sgbm->setBlockSize(matching_parameters_.stereoMatcher.block_size);
        _sgbm->setDisp12MaxDiff(matching_parameters_.stereoMatcher.disp12_max_diff);
        _sgbm->setMinDisparity(matching_parameters_.stereoMatcher.min_disparity);
        _sgbm->setMode(matching_parameters_.stereoMatcher.sgbm_params.mode);
        _sgbm->setNumDisparities(matching_parameters_.stereoMatcher.num_disparities);
        _sgbm->setP1(matching_parameters_.stereoMatcher.sgbm_params.P1);
        _sgbm->setP2(matching_parameters_.stereoMatcher.sgbm_params.P2);
        _sgbm->setPreFilterCap(matching_parameters_.stereoMatcher.pre_filter_cap);
        _sgbm->setSpeckleRange(matching_parameters_.stereoMatcher.speckle_range);
        _sgbm->setSpeckleWindowSize(matching_parameters_.stereoMatcher.speckle_window_size);
        _sgbm->setUniquenessRatio(matching_parameters_.stereoMatcher.uniqueness_ratio);

        _sgbm->compute(rect_left, rect_right, disparity);

    }

#if WITH_XIMGPROC
    bool reset_filter = false;
    bool reset_matcher = false;
    if(matching_parameters_.stereoMatcher.algorithm != _algorithm)
    {
        _algorithm = matching_parameters_.stereoMatcher.algorithm;
        reset_filter = true;
        reset_matcher = true;
    }
    else if(matching_parameters_.filter.use_confidence != _use_confidence)
    {
        _use_confidence = matching_parameters_.filter.use_confidence;
        reset_filter = true;
    }

    if(matching_parameters_.filter.use_filter)
    {
        cv::Mat disparity_filtered;

        if(matching_parameters_.filter.use_confidence
                ){
            cv::Mat disparity_right;
            if(_right_matcher.empty() || reset_matcher)
            {
                switch(_algorithm)
                {
                    case 0:
                        _right_matcher = cv::ximgproc::createRightMatcher(_bm);
                        break;
                    case 1:
                        _right_matcher = cv::ximgproc::createRightMatcher(_sgbm);
                        break;
                }
            }

            _right_matcher->compute(rect_left, rect_right, disparity_right);

            if(_filter.empty() || reset_filter)
            {
                switch(_algorithm)
                {
                    case 0:
                        _filter = cv::ximgproc::createDisparityWLSFilter(_bm);
                        break;
                    case 1:
                        _filter = cv::ximgproc::createDisparityWLSFilter(_sgbm);
                        break;
                }
            }

            _filter->setDepthDiscontinuityRadius(matching_parameters_.filter.depth_discontinuity_radius);
            _filter->setLambda(matching_parameters_.filter.lambda);
            _filter->setLRCthresh(matching_parameters_.filter.lrc_thresh);
            _filter->setSigmaColor(matching_parameters_.filter.sigma_color);

            _filter->filter(disparity, rect_left, disparity_filtered, disparity_right);
        }
        else
        {
            if(_filter.empty() || reset_filter)
            {
                _filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
            }

            _filter->setDepthDiscontinuityRadius(matching_parameters_.filter.depth_discontinuity_radius);
            _filter->setLambda(matching_parameters_.filter.lambda);
            _filter->setSigmaColor(matching_parameters_.filter.sigma_color);

            _filter->filter(disparity, rect_left, disparity_filtered);
        }

        disparity = disparity_filtered;
    }
#endif

    // Convert Mat to ASN.1
    out_raw_disparity.metadata.msgVersion = frame_Version;
    out_raw_disparity.metadata = in_frame_pair.left.metadata;
    out_raw_disparity.intrinsic = in_frame_pair.left.intrinsic;
    out_raw_disparity.extrinsic = in_frame_pair.left.extrinsic;

    out_raw_disparity.metadata.mode = asn1Sccmode_UNDEF;
    out_raw_disparity.metadata.pixelModel = asn1Sccpix_DISP;
    out_raw_disparity.metadata.errValues.arr[0].type = asn1Sccerror_UNDEFINED;
    out_raw_disparity.metadata.errValues.arr[0].value = -16.0;
    out_raw_disparity.metadata.errValues.nCount = 1;

    double min_disp, max_disp;
    cv::minMaxLoc(disparity, &min_disp, &max_disp);
    out_raw_disparity.metadata.pixelCoeffs.arr[0] = 16.0;
    out_raw_disparity.metadata.pixelCoeffs.arr[1] = 0.0;
    out_raw_disparity.metadata.pixelCoeffs.arr[2] = in_frame_pair.baseline;
    out_raw_disparity.metadata.pixelCoeffs.arr[3] = max_disp;
    out_raw_disparity.metadata.pixelCoeffs.arr[4] = min_disp;

    out_raw_disparity.data.msgVersion = array3D_Version;
    out_raw_disparity.data.channels = static_cast<asn1SccT_UInt32>(disparity.channels());
    out_raw_disparity.data.rows = static_cast<asn1SccT_UInt32>(disparity.rows);
    out_raw_disparity.data.cols = static_cast<asn1SccT_UInt32>(disparity.cols);
    out_raw_disparity.data.depth = static_cast<asn1SccArray3D_depth_t>(disparity.depth());
    out_raw_disparity.data.rowSize = disparity.step[0];
    out_raw_disparity.data.data.nCount =  static_cast<int>(out_raw_disparity.data.rows * out_raw_disparity.data.rowSize);
    memcpy(out_raw_disparity.data.data.arr, disparity.data, static_cast<size_t>(out_raw_disparity.data.data.nCount));

    // Convert the filtered image as a cv::Mat for display
    cv::Mat filtered =  cv::Mat(static_cast<int>(out_raw_disparity.data.rows), static_cast<int>(out_raw_disparity.data.cols),
                                CV_MAKETYPE(static_cast<int>(out_raw_disparity.data.depth), static_cast<int>(out_raw_disparity.data.channels)),
                                out_raw_disparity.data.data.arr, out_raw_disparity.data.rowSize);

    // Apply a colormap
    cv::Mat filteredDisparityColor;
    double min,    max;
    cv::minMaxLoc(filtered, &min, &max);
    filtered.convertTo(filtered, CV_8U, 255 / (max - min), -255.0 * min / (max - min));
    cv::Mat mask = filtered > 0;
    cv::applyColorMap(filtered, filtered, 2);
    filtered.copyTo(filteredDisparityColor, mask);

    // Convert Mat to ASN.1
    out_color_disparity.metadata.msgVersion = frame_Version;
    out_color_disparity.metadata = in_frame_pair.left.metadata;
    out_color_disparity.intrinsic = in_frame_pair.left.intrinsic;
    out_color_disparity.extrinsic = in_frame_pair.left.extrinsic;

    out_color_disparity.metadata.mode = asn1Sccmode_UNDEF;
    out_color_disparity.metadata.pixelModel = asn1Sccpix_DISP;
    out_color_disparity.metadata.errValues.arr[0].type = asn1Sccerror_UNDEFINED;
    out_color_disparity.metadata.errValues.arr[0].value = -16.0;
    out_color_disparity.metadata.errValues.nCount = 1;

    double min_color_disp, max_color_disp;
    cv::minMaxLoc(filteredDisparityColor, &min_color_disp, &max_color_disp);
    out_color_disparity.metadata.pixelCoeffs.arr[0] = 16.0;
    out_color_disparity.metadata.pixelCoeffs.arr[1] = 0.0;
    out_color_disparity.metadata.pixelCoeffs.arr[2] = in_frame_pair.baseline;
    out_color_disparity.metadata.pixelCoeffs.arr[3] = max_color_disp;
    out_color_disparity.metadata.pixelCoeffs.arr[4] = min_color_disp;

    out_color_disparity.data.msgVersion = array3D_Version;
    out_color_disparity.data.channels = static_cast<asn1SccT_UInt32>(filteredDisparityColor.channels());
    out_color_disparity.data.rows = static_cast<asn1SccT_UInt32>(filteredDisparityColor.rows);
    out_color_disparity.data.cols = static_cast<asn1SccT_UInt32>(filteredDisparityColor.cols);
    out_color_disparity.data.depth = static_cast<asn1SccArray3D_depth_t>(filteredDisparityColor.depth());
    out_color_disparity.data.rowSize = filteredDisparityColor.step[0];
    out_color_disparity.data.data.nCount =  static_cast<int>(out_color_disparity.data.rows * out_color_disparity.data.rowSize);
    memcpy(out_color_disparity.data.data.arr, filteredDisparityColor.data, static_cast<size_t>(out_color_disparity.data.data.nCount));


    // Get the number of paired pixels and calculate percentage
    double nb_paired = 0;
    double percentage;
    for (int x = 0; x<disparity.rows; x++)
    {
        for (int y = 0; y<disparity.cols; y++)
        {
            // Accesssing values of each pixel
            if ((disparity.at<int16_t>(x, y)/16) > 0)
                nb_paired += 1;
        }
    }

    percentage = (nb_paired/(disparity.rows * disparity.cols))*100;

    metadata_values.push_back(std::to_string(in_frame_pair.left.metadata.timeStamp.microseconds));
    metadata_values.push_back(std::to_string(in_frame_pair.right.metadata.timeStamp.microseconds));
    metadata_values.push_back(std::to_string(static_cast<int>(nb_paired)));
    metadata_values.push_back(std::to_string(percentage));

}

void ImagePairMatcher::ProcessStereoRectification(asn1SccFramePair& in_original_stereo_pair, asn1SccFramePair& out_rectified_stereo_pair, cv::Mat & out_rect_left, cv::Mat & out_rect_right){
    cv::Mat in_left(static_cast<int>(in_original_stereo_pair.left.data.rows), static_cast<int>(in_original_stereo_pair.left.data.cols), CV_MAKETYPE(static_cast<int>(in_original_stereo_pair.left.data.depth), static_cast<int>(in_original_stereo_pair.left.data.channels)), in_original_stereo_pair.left.data.data.arr, in_original_stereo_pair.left.data.rowSize);
    cv::Mat in_right(static_cast<int>(in_original_stereo_pair.right.data.rows), static_cast<int>(in_original_stereo_pair.right.data.cols), CV_MAKETYPE(static_cast<int>(in_original_stereo_pair.right.data.depth), static_cast<int>(in_original_stereo_pair.right.data.channels)), in_original_stereo_pair.right.data.data.arr, in_original_stereo_pair.right.data.rowSize);

    // Generate correction maps if needed
    if( std::string(reinterpret_cast<char const *>(in_original_stereo_pair.left.intrinsic.sensorId.arr)) != _sensor_id_left ||
            std::string(reinterpret_cast<char const *>(in_original_stereo_pair.right.intrinsic.sensorId.arr)) != _sensor_id_right ||
            rect_parameters_.calibration_file_path != _calibration_file_path ||
            rect_parameters_.xratio != _xratio ||
            rect_parameters_.yratio != _yratio ||
            rect_parameters_.scaling != _scaling){

        _sensor_id_left = std::string(reinterpret_cast<char const *>(in_original_stereo_pair.left.intrinsic.sensorId.arr));
        _sensor_id_right = std::string(reinterpret_cast<char const *>(in_original_stereo_pair.right.intrinsic.sensorId.arr));
        _calibration_file_path = rect_parameters_.calibration_file_path;
        _xratio = rect_parameters_.xratio;
        _yratio = rect_parameters_.yratio;
        _scaling = rect_parameters_.scaling;

        cv::FileStorage fs( _calibration_file_path + "/" + _sensor_id_left + std::string("-") + _sensor_id_right + ".yml", cv::FileStorage::READ );
        if( fs.isOpened() ){
            cv::Size image_size;
            cv::Mat1d camera_matrix_L;
            cv::Mat1d camera_matrix_R;
            cv::Mat1d dist_coeffs_L;
            cv::Mat1d dist_coeffs_R;
            cv::Mat1d R;
            cv::Mat1d T;

            fs["image_width"]  >> image_size.width;
            fs["image_height"] >> image_size.height;

            fs["camera_matrix_1"] >> camera_matrix_L;
            fs["distortion_coefficients_1"] >> dist_coeffs_L;

            fs["camera_matrix_2"] >> camera_matrix_R;
            fs["distortion_coefficients_2"] >> dist_coeffs_R;

            fs["rotation_matrix"] >> R;
            fs["translation_coefficients"] >> T;

            cv::Size newSize;
            newSize.width = image_size.width / _xratio;
            newSize.height = image_size.height / _yratio;

            cv::Mat1d RLeft;
            cv::Mat1d RRight;
            cv::Mat1d Q;

            cv::stereoRectify(camera_matrix_L, dist_coeffs_L, camera_matrix_R, dist_coeffs_R, image_size, R, T, RLeft, RRight, _PLeft, _PRight, Q, CV_CALIB_ZERO_DISPARITY, _scaling, newSize);

            cv::initUndistortRectifyMap(camera_matrix_L, dist_coeffs_L, RLeft, _PLeft, newSize, CV_32F, _lmapx, _lmapy);
            cv::initUndistortRectifyMap(camera_matrix_R, dist_coeffs_R, RRight, _PRight, newSize, CV_32F, _rmapx, _rmapy);

            _baseline = 1.0 / Q.at<double>(3,2);

            _initialized = true;
        }
        else{
            _initialized = false;
            std::cerr << "Can't open the calibration file: " << _calibration_file_path + "/" + _sensor_id_left + std::string("-") + _sensor_id_right + ".yml" << std::endl;
        }
    }

    if( _initialized ){
        cv::remap(in_left, out_rect_left, _lmapx, _lmapy, cv::INTER_LINEAR);
        cv::remap(in_right, out_rect_right, _rmapx, _rmapy, cv::INTER_LINEAR);

        // Getting image pair
        out_rectified_stereo_pair.msgVersion = frame_Version;
        out_rectified_stereo_pair.baseline = _baseline;

        // Left image
        {
            asn1SccFrame & img = out_rectified_stereo_pair.left;

            // init the structure
            img.msgVersion = frame_Version;

            img.intrinsic = in_original_stereo_pair.left.intrinsic;

            Eigen::Map<Eigen::Matrix3d>(img.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(_PLeft.data), 3, 3, Eigen::OuterStride<>(4));

            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(img.intrinsic.distCoeffs.arr, in_original_stereo_pair.left.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(in_original_stereo_pair.left.intrinsic.distCoeffs.nCount, 1);
            img.intrinsic.distCoeffs.nCount = in_original_stereo_pair.left.intrinsic.distCoeffs.nCount;

            img.intrinsic.cameraModel = asn1Scccam_PINHOLE;

            img.extrinsic = in_original_stereo_pair.left.extrinsic;
            img.metadata = in_original_stereo_pair.left.metadata;

            // Array3D
            {
                img.data.msgVersion = array3D_Version;
                img.data.rows = static_cast<asn1SccT_UInt32>(out_rect_left.rows);
                img.data.cols = static_cast<asn1SccT_UInt32>(out_rect_left.cols);
                img.data.channels = static_cast<asn1SccT_UInt32>(out_rect_left.channels());
                img.data.depth = static_cast<asn1SccArray3D_depth_t>(out_rect_left.depth());
                img.data.rowSize = static_cast<asn1SccT_UInt32>(out_rect_left.step[0]);
                img.data.data.nCount = static_cast<int>(img.data.rows * img.data.rowSize);
                memcpy(img.data.data.arr, out_rect_left.data, static_cast<size_t>(img.data.data.nCount));
            }
        }

        // Right image
        {
            asn1SccFrame & img = out_rectified_stereo_pair.right;

            // init the structure
            img.msgVersion = frame_Version;

            img.intrinsic = in_original_stereo_pair.right.intrinsic;

            Eigen::Map<Eigen::Matrix3d>(img.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(_PRight.data), 3, 3, Eigen::OuterStride<>(4));

            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(img.intrinsic.distCoeffs.arr, in_original_stereo_pair.right.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(in_original_stereo_pair.right.intrinsic.distCoeffs.nCount, 1);
            img.intrinsic.distCoeffs.nCount = in_original_stereo_pair.right.intrinsic.distCoeffs.nCount;

            img.intrinsic.cameraModel = asn1Scccam_PINHOLE;

            img.extrinsic = in_original_stereo_pair.right.extrinsic;
            img.metadata = in_original_stereo_pair.right.metadata;

            // Array3D
            {
                img.data.msgVersion = array3D_Version;
                img.data.rows = static_cast<asn1SccT_UInt32>(out_rect_right.rows);
                img.data.cols = static_cast<asn1SccT_UInt32>(out_rect_right.cols);
                img.data.channels = static_cast<asn1SccT_UInt32>(out_rect_right.channels());
                img.data.depth = static_cast<asn1SccArray3D_depth_t>(out_rect_right.depth());
                img.data.rowSize = static_cast<asn1SccT_UInt32>(out_rect_right.step[0]);
                img.data.data.nCount = static_cast<int>(img.data.rows * img.data.rowSize);
                memcpy(img.data.data.arr, out_rect_right.data, static_cast<size_t>(img.data.data.nCount));
            }
        }
    }
}

} // infuse_debug_tools

