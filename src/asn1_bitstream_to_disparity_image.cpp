#include "asn1_bitstream_to_disparity_image.hpp"


void fromASN1SCCToRosImage(const asn1SccFrame& image, sensor_msgs::Image& msg_image){

    // Fill the sensor_msgs::Image data struct with asn1SccFrame data
    msg_image.header.stamp.fromNSec((uint64_t)image.metadata.timeStamp.microseconds * 1000ull);

    msg_image.width = image.data.cols;
    msg_image.height = image.data.rows;

    msg_image.encoding = "mono8";
    msg_image.is_bigendian = true; //or false?
    msg_image.step = msg_image.width;
    msg_image.data.resize(image.data.data.nCount);

    int i = 0;
    while(i < msg_image.data.size())
    {
        msg_image.data[i] = image.data.data.arr[i];
        i++;
    }
}

namespace infuse_debug_tools {

    StereoMatching::StereoMatching() :
        private_nh_{"~"},
        connect_image_srv_{private_nh_.advertiseService("connect_image", &StereoMatching::connect_image, this)},
        asn1_image_ptr_{std::make_unique<asn1SccFramePair>()},
        out_raw_disparity_ptr_{std::make_unique<asn1SccFrame>()}
    {
        // Set the differents parameters
        private_nh_.param("algorithm", parameters.stereo_matcher.algorithm, int(1));
        private_nh_.param("min_disparity", parameters.stereo_matcher.min_disparity, int(0));
        private_nh_.param("num_disparities", parameters.stereo_matcher.num_disparities, int(192));
        private_nh_.param("block_size", parameters.stereo_matcher.block_size, int(5));
        private_nh_.param("speckle_window_size", parameters.stereo_matcher.speckle_window_size, int(0));
        private_nh_.param("speckle_range", parameters.stereo_matcher.speckle_range, int(0));
        private_nh_.param("disp12_max_diff", parameters.stereo_matcher.disp12_max_diff, int(-1));
        private_nh_.param("pre_filter_cap", parameters.stereo_matcher.pre_filter_cap, int(31));
        private_nh_.param("uniqueness_ratio", parameters.stereo_matcher.uniqueness_ratio, int(15));

        private_nh_.param("pre_filter_type", parameters.stereo_matcher.bm_params.pre_filter_type, int(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE));
        private_nh_.param("pre_filter_size", parameters.stereo_matcher.bm_params.pre_filter_size, int(9));
        private_nh_.param("texture_threshold", parameters.stereo_matcher.bm_params.texture_threshold, int(10));

        private_nh_.param("P1", parameters.stereo_matcher.sgbm_params.P1, int(0));
        private_nh_.param("P2", parameters.stereo_matcher.sgbm_params.P2, int(0));
        private_nh_.param("mode", parameters.stereo_matcher.sgbm_params.mode, int(cv::StereoSGBM::MODE_SGBM));

        #if WITH_XIMGPROC
            private_nh_.param("use_filter", parameters.filter.use_filter, bool(false));
            private_nh_.param("use_confidence", parameters.filter.use_confidence, bool(false));
            private_nh_.param("depth_discontinuity_radius", parameters.filter.depth_discontinuity_radius, int(0));
            private_nh_.param("lambda", parameters.filter.lambda, double(8000.0));
            private_nh_.param("lrc_thresh", parameters.filter.lrc_thresh, int(24));
            private_nh_.param("sigma_color", parameters.filter.sigma_color, double(1.5));
        #endif

        {
            // Get topic name to connect
            std::string topics_to_connect;
            if (private_nh_.getParam("topics_to_connect", topics_to_connect)) {

                // Split strings into a vector
                std::vector<std::string> topics;
                {
                    std::stringstream ss(topics_to_connect);
                    std::istream_iterator<std::string> begin(ss), eos; // end-of-stream
                    topics.assign(begin, eos);
                }

                // Connect to topics
                for (const auto & topic : topics) {
                    try {
                        bind_subToPubs(topic);
                    } catch (...) {
                        ROS_INFO_STREAM("ERROR: Could not connect to topic " << topic);
                    }
                }
            }
        }
    }

    bool StereoMatching::connect_image(infuse_debug_tools::ConnectTopic::Request  &req,
                                       infuse_debug_tools::ConnectTopic::Response &res)
    {
        try
        {
            // Create subscriber and publishers
            bind_subToPubs(req.topic);
            res.success = true;

            return true;
        }
        catch (...)
        {
            std::stringstream ss;
            ss << "Unknown exception when subscribing to topic " << req.topic;
            ROS_INFO_STREAM(ss.str());
            res.success = false;
            res.message = ss.str();

            return false;
        }
    }

    void StereoMatching::bind_subToPubs(std::string topic_in){

        std::string output_topic_disparity = topic_in + "_rosDisparity";

        // Create publisher for disparity image
        pub_map_[topic_in] = nh_.advertise<sensor_msgs::Image>(output_topic_disparity, 100);

        // Bind publisher to the callback
        boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback =
                boost::bind(&StereoMatching::image_callback, this, _1, boost::cref(pub_map_[topic_in]));

        // Create subscriber
        sub_map_[topic_in] = nh_.subscribe<sensor_msgs::Image>(topic_in, 100, callback);
        ROS_INFO_STREAM("Connected to topic " << topic_in << ". Publishing disparity image on " << output_topic_disparity);

    }

    void StereoMatching::image_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub_disparity)
    {
        // Get time at the begining of the callback -- unused now...
        auto cb_time = ros::Time::now();

        // Initialize
        asn1SccFramePair_Initialize(asn1_image_ptr_.get());

        // Decode
        flag res;
        int errorCode;
        BitStream bstream;
        BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
        res = asn1SccFramePair_Decode(asn1_image_ptr_.get(), &bstream, &errorCode);
        if (not res)
        {
            ROS_INFO("Error decoding asn1Image! Error: %d", errorCode);
            return;
        }

        // Process stereo matching
        process_stereo_matching(*asn1_image_ptr_, *out_raw_disparity_ptr_);

        // Convert to sensor_msgs/Image
        static sensor_msgs::Image msg_image_disparity;
        fromASN1SCCToRosImage(*out_raw_disparity_ptr_, msg_image_disparity);
        pub_disparity.publish(msg_image_disparity);
    }

    void StereoMatching::process_stereo_matching(asn1SccFramePair& in_frame_pair, asn1SccFrame& out_raw_disparity)
    {
        cv::Mat img_left(static_cast<int>(in_frame_pair.left.data.rows), static_cast<int>(in_frame_pair.left.data.cols), CV_MAKETYPE(static_cast<int>(in_frame_pair.left.data.depth), static_cast<int>(in_frame_pair.left.data.channels)), in_frame_pair.left.data.data.arr, in_frame_pair.left.data.rowSize);
        cv::Mat img_right(static_cast<int>(in_frame_pair.right.data.rows), static_cast<int>(in_frame_pair.right.data.cols), CV_MAKETYPE(static_cast<int>(in_frame_pair.right.data.depth), static_cast<int>(in_frame_pair.right.data.channels)), in_frame_pair.right.data.data.arr, in_frame_pair.right.data.rowSize);
        cv::Mat disparity, disparity8;

        // Using Algorithm StereoBM
        if(parameters.stereo_matcher.algorithm == 0)
        {
            if(_bm.empty())
            {
                _bm = cv::StereoBM::create(parameters.stereo_matcher.num_disparities, parameters.stereo_matcher.block_size);
            }

            _bm->setBlockSize(parameters.stereo_matcher.block_size);
            _bm->setDisp12MaxDiff(parameters.stereo_matcher.disp12_max_diff);
            _bm->setMinDisparity(parameters.stereo_matcher.min_disparity);
            _bm->setNumDisparities(parameters.stereo_matcher.num_disparities);
            _bm->setPreFilterCap(parameters.stereo_matcher.pre_filter_cap);
            _bm->setPreFilterSize(parameters.stereo_matcher.bm_params.pre_filter_size);
            _bm->setPreFilterType(parameters.stereo_matcher.bm_params.pre_filter_type);
            _bm->setSpeckleRange(parameters.stereo_matcher.speckle_range);
            _bm->setSpeckleWindowSize(parameters.stereo_matcher.speckle_window_size);
            _bm->setTextureThreshold(parameters.stereo_matcher.bm_params.texture_threshold);
            _bm->setUniquenessRatio(parameters.stereo_matcher.uniqueness_ratio);

            _bm->compute(img_left, img_right, disparity);



        }
        // Using Algorithm StereoSGBM
        else if(parameters.stereo_matcher.algorithm == 1)
        {
            if(_sgbm.empty())
            {
                _sgbm = cv::StereoSGBM::create(parameters.stereo_matcher.min_disparity, parameters.stereo_matcher.num_disparities, parameters.stereo_matcher.block_size, parameters.stereo_matcher.sgbm_params.P1, parameters.stereo_matcher.sgbm_params.P2, parameters.stereo_matcher.disp12_max_diff, parameters.stereo_matcher.pre_filter_cap, parameters.stereo_matcher.uniqueness_ratio, parameters.stereo_matcher.speckle_window_size, parameters.stereo_matcher.speckle_range, parameters.stereo_matcher.sgbm_params.mode);
            }

            _sgbm->setBlockSize(parameters.stereo_matcher.block_size);
            _sgbm->setDisp12MaxDiff(parameters.stereo_matcher.disp12_max_diff);
            _sgbm->setMinDisparity(parameters.stereo_matcher.min_disparity);
            _sgbm->setMode(parameters.stereo_matcher.sgbm_params.mode);
            _sgbm->setNumDisparities(parameters.stereo_matcher.num_disparities);
            _sgbm->setP1(parameters.stereo_matcher.sgbm_params.P1);
            _sgbm->setP2(parameters.stereo_matcher.sgbm_params.P2);
            _sgbm->setPreFilterCap(parameters.stereo_matcher.pre_filter_cap);
            _sgbm->setSpeckleRange(parameters.stereo_matcher.speckle_range);
            _sgbm->setSpeckleWindowSize(parameters.stereo_matcher.speckle_window_size);
            _sgbm->setUniquenessRatio(parameters.stereo_matcher.uniqueness_ratio);

            _sgbm->compute(img_left, img_right, disparity);

        }

    #if WITH_XIMGPROC
        bool reset_filter = false;
        bool reset_matcher = false;
        if(parameters.stereo_matcher.algorithm != _algorithm)
        {
            _algorithm = parameters.stereo_matcher.algorithm;
            reset_filter = true;
            reset_matcher = true;
        }
        else if(parameters.filter.use_confidence != _use_confidence)
        {
            _use_confidence = parameters.filter.use_confidence;
            reset_filter = true;
        }

        if(parameters.filter.use_filter)
        {
            cv::Mat disparity_filtered;

            if(parameters.filter.use_confidence
                    ){
                cv::Mat disparityRight;
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

                _right_matcher->compute(img_right, img_left, disparityRight);

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

                _filter->setDepthDiscontinuityRadius(parameters.filter.depth_discontinuity_radius);
                _filter->setLambda(parameters.filter.lambda);
                _filter->setLRCthresh(parameters.filter.lrc_thresh);
                _filter->setSigmaColor(parameters.filter.sigma_color);

                _filter->filter(disparity, img_left, disparity_filtered, disparityRight);
            }
            else
            {
                if(_filter.empty() || reset_filter)
                {
                    _filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
                }

                _filter->setDepthDiscontinuityRadius(parameters.filter.depth_discontinuity_radius);
                _filter->setLambda(parameters.filter.lambda);
                _filter->setSigmaColor(parameters.filter.sigma_color);

                _filter->filter(disparity, img_left, disparity_filtered);
            }

            disparity = disparity_filtered;
        }
    #endif

        // Normalize and remap the disparity image
        normalize(disparity, disparity8, 0, 255, CV_MINMAX, CV_8U);

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

        double min_disp, maxDisp;
        cv::minMaxLoc(disparity8, &min_disp, &maxDisp);
        out_raw_disparity.metadata.pixelCoeffs.arr[0] = 16.0;
        out_raw_disparity.metadata.pixelCoeffs.arr[1] = 0.0;
        out_raw_disparity.metadata.pixelCoeffs.arr[2] = in_frame_pair.baseline;
        out_raw_disparity.metadata.pixelCoeffs.arr[3] = maxDisp;
        out_raw_disparity.metadata.pixelCoeffs.arr[4] = min_disp;

        out_raw_disparity.data.msgVersion = array3D_Version;
        out_raw_disparity.data.channels = static_cast<asn1SccT_UInt32>(disparity8.channels());
        out_raw_disparity.data.rows = static_cast<asn1SccT_UInt32>(in_frame_pair.left.data.rows);
        out_raw_disparity.data.cols = static_cast<asn1SccT_UInt32>(in_frame_pair.left.data.cols);
        out_raw_disparity.data.depth = static_cast<asn1SccArray3D_depth_t>(disparity8.depth());
        out_raw_disparity.data.rowSize = disparity8.step[0];
        out_raw_disparity.data.data.nCount =  static_cast<int>(out_raw_disparity.data.rows * out_raw_disparity.data.rowSize);
        memcpy(out_raw_disparity.data.data.arr, disparity8.data, static_cast<size_t>(out_raw_disparity.data.data.nCount));

    }

} // namespace infuse_debug_tools

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_matching");

    infuse_debug_tools::StereoMatching stereo_match;

    ros::spin();

    return 0;
}

