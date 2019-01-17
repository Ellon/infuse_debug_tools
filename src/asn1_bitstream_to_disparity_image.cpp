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
        outRawDisparity_ptr_{std::make_unique<asn1SccFrame>()}
    {
        // Set the differents parameters
        private_nh_.param("algorithm", parameters.stereoMatcher.algorithm, int(1));
        private_nh_.param("minDisparity", parameters.stereoMatcher.minDisparity, int(0));
        private_nh_.param("numDisparities", parameters.stereoMatcher.numDisparities, int(192));
        private_nh_.param("blockSize", parameters.stereoMatcher.blockSize, int(5));
        private_nh_.param("speckleWindowSize", parameters.stereoMatcher.speckleWindowSize, int(0));
        private_nh_.param("speckleRange", parameters.stereoMatcher.speckleRange, int(0));
        private_nh_.param("disp12MaxDiff", parameters.stereoMatcher.disp12MaxDiff, int(-1));
        private_nh_.param("preFilterCap", parameters.stereoMatcher.preFilterCap, int(31));
        private_nh_.param("uniquenessRatio", parameters.stereoMatcher.uniquenessRatio, int(15));

        private_nh_.param("preFilterType", parameters.stereoMatcher.bmParams.preFilterType, int(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE));
        private_nh_.param("preFilterSize", parameters.stereoMatcher.bmParams.preFilterSize, int(9));
        private_nh_.param("textureThreshold", parameters.stereoMatcher.bmParams.textureThreshold, int(10));

        private_nh_.param("P1", parameters.stereoMatcher.sgbmParams.P1, int(0));
        private_nh_.param("P2", parameters.stereoMatcher.sgbmParams.P2, int(0));
        private_nh_.param("mode", parameters.stereoMatcher.sgbmParams.mode, int(cv::StereoSGBM::MODE_SGBM));

        #if WITH_XIMGPROC
            private_nh_.param("useFilter", parameters.filter.useFilter, bool(false));
            private_nh_.param("useConfidence", parameters.filter.useConfidence, bool(false));
            private_nh_.param("depthDiscontinuityRadius", parameters.filter.depthDiscontinuityRadius, int(0));
            private_nh_.param("lambda", parameters.filter.lambda, double(8000.0));
            private_nh_.param("lrcThresh", parameters.filter.lrcThresh, int(24));
            private_nh_.param("sigmaColor", parameters.filter.sigmaColor, double(1.5));
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
        process_stereo_matching(*asn1_image_ptr_, *outRawDisparity_ptr_);

        // Convert to sensor_msgs/Image
        static sensor_msgs::Image msg_image_disparity;
        fromASN1SCCToRosImage(*outRawDisparity_ptr_, msg_image_disparity);
        pub_disparity.publish(msg_image_disparity);
    }

    void StereoMatching::process_stereo_matching(asn1SccFramePair& inFramePair, asn1SccFrame& outRawDisparity)
    {
        cv::Mat imgLeft(static_cast<int>(inFramePair.left.data.rows), static_cast<int>(inFramePair.left.data.cols), CV_MAKETYPE(static_cast<int>(inFramePair.left.data.depth), static_cast<int>(inFramePair.left.data.channels)), inFramePair.left.data.data.arr, inFramePair.left.data.rowSize);
        cv::Mat imgRight(static_cast<int>(inFramePair.right.data.rows), static_cast<int>(inFramePair.right.data.cols), CV_MAKETYPE(static_cast<int>(inFramePair.right.data.depth), static_cast<int>(inFramePair.right.data.channels)), inFramePair.right.data.data.arr, inFramePair.right.data.rowSize);
        cv::Mat disparity, disparity8;

        // Using Algorithm StereoBM
        if(parameters.stereoMatcher.algorithm == 0)
        {
            if(_bm.empty())
            {
                _bm = cv::StereoBM::create(parameters.stereoMatcher.numDisparities, parameters.stereoMatcher.blockSize);
            }

            _bm->setBlockSize(parameters.stereoMatcher.blockSize);
            _bm->setDisp12MaxDiff(parameters.stereoMatcher.disp12MaxDiff);
            _bm->setMinDisparity(parameters.stereoMatcher.minDisparity);
            _bm->setNumDisparities(parameters.stereoMatcher.numDisparities);
            _bm->setPreFilterCap(parameters.stereoMatcher.preFilterCap);
            _bm->setPreFilterSize(parameters.stereoMatcher.bmParams.preFilterSize);
            _bm->setPreFilterType(parameters.stereoMatcher.bmParams.preFilterType);
            _bm->setSpeckleRange(parameters.stereoMatcher.speckleRange);
            _bm->setSpeckleWindowSize(parameters.stereoMatcher.speckleWindowSize);
            _bm->setTextureThreshold(parameters.stereoMatcher.bmParams.textureThreshold);
            _bm->setUniquenessRatio(parameters.stereoMatcher.uniquenessRatio);

            _bm->compute(imgLeft, imgRight, disparity);



        }
        // Using Algorithm StereoSGBM
        else if(parameters.stereoMatcher.algorithm == 1)
        {
            if(_sgbm.empty())
            {
                _sgbm = cv::StereoSGBM::create(parameters.stereoMatcher.minDisparity, parameters.stereoMatcher.numDisparities, parameters.stereoMatcher.blockSize, parameters.stereoMatcher.sgbmParams.P1, parameters.stereoMatcher.sgbmParams.P2, parameters.stereoMatcher.disp12MaxDiff, parameters.stereoMatcher.preFilterCap, parameters.stereoMatcher.uniquenessRatio, parameters.stereoMatcher.speckleWindowSize, parameters.stereoMatcher.speckleRange, parameters.stereoMatcher.sgbmParams.mode);
            }

            _sgbm->setBlockSize(parameters.stereoMatcher.blockSize);
            _sgbm->setDisp12MaxDiff(parameters.stereoMatcher.disp12MaxDiff);
            _sgbm->setMinDisparity(parameters.stereoMatcher.minDisparity);
            _sgbm->setMode(parameters.stereoMatcher.sgbmParams.mode);
            _sgbm->setNumDisparities(parameters.stereoMatcher.numDisparities);
            _sgbm->setP1(parameters.stereoMatcher.sgbmParams.P1);
            _sgbm->setP2(parameters.stereoMatcher.sgbmParams.P2);
            _sgbm->setPreFilterCap(parameters.stereoMatcher.preFilterCap);
            _sgbm->setSpeckleRange(parameters.stereoMatcher.speckleRange);
            _sgbm->setSpeckleWindowSize(parameters.stereoMatcher.speckleWindowSize);
            _sgbm->setUniquenessRatio(parameters.stereoMatcher.uniquenessRatio);

            _sgbm->compute(imgLeft, imgRight, disparity);

        }

    #if WITH_XIMGPROC
        bool resetFilter = false;
        bool resetMatcher = false;
        if(parameters.stereoMatcher.algorithm != _algorithm)
        {
            _algorithm = parameters.stereoMatcher.algorithm;
            resetFilter = true;
            resetMatcher = true;
        }
        else if(parameters.filter.useConfidence != _useConfidence)
        {
            _useConfidence = parameters.filter.useConfidence;
            resetFilter = true;
        }

        if(parameters.filter.useFilter)
        {
            cv::Mat disparityFiltered;

            if(parameters.filter.useConfidence
                    ){
                cv::Mat disparityRight;
                if(_rightMatcher.empty() || resetMatcher)
                {
                    switch(_algorithm)
                    {
                        case 0:
                            _rightMatcher = cv::ximgproc::createRightMatcher(_bm);
                            break;
                        case 1:
                            _rightMatcher = cv::ximgproc::createRightMatcher(_sgbm);
                            break;
                    }
                }

                _rightMatcher->compute(imgRight, imgLeft, disparityRight);

                if(_filter.empty() || resetFilter)
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

                _filter->setDepthDiscontinuityRadius(parameters.filter.depthDiscontinuityRadius);
                _filter->setLambda(parameters.filter.lambda);
                _filter->setLRCthresh(parameters.filter.lrcThresh);
                _filter->setSigmaColor(parameters.filter.sigmaColor);

                _filter->filter(disparity, imgLeft, disparityFiltered, disparityRight);
            }
            else
            {
                if(_filter.empty() || resetFilter)
                {
                    _filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
                }

                _filter->setDepthDiscontinuityRadius(parameters.filter.depthDiscontinuityRadius);
                _filter->setLambda(parameters.filter.lambda);
                _filter->setSigmaColor(parameters.filter.sigmaColor);

                _filter->filter(disparity, imgLeft, disparityFiltered);
            }

            disparity = disparityFiltered;
        }
    #endif

        // Normalize and remap the disparity image
        normalize(disparity, disparity8, 0, 255, CV_MINMAX, CV_8U);

        // Convert Mat to ASN.1
        outRawDisparity.metadata.msgVersion = frame_Version;
        outRawDisparity.metadata = inFramePair.left.metadata;
        outRawDisparity.intrinsic = inFramePair.left.intrinsic;
        outRawDisparity.extrinsic = inFramePair.left.extrinsic;

        outRawDisparity.metadata.mode = asn1Sccmode_UNDEF;
        outRawDisparity.metadata.pixelModel = asn1Sccpix_DISP;
        outRawDisparity.metadata.errValues.arr[0].type = asn1Sccerror_UNDEFINED;
        outRawDisparity.metadata.errValues.arr[0].value = -16.0;
        outRawDisparity.metadata.errValues.nCount = 1;

        double minDisp, maxDisp;
        cv::minMaxLoc(disparity8, &minDisp, &maxDisp);
        outRawDisparity.metadata.pixelCoeffs.arr[0] = 16.0;
        outRawDisparity.metadata.pixelCoeffs.arr[1] = 0.0;
        outRawDisparity.metadata.pixelCoeffs.arr[2] = inFramePair.baseline;
        outRawDisparity.metadata.pixelCoeffs.arr[3] = maxDisp;
        outRawDisparity.metadata.pixelCoeffs.arr[4] = minDisp;

        outRawDisparity.data.msgVersion = array3D_Version;
        outRawDisparity.data.channels = static_cast<asn1SccT_UInt32>(disparity8.channels());
        outRawDisparity.data.rows = static_cast<asn1SccT_UInt32>(inFramePair.left.data.rows);
        outRawDisparity.data.cols = static_cast<asn1SccT_UInt32>(inFramePair.left.data.cols);
        outRawDisparity.data.depth = static_cast<asn1SccArray3D_depth_t>(disparity8.depth());
        outRawDisparity.data.rowSize = disparity8.step[0];
        outRawDisparity.data.data.nCount =  static_cast<int>(outRawDisparity.data.rows * outRawDisparity.data.rowSize);
        memcpy(outRawDisparity.data.data.arr, disparity8.data, static_cast<size_t>(outRawDisparity.data.data.nCount));

    }

} // namespace infuse_debug_tools

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_matching");

    infuse_debug_tools::StereoMatching stereo_match;

    ros::spin();

    return 0;
}

