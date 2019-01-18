#ifndef ASN1BITSTREAMTODISPARITYIMAGE_H
#define ASN1BITSTREAMTODISPARITYIMAGE_H

#include <ros/ros.h>

#include <boost/filesystem.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#if WITH_XIMGPROC
#include <opencv2/ximgproc.hpp>
#endif

#include <infuse_debug_tools/ConnectTopic.h>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_conversions/asn1_ros_conversions.hpp>
#include <infuse_asn1_types/Frame.h>

#include <sensor_msgs/Image.h>

namespace infuse_debug_tools {
    class StereoMatching
    {
        public:

            /**
                Constructor
            */
            StereoMatching();

            /**
                Process stereo matching between two images and calculate the disparity image.

                @param image pair to compare
                @param output disparity image
            */
            void process_stereo_matching(asn1SccFramePair& in_frame_pair, asn1SccFrame& out_raw_disparity);

            /**
                Connect image topic

                @param request
                @param result
                @return true if success
            */
            bool connect_image(infuse_debug_tools::ConnectTopic::Request  &req,
                               infuse_debug_tools::ConnectTopic::Response &res);

            /**
                Callback function when images pair is received.
                Process stereo matching and publish disparity image and number paired pixels.

                @param asn1 data
                @param disparity image publisher
            */
            void image_callback (const infuse_msgs::asn1_bitstream::Ptr& msg, const ros::Publisher &pub_disparity);



            struct stereoBMParams
            {
                /**
                 * @brief Pre-Filter Type
                 * 0 = PREFILTER_NORMALIZED_RESPONSE
                 * 1 = PREFILTER_XSOBEL
                 */
                int pre_filter_type;

                /**
                 * @brief Size of the Pre-Filter
                 * It must be an odd number >= 5 & <= 255
                 */
                int pre_filter_size;

                /**
                 * @brief Texture Threshold
                 */
                int texture_threshold;
            };

            struct stereoSGBMParams
            {
                /**
                 * @brief The first parameter controlling the disparity smoothness.
                 */
                int P1;

                /**
                 * @brief The second parameter controlling the disparity smoothness.
                 * The larger the values are, the smoother the disparity is.
                 * P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels.
                 * P2 is the penalty on the disparity change by more than 1 between neighbor pixels.
                 * The algorithm requires P2 > P1 .
                 * See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown
                 * (like 8*number_of_image_channels*block_size*block_size
                 * and 32*number_of_image_channels*block_size*block_size , respectively)
                 */
                int P2;

                /**
                 * @brief Algorithm mode.
                 * 0 = MODE_SGBM
                 * 1 = MODE_HH
                 * 2 = MODE_SGBM_3WAY
                 * 3 = MODE_HH4
                 * Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm.
                 * It will consume O(W*H*num_disparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures.
                 */
                int mode;
            };

            struct stereoMatcherParams
            {
                /**
                 * @brief Algorithm to be used
                 * 0 = Block Matching (BM)
                 * 1 = Semi Global Block Matching (SGBM)
                 */
                int algorithm;

                /**
                 * @brief Minimum possible disparity value.
                 * Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
                 */
                int min_disparity;

                /**
                 * @brief Maximum disparity minus minimum disparity.
                 * The value is always greater than zero.
                 * In the current implementation, this parameter must be divisible by 16.
                 */
                int num_disparities;

                /**
                 * @brief Matched block size. It must be an odd number >=1 .
                 * Normally, it should be somewhere in the 3..11 range.
                 */
                int block_size;

                /**
                 * @brief Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
                 * Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
                 */
                int speckle_window_size;

                /**
                 * @brief Maximum disparity variation within each connected component.
                 * If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
                 * Normally, 1 or 2 is good enough.
                 */
                int speckle_range;

                /**
                 * @brief Maximum allowed difference (in integer pixel units) in the left-right disparity check.
                 * Set it to a non-positive value to disable the check.
                 */
                int disp12_max_diff;

                /**
                 * @brief Truncation value for the prefiltered image pixels.
                 * The algorithm first computes x-derivative at each pixel and clips its value by [-pre_filter_cap, pre_filter_cap] interval.
                 * The result values are passed to the Birchfield-Tomasi pixel cost function.
                 */
                int pre_filter_cap;

                /**
                 * @brief Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
                 * Normally, a value within the 5-15 range is good enough.
                 */
                int uniqueness_ratio;

                stereoBMParams bm_params;
                stereoSGBMParams sgbm_params;
            };

            struct filterParams
            {
                /**
                 * @brief Set to true to filter the disparity map
                 */
                bool use_filter;

                /**
                 * @brief Filtering with confidence requires two disparity maps (for the left and right views) and is approximately two times slower.
                 * However, quality is typically significantly better.
                 */
                bool use_confidence;

                /** @brief DepthDiscontinuityRadius is a parameter used in confidence computation.
                 * It defines the size of low-confidence regions around depth discontinuities.
                 */
                int depth_discontinuity_radius;

                /** @brief Lambda is a parameter defining the amount of regularization during filtering.
                 * Larger values force filtered disparity map edges to adhere more to source image edges.
                 * Typical value is 8000.
                 */
                double lambda;

                /** @brief LRCthresh is a threshold of disparity difference used in left-right-consistency check during
                 * confidence map computation. The default value of 24 (1.5 pixels) is virtually always good enough.
                 */
                int lrc_thresh;

                /** @brief SigmaColor is a parameter defining how sensitive the filtering process is to source image edges.
                 * Large values can lead to disparity leakage through low-contrast edges.
                 * Small values can make the filter too sensitive to noise and textures in the source image.
                 * Typical values range from 0.8 to 2.0.
                 */
                double sigma_color;
            };

            struct StereoMatchingParams
            {
                stereoMatcherParams stereo_matcher;
                #if WITH_XIMGPROC
                    filterParams filter;
                #endif
            };

            StereoMatchingParams parameters;

        private:
            // Node Handlers
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            // Connect topic service
            ros::ServiceServer connect_image_srv_;
            // Images pair subscriber
            std::map<std::string, ros::Subscriber> sub_map_;
            // Disparity image publisher
            std::map<std::string, ros::Publisher> pub_map_;

            cv::Ptr<cv::StereoBM> _bm;
            cv::Ptr<cv::StereoSGBM> _sgbm;
            #if WITH_XIMGPROC
                    int _algorithm;
                    bool _use_confidence;
                    cv::Ptr<cv::ximgproc::DisparityWLSFilter> _filter;
                    cv::Ptr<cv::StereoMatcher> _right_matcher;
            #endif

            // Input asn1 images pair
            std::unique_ptr<asn1SccFramePair> asn1_image_ptr_;
            void bind_subToPubs(std::string topic_in);

            // Output asn1 disparity image
            std::unique_ptr<asn1SccFrame> out_raw_disparity_ptr_;
    };
} // infuse_debug_tools

#endif // ASN1BITSTREAMTODISPARITYIMAGE_H
