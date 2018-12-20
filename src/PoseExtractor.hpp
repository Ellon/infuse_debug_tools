#ifndef __POSE_EXTRACTOR_HPP__
#define __POSE_EXTRACTOR_HPP__

#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/TransformWithCovariance.h>

#include <boost/filesystem.hpp>

namespace infuse_debug_tools {

class PoseExtractor{
    public:
    PoseExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::vector<std::string> &pose_topics, const std::string &output_name);
    void Extract();

    private:
    void ProcessPose(const infuse_msgs::asn1_bitstream::Ptr& msg);
    std::pair<std::string, size_t> GetMainTopicInfo();

    private:
    boost::filesystem::path output_dir_;
    std::vector<std::string> bag_paths_;
    std::vector<std::string> pose_topics_;
    std::string output_name_;
    std::unique_ptr<asn1SccTransformWithCovariance> asn1_pose_ptr_;
    std::ofstream data_ofs_;
};

}
#endif
