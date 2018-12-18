#include "PoseExtractor.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>

#include <boost/progress.hpp>
#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools
{

    PoseExtractor::PoseExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &pose_topic, const std::string &pose_source)
    : bag_paths_{bag_paths},
      pose_topic_{pose_topic},
      pose_source_{pose_source},
      asn1_pose_ptr_{std::make_unique<asn1SccTransformWithCovariance>()}
    {
        output_dir_ = output_dir + "/poses";
    }

    void PoseExtractor::Extract()
    {
      // Makes sure the output dir does not already exists
        if (bfs::exists(output_dir_)) 
        {
            std::stringstream ss;
            if (bfs::is_directory(output_dir_))
                ss << "A directory named \"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory to output the point clouds.";
            else if (bfs::is_regular_file(output_dir_))
                ss << "A regular file named \"" << output_dir_.string() << "\" already exists. Please remove this file or choose another directory name to output the point clouds.";
            else
                ss << "\"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory name to output the point clouds.";
            throw std::runtime_error(ss.str());
        }

        bool dir_created = bfs::create_directory(output_dir_);
        if (not dir_created)
        {
            std::stringstream ss;
            ss << "Could not create \"" << output_dir_.string() << "\" directory.";
            throw std::runtime_error(ss.str());
        }

        std::ofstream dataformat_ofs((output_dir_ / "dataformat.txt").string());

        std::vector<std::string> entries{ASN1BitstreamLogger::GetTransformWithCovarianceLogEntries()};
        unsigned int index = 1;
        for (auto entry : entries)
        {
            dataformat_ofs << "# " << std::setw(2) << index << " - " << entry << '\n';
            index++;
        }
        dataformat_ofs.close();

        std::string filename = pose_source_ + ".txt";
        metadata_ofs_.open((output_dir_ / filename).string());

        std::vector<std::string> topics = {pose_topic_};

        size_t n_poses = 0;

        for (auto bag_path : bag_paths_)
        {
            rosbag::Bag bag(bag_path);
            rosbag::View view(bag, rosbag::TopicQuery(topics));
            n_poses += view.size();
            bag.close();
        }
  
        // Setup progress display
        std::cout << "Extracting " << n_poses << " poses...";
        boost::progress_display show_progress( n_poses);
        
        // Loop over bags
        for (auto bag_path : bag_paths_) 
        {
            rosbag::Bag bag(bag_path); // bagmode::Read by default
            // Create a view of the bag with the selected topics only
            rosbag::View view(bag, rosbag::TopicQuery(topics));

            try 
            {
                // Loop over messages in each view
                for (rosbag::MessageInstance const m: view) 
                {
                    infuse_msgs::asn1_bitstream::Ptr i = m.instantiate<infuse_msgs::asn1_bitstream>();
                    if (i != nullptr) 
                    {
                        ProcessPose(i);
                        ++show_progress; // Update progress display
                    } 
                    else
                        throw std::runtime_error("Could not instantiate an infuse_msgs::asn1_bitstream message!");
                } // for msgs in view
            } catch (...) 
            {
                // Assure the bags are closed if something goes wrong and re-trhow
                bag.close();
                throw;
            }
            bag.close();
        } // for bags

        metadata_ofs_.close();
    }

    void PoseExtractor::ProcessPose(const infuse_msgs::asn1_bitstream::Ptr &msg)
    {
        // Initialize asn1 pose to be sure we have a clean object.
        asn1SccTransformWithCovariance_Initialize(asn1_pose_ptr_.get());

        // Decode
        flag res;
        int errorCode;
        BitStream bstream;
        BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
        res = asn1SccTransformWithCovariance_Decode(asn1_pose_ptr_.get(), &bstream, &errorCode);
        if (not res)
        {
            std::stringstream ss;
            ss << "Error decode asn1TransformWithCovariance! Error: " << errorCode << "\n";
            throw std::runtime_error(ss.str());
        }

        ASN1BitstreamLogger::LogTransformWithCovariance(*asn1_pose_ptr_,metadata_ofs_);
        metadata_ofs_ << '\n';
    }
}
