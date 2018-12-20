#include "PoseExtractor.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>

#include <boost/progress.hpp>
#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools
{

    PoseExtractor::PoseExtractor(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::vector<std::string> &pose_topics, const std::string &output_name)
    : output_dir_{output_dir},
      bag_paths_{bag_paths},
      pose_topics_{pose_topics},
      output_name_{output_name},
      asn1_pose_ptr_{std::make_unique<asn1SccTransformWithCovariance>()}
    {}

    void PoseExtractor::Extract()
    {
        // Get the number of poses to process
        std::string main_topic;
        size_t n_poses;
        std::tie(main_topic, n_poses) = GetMainTopicInfo();

        // Stop here if there's nothing on the topic
        if (n_poses == 0)
        {
            std::cout << "Warning: Nothing to extract on topic" << (pose_topics_.size()>1? "s " : " ");
            for (const auto & topic : pose_topics_)
                std::cout << topic << " ";
            std::cout << std::endl;
            return;
        }

        // Makes sure the output dir does not already exists
        if (bfs::exists(output_dir_)) 
        {
            std::stringstream ss;
            if (bfs::is_directory(output_dir_))
                ss << "A directory named \"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory to output the poses.";
            else if (bfs::is_regular_file(output_dir_))
                ss << "A regular file named \"" << output_dir_.string() << "\" already exists. Please remove this file or choose another directory name to output the poses.";
            else
                ss << "\"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory name to output the poses.";
            throw std::runtime_error(ss.str());
        }

        // Lambda function that creates a directory (or subdir inside dir if specified).
        auto lambda_create_subdir = [](bfs::path dir, std::string subdir = "") -> bfs::path
        {
            bfs::path dirpath = dir / subdir;
            bool dir_created = bfs::create_directory(dirpath);
            if (not dir_created)
            {
                std::stringstream ss;
                ss << "Could not create \"" << dirpath.string() << "\" directory.";
                throw std::runtime_error(ss.str());
            }
            return dirpath;
        };
        // Create output dir
        lambda_create_subdir(output_dir_);

        // Write dataformat file. The rationalle of keeping the dataformat separated
        // from the metadata is that this way it is possible to associate the cloud
        // number with the line in the metadata file.
        std::ofstream dataformat_ofs((output_dir_ / "dataformat.txt").string());
        std::vector<std::string> entries{ASN1BitstreamLogger::GetTransformWithCovarianceLogEntries()};
        unsigned int index = 1;
        for (auto entry : entries)
        {
            dataformat_ofs << "# " << std::setw(2) << index << " - " << entry << '\n';
            index++;
        }
        dataformat_ofs.close();

        // Setup output data file
        bfs::path output_filename = (output_dir_ / output_name_).replace_extension(".txt");
        data_ofs_.open(output_filename.string());

        // Setup progress display
        std::cout << "Extracting " << n_poses << " poses to " << output_filename.string();
        boost::progress_display show_progress(n_poses);
        
        // Vector used to create a view on the bag with the main topic
        std::vector<std::string> topics = {main_topic};

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
                // Assure files are closed if something goes wrong and re-trhow
                bag.close();
                data_ofs_.close();
                throw;
            }
            bag.close();
        } // for bags

        data_ofs_.close();
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

        ASN1BitstreamLogger::LogTransformWithCovariance(*asn1_pose_ptr_, data_ofs_);
        data_ofs_ << '\n';
    }

    std::pair<std::string, size_t> PoseExtractor::GetMainTopicInfo()
    {
        size_t n_poses = 0;
        size_t main_idx = 0;
        size_t idx = 0;
        for (const auto & topic : pose_topics_)
        {
            // Count all messages on a given topic
            size_t n_poses_in_topic = 0;
            for (auto bag_path : bag_paths_)
            {
                rosbag::Bag bag(bag_path);
                rosbag::View view(bag, rosbag::TopicQuery({topic}));
                n_poses_in_topic += view.size();
                bag.close();
            }

            // Allow only one topic to have pose messages.
            if (n_poses_in_topic > 0 and n_poses == 0)
            {
                n_poses = n_poses_in_topic;
                main_idx = idx;
            }
            else if (n_poses_in_topic > 0 and n_poses > 0)
            {
                std::stringstream ss;
                ss << "Found multiple topics with pose data when processing " << output_name_ << " data";
                throw std::runtime_error(ss.str());
            }

            idx++;
        }

        return std::make_pair(pose_topics_[main_idx], n_poses);
    }

}
