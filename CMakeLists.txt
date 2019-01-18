cmake_minimum_required(VERSION 2.8.3)
project(infuse_debug_tools)

set (CMAKE_CXX_STANDARD 14)

## Set options
OPTION(WITH_XIMGPROC "Compilation avec XImgProc (module OpenCV contrib)" ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp infuse_msgs std_msgs message_generation tf2_ros rosbag_storage infuse_novatel_gps_msgs)

find_package(infuse_asn1_types REQUIRED)
find_package(infuse_asn1_conversions REQUIRED)
find_package(PCL 1.7.1 REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem timer program_options)
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  ConnectTopic.srv
  AddVirtualFrame.srv
  LogTopic.srv
)

generate_messages(
  # DEPENDENCIES
  # std_msgs
)

## Declare a catkin package
catkin_package()

add_library(infuse_debug_tools 
	src/asn1_bitstream_transform_processer.cpp
	src/asn1_bitstream_logger.cpp
	src/PointCloudExtractor.cpp
	src/PoseExtractor.cpp
	src/ImagePairExtractor.cpp
    src/ImagePairMatcher.cpp
	src/GpsExtractor.cpp
)
target_include_directories(infuse_debug_tools
	PUBLIC
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(infuse_debug_tools
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	${PCL_IO_LIBRARIES}
	${PCL_VISUALIZATION_LIBRARIES}
	${OpenCV_LIBS}
	infuse_asn1_types::infuse_asn1_types
	infuse_asn1_conversions::base_conversions
	infuse_asn1_conversions::pcl_conversions
)

## asn1_bitstream_to_pcd_converter
add_executable(asn1_bitstream_to_pcd_converter_node
	src/asn1_bitstream_to_pcd_converter.cpp
)
target_include_directories(asn1_bitstream_to_pcd_converter_node
	PUBLIC
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(asn1_bitstream_to_pcd_converter_node
	${catkin_LIBRARIES}
	${PCL_IO_LIBRARIES}
	infuse_asn1_types::infuse_asn1_types
	infuse_asn1_conversions::pcl_conversions
)

## asn1_bitstream_to_tf
add_executable(asn1_bitstream_to_tf_node src/asn1_bitstream_to_tf.cpp)
target_include_directories(asn1_bitstream_to_tf_node
	PUBLIC
	${catkin_INCLUDE_DIRS}
	# ${PCL_INCLUDE_DIRS}
)
target_link_libraries(asn1_bitstream_to_tf_node
	infuse_debug_tools
	${catkin_LIBRARIES}
	# ${PCL_IO_LIBRARIES}
	infuse_asn1_types::infuse_asn1_types
	infuse_asn1_conversions::base_conversions
	# infuse_asn1_conversions::pcl_conversions
)

## asn1_bitstream_to_point_cloud
add_executable(asn1_bitstream_to_point_cloud_node src/asn1_bitstream_to_point_cloud.cpp)
target_include_directories(asn1_bitstream_to_point_cloud_node
	PUBLIC
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
target_link_libraries(asn1_bitstream_to_point_cloud_node
	infuse_debug_tools
	${catkin_LIBRARIES}
	${PCL_IO_LIBRARIES}
	infuse_asn1_types::infuse_asn1_types
	infuse_asn1_conversions::base_conversions
	infuse_asn1_conversions::pcl_conversions
)

## asn1_bitstream_transform_to_logfile
add_executable(asn1_bitstream_transform_to_logfile_node
	src/asn1_bitstream_transform_to_logfile.cpp
)
target_include_directories(asn1_bitstream_transform_to_logfile_node
	PUBLIC
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(asn1_bitstream_transform_to_logfile_node
	${catkin_LIBRARIES}
	infuse_debug_tools
)

## asn1_bitstream_point_cloud_to_logfile
add_executable(asn1_bitstream_point_cloud_to_logfile_node
	src/asn1_bitstream_point_cloud_to_logfile.cpp
)
target_include_directories(asn1_bitstream_point_cloud_to_logfile_node
	PUBLIC
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(asn1_bitstream_point_cloud_to_logfile_node
	${catkin_LIBRARIES}
	infuse_debug_tools
)

## infuse_data_extractor
add_executable(infuse_data_extractor
	src/infuse_data_extractor.cpp
)
target_include_directories(infuse_data_extractor
	PUBLIC
	${Boost_INCLUDE_DIRS}
)
target_link_libraries(infuse_data_extractor
	${Boost_LIBRARIES}
	infuse_debug_tools
)

## infuse_stereo_matching
add_executable(infuse_stereo_matching
        src/infuse_stereo_matching.cpp
)
target_include_directories(infuse_stereo_matching
        PUBLIC
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(infuse_stereo_matching
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        infuse_debug_tools
)

if(WITH_XIMGPROC)
    add_definitions(-DWITH_XIMGPROC)
endif()

## asn1_bitstream_to_image
add_executable(asn1_bitstream_to_image_node src/asn1_bitstream_to_img.cpp)
target_include_directories(asn1_bitstream_to_image_node
        PUBLIC
	${catkin_INCLUDE_DIRS}
)
target_link_libraries(asn1_bitstream_to_image_node
        ${catkin_LIBRARIES}
	infuse_debug_tools
	#${PCL_IO_LIBRARIES}
	infuse_asn1_types::infuse_asn1_types
	infuse_asn1_conversions::base_conversions
	#infuse_asn1_conversions::ros_conversions
)

## asn1_bitstream_to_disparity_image
add_executable(asn1_bitstream_to_disparity_image_node src/asn1_bitstream_to_disparity_image.cpp)
target_include_directories(asn1_bitstream_to_disparity_image_node
        PUBLIC
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(asn1_bitstream_to_disparity_image_node
        ${catkin_LIBRARIES}
        infuse_debug_tools
        infuse_asn1_types::infuse_asn1_types
        infuse_asn1_conversions::base_conversions
        ${OpenCV_LIBRARIES}
)
