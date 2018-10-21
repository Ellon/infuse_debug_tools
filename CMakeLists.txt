cmake_minimum_required(VERSION 2.8.3)
project(infuse_debug_tools)

set (CMAKE_CXX_STANDARD 14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp infuse_msgs std_msgs message_generation tf2_ros)

find_package(infuse_asn1_types REQUIRED)
find_package(infuse_asn1_conversions REQUIRED)
find_package( PCL 1.7.1 REQUIRED )

add_service_files(
  FILES
  ConnectTopic.srv
  AddVirtualFrame.srv
)

generate_messages(
  # DEPENDENCIES
  # std_msgs
)

## Declare a catkin package
catkin_package()

add_library(infuse_debug_tools 
	src/asn1_bitstream_transform_processer.cpp
)
target_include_directories(infuse_debug_tools
	PUBLIC
	${catkin_INCLUDE_DIRS}
)
target_link_libraries(infuse_debug_tools
	${catkin_LIBRARIES}
	infuse_asn1_types::infuse_asn1_types
	infuse_asn1_conversions::base_conversions
)

## asn1_bitstream_to_pcd_converter
add_executable(asn1_bitstream_to_pcd_converter_node
	src/asn1_bitstream_to_pcd_converter.cpp
)
target_include_directories(asn1_bitstream_to_pcd_converter_node
	PUBLIC
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
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
	${catkin_LIBRARIES}
	${PCL_IO_LIBRARIES}
	infuse_asn1_types::infuse_asn1_types
	infuse_asn1_conversions::base_conversions
	infuse_asn1_conversions::pcl_conversions
)
