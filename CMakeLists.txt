cmake_minimum_required(VERSION 2.8.3)
project(infuse_debug_tools)

set (CMAKE_CXX_STANDARD 14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp infuse_msgs std_msgs)

find_package(infuse_asn1_types REQUIRED)
find_package(infuse_asn1_conversions REQUIRED)

find_package( PCL 1.7.1 REQUIRED )

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories (${PCL_INCLUDE_DIRS})


add_executable(asn1_bitstream_to_pcd_converter_node src/asn1_bitstream_to_pcd_converter.cpp)
target_link_libraries(asn1_bitstream_to_pcd_converter_node
	${catkin_LIBRARIES}
	${PCL_IO_LIBRARIES}
	infuse_asn1_types::infuse_asn1_types
	infuse_asn1_conversions::pcl_conversions
	)