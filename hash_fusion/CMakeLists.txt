cmake_minimum_required(VERSION 3.0.2)

project(hash_fusion)

## Compile as C++11
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++17)
# supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             OpenCV
             pcl_ros
             roscpp
             rospy
             sensor_msgs
             std_msgs
             nav_msgs
             tf
             filters
             fusion_msgs
             grid_map_msgs
             grid_map_ros
)

# embree find_package
FIND_PACKAGE(embree 4 REQUIRED)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_odomlaser
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
	include
    include/tools
    include/updater
    include/volume
    ${GLOG_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    #${OCTOMAP_INCLUDE_DIRS}
)


aux_source_directory(src DIR_SRCS)
aux_source_directory(src/tools DIR_SRCS)
aux_source_directory(src/volume DIR_SRCS)
aux_source_directory(src/updater DIR_SRCS)

###### build ########
add_executable(hash_fusion ${DIR_SRCS})

# target_include_directories(hash_fusion PUBLIC src)

target_link_libraries(hash_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} embree glog)