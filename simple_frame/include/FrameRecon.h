#ifndef FRAMERECON_H
#define FRAMERECON_H

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <cmath>
#include <stdio.h>
#include <string.h>

// glog
#include <glog/logging.h>

// ros related
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// pcl related
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_ros/transforms.h"

// polygon related
#include <fusion_msgs/MeshArray.h>
#include <shape_msgs/Mesh.h>
#include <visualization_msgs/Marker.h>

// project related
#include "CircularVector.h"
#include "ExplicitRec.h"
#include "GHPR.h"
#include "MeshSample.h"
#include "OutputUtils.h"
#include "SectorPartition.h"
#include "tools/DebugManager.h"

#include "LiDarPointType.h"

// 基于odom坐标系 完成单帧点云重建
//  Trajectory state data.
struct RosTimePoint {
    // The time of the measurement leading to this state (in seconds). //
    ros::Time oTimeStamp;

    //********coordinate value****************
    // The global trajectory position in 3D space.
    pcl::PointXYZI oLocation;
};
inline std::ostream &operator<<(std::ostream &out, const sensor_msgs::PointCloud2::_header_type &header) {
    out << header.frame_id << ", " << header.seq << ", " << header.stamp.sec << "." << header.stamp.nsec;
    return out;
}

class FrameRecon {
  public:
    //*************Initialization function*************
    // Constructor
    FrameRecon(ros::NodeHandle &node, ros::NodeHandle &nodeHandle);

    // Destructor
    virtual ~FrameRecon();

    // For lazy loading
    virtual void LazyLoading();

    // Reads and verifies the ROS parameters.
    virtual bool ReadLaunchParams(ros::NodeHandle &nodeHandle);

    void SamplePoints(const pcl::PointCloud<pcl::PointXYZI> &vCloud, pcl::PointCloud<pcl::PointXYZI> &vNewCloud,
                      int iSampleNum, bool bIntervalSamp = true);

    //*************handler function*************
    // handle the trajectory information
    virtual void HandleTrajectory(const nav_msgs::Odometry &oTrajectory);

    // handle the ground point clouds topic
    virtual void HandlePointClouds(const sensor_msgs::PointCloud2 &vCloudRosData);

    //*************Publish function*************
    template <class T> void PublishPointCloud(pcl::PointCloud<T> &pointcloud, ros::Publisher &publisher);
    // publish point clouds
    void PublishFramePointsNormal(const pcl::PointCloud<pcl::PointNormal> &vCloudNormal);
    void PublishAdditionalPoints(const pcl::PointCloud<pcl::PointNormal> &vCloudNormal);

    // publish meshes
    void PublishFrameMesh();
    void PublishFrameMeshAlgorithm();
    void PublishFrameMeshAlgorithmView();

    //*******odom related*******
    // Trajectory line interpolation
    void InterpolateTraj(const RosTimePoint &oCurrent, const RosTimePoint &oPast, const float &fRatio,
                         pcl::PointXYZI &oInter);

    // Query the nearest trajectory point
    pcl::PointXYZI ComputeQueryTraj(const ros::Time &oQueryTime);

    std::unordered_map<std::string, std::string> outputPathList;

    std::string CheckOutputPath(std::string & outFileHead, const std::string & fileName);

    // output point cloud for test
    void OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> &vCloud, bool bAllRecord = false);

    // reload, output point cloud with given feature for test
    void OutputPCFile(const pcl::PointCloud<pcl::PointXYZI> &vCloud, const std::vector<float> &vFeatures,
                      bool bAllRecord = false);

  protected:
    ros::NodeHandle &node;
    ros::NodeHandle &nodeHandle;

    // launch文件中读入的赋值
    //***file related***
    std::string sfOutFileHead; // launch文件中读入的赋值
    std::stringstream outPCFileName;

    bool outPCFileFlag;      // whether the file is generated or not
    std::ofstream outPCFile; // ouput file
    bool outputFiles;

    //***for input odom topic***
    std::string subOdomTopic; // launch文件中读入的赋值
    ros::Subscriber odomSuber;

    //***for input point cloud topic***
    std::string subCloudTopic; // launch文件中读入的赋值
    ros::Subscriber cloudSuber;

    //***for output cloud topic***
    std::string pubFramePointsNormalTopic;
    ros::Publisher FramePointsNormalPublisher;

    // displayed point topic
    std::string pubAdditionalPointsTopic = "/additional_points";
    ros::Publisher AdditionalPointsPublisher;

    //***for output mesh***
    // /frame_meshs，单帧网格的输出
    std::string pubFrameMeshTopic;
    ros::Publisher FrameMeshPublisher;

    // mesh algo for multiframe update
    std::string pubFrameMeshAlgoTopic;
    ros::Publisher FrameMeshAlgoPublisher;

    std::string pubFrameMeshAlgoViewTopic;
    ros::Publisher FrameMeshAlgoViewPublisher;

    ros::Publisher viewPointPublisher;
    ros::Publisher odomPublisher;
    ros::Publisher FrameDebugPointsPublisher;
    // frame sampling
    int frameSmpNum;
    // sampling number of input point clouds
    int sampleInPNum;
    // number of sector
    int sectorNum;

    // 激光雷达类型 确定点云类型 ，以及相关参数
    // Lidar ids config, stored in the intensity of point-cloud
    std::string lidarFrameId = "lidar_sensor_VLP16";
    std::string baselinkFrameId = "base_link";
    std::string odomFrameId = "odom";
    std::string lidarType;
    int lidarLineMin;
    int lidarLineMax;

    // 位姿转换为视点相关参数
    tf::TransformListener listener;

    bool isDebug;

    //**frenquency related**
    // How many frames of point cloud have been calculated cumulatively
    unsigned int pcFrameCount;
    // frame count
    unsigned int trajCount;
    // z offset of odom to lidar sensor
    float viewZOffset;

    double averageReconstructTime;
    double maxReconstructTime;
    int reconstructFrameNum;
    int totalFrameNum;
    unsigned int trajFrameNum;

    // circle vector of odom
    CircularVector<RosTimePoint> odomHistory;
    // explicit reconstruction
    ExplicitRec meshAlgoBuilder;

    TimeDebugger timer;
};

std::ostream &operator<<(std::ostream &out, const sensor_msgs::PointCloud2::_header_type &header);

#endif
