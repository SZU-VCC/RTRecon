#ifndef LIDAR_POINTCLOUD_POINT_TYPES_H
#define LIDAR_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
// if lidar type is HesaiPander40
namespace pcl {
    struct PointXYZITR {
        PCL_ADD_POINT4D
        float intensity;
        double timestamp;
        std::uint16_t ring;                   ///< laser ring number
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZITR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double,timestamp,timestamp)
                                  (std::uint16_t, ring, ring))

// is lidar type is velodyne 16
namespace pcl {
    struct PointXYZIR {
        PCL_ADD_POINT4D
        float intensity;
        std::uint16_t ring;                   ///< laser ring number
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;

}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring))

namespace LiDarPointType {
    typedef pcl::PointXYZITR HesaiPander40;
    typedef pcl::PointXYZIR  Velodyne16;
}  // namespace LiDarPointType

#endif