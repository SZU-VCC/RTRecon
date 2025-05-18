#ifndef HESAI_PANDAR_POINTCLOUD_POINT_TYPES_H
#define HESAI_PANDAR_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace HesaiPandarPointcloud
{
struct PointXYZITR {
    PCL_ADD_POINT4D
    float intensity;
    double timestamp;
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

}; // namespace pandar_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZITR,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))
#endif
