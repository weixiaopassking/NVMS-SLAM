#ifndef UTILS_TRANS_H_
#define UTILS_TRANS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utility.h"

/// @brief 点云数据下采样，采用自定义方式
/// @param frame input
/// @param keypoints    output
/// @param size_voxel_subsampling   downsample resolution
void gridSampling0(const pcl::PointCloud<pcl::PointXYZI>::Ptr &frame, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints, double size_voxel_subsampling);

#endif