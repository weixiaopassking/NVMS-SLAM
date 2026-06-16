#ifndef UTILITY_H_
#define UTILITY_H_
// c++
#include <iostream>
#include <string>
#include <tr1/unordered_map>

#include "cloudMap.hpp"

#include "sophus/so3.hpp"

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b);

void subSampleFrame2(std::vector<point3D> &frame, double size_voxel);

void subSampleFrame(std::vector<point3D> &frame, double size_voxel);

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling);

void transformPoint(point3D &point_temp, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar);

Eigen::Matrix3d g2R(const Eigen::Vector3d &g);

namespace std
{
     template <typename T, typename... Args>
     std::unique_ptr<T> make_unique(Args &&...args)
     {
          return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
     }
}
#endif