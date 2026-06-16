
// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once


#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// robin_map
#include <tsl/robin_map.h>
#include "cloudMap.hpp"
struct Neighborhood
{
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     Eigen::Vector3d center = Eigen::Vector3d::Zero();
     Eigen::Vector3d normal = Eigen::Vector3d::Zero();
     Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
     double a2D = 1.0; // Planarity coefficient
};


typedef pcl::PointXYZINormal PointType;

inline double rad2deg_(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

void subSampleFrame2(std::vector<point3D> &frame, double size_voxel)
{
    tsl::robin_map<voxel, point3D> grid;
    grid.reserve(size_t(frame.size() / 4.));
    voxel vox;
    for (int i = 0; i < (int)frame.size(); i++)
    {
        vox.x = static_cast<short>(frame[i].point[0] / size_voxel);
        vox.y = static_cast<short>(frame[i].point[1] / size_voxel);
        vox.z = static_cast<short>(frame[i].point[2] / size_voxel);
        if (grid.find(vox) == grid.end())
            grid[vox] = frame[i];
    }
    // std::cout << "frame size:" << frame.size() << "res:" << size_voxel << std::endl;
    frame.resize(0);
    frame.reserve(grid.size());
    for (const auto &[_, point] : grid)
        frame.push_back(point);
    // std::cout << "after size: " << frame.size() << ", " << grid.size() << std::endl;
}


void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling)
{
     keypoints.resize(0);
     std::vector<point3D> frame_sub;
     frame_sub.resize(frame.size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          frame_sub[i] = frame[i];
     }
     subSampleFrame2(frame_sub, size_voxel_subsampling);
     keypoints.reserve(frame_sub.size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          keypoints.push_back(frame_sub[i]);
     }
}

struct PointToPlaneFunctor
{

     static constexpr int NumResiduals() { return 1; }

     PointToPlaneFunctor(const Eigen::Vector3d &reference,
                         const Eigen::Vector3d &target,
                         const Eigen::Vector3d &reference_normal,
                         double weight = 1.0) : reference_(reference),
                                                target_(target),
                                                reference_normal_(reference_normal),
                                                weight_(weight) {}

     template <typename T>
     bool operator()(const T *const trans_params, const T *const rot_params, T *residual) const
     {
          Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
          Eigen::Matrix<T, 3, 1> target_temp(T(target_(0, 0)), T(target_(1, 0)), T(target_(2, 0)));
          Eigen::Matrix<T, 3, 1> transformed = quat * target_temp;
          transformed(0, 0) += trans_params[0];
          transformed(1, 0) += trans_params[1];
          transformed(2, 0) += trans_params[2];

          Eigen::Matrix<T, 3, 1> reference_temp(T(reference_(0, 0)), T(reference_(1, 0)), T(reference_(2, 0)));
          Eigen::Matrix<T, 3, 1> reference_normal_temp(T(reference_normal_(0, 0)), T(reference_normal_(1, 0)), T(reference_normal_(2, 0)));

          residual[0] = T(weight_) * (transformed - reference_temp).transpose() * reference_normal_temp;
          return true;
     }

     static ceres::CostFunction *Create(const Eigen::Vector3d &point_world_,
                                        const Eigen::Vector3d &point_body_,
                                        const Eigen::Vector3d &norm_vector_,
                                        double weight_ = 1.0)
     {
          return (new ceres::AutoDiffCostFunction<PointToPlaneFunctor, 1, 3, 4>(
              new PointToPlaneFunctor(point_world_, point_body_, norm_vector_, weight_)));
     }

     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     Eigen::Vector3d reference_;
     Eigen::Vector3d target_;
     Eigen::Vector3d reference_normal_;
     double weight_ = 1.0;
};

struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};


