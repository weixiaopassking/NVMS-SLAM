#include "utils_trans.h"

void gridSampling0(const pcl::PointCloud<pcl::PointXYZI>::Ptr &frame, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints, double size_voxel_subsampling)
{
     keypoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
     std::vector<point3D> frame_sub;
     frame_sub.resize(frame->size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          point3D pt;
          pt.raw_point = Eigen::Vector3d(frame->points[i].x, frame->points[i].y, frame->points[i].z);
          pt.point = pt.raw_point;
          pt.intensity = frame->points[i].intensity;

          frame_sub[i] = pt;
     }
     // std::cout << "input :" << frame->size() << ", " << frame_sub.size() << ",vs:" << size_voxel_subsampling << std::endl;
     subSampleFrame2(frame_sub, size_voxel_subsampling);
     // std::cout << "after:" << frame_sub.size() << std::endl;
     // keypoints.reserve(frame_sub.size());
     for (int i = 0; i < (int)frame_sub.size(); i++)
     {
          pcl::PointXYZI pt;
          pt.x = frame_sub[i].raw_point[0], pt.y = frame_sub[i].raw_point[1], pt.z = frame_sub[i].raw_point[2];
          pt.intensity = frame_sub[i].intensity;
          keypoints->push_back(pt);
     }
     // std::cout << "after:" << keypoints->size() << std::endl;
}