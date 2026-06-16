/*
 * @Author: lian-yue0515 506630928@qq.com
 * @Date: 2025-01-08 21:37:32
 * @LastEditors: lian-yue0515 506630928@qq.com
 * @LastEditTime: 2025-01-08 22:02:08
 * @FilePath: /LIO_NVM/src/lio_nvm-main/src/common/keyframe.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by xiang on 22-12-6.
//

#ifndef SLAM_IN_AUTO_DRIVING_KEYFRAME_H
#define SLAM_IN_AUTO_DRIVING_KEYFRAME_H

#include "common/eigen_types.h"
#include "tools/point_types.h"
#include "cloudMap.hpp"

#include <map>

namespace zjloc
{
    struct lioFrame //  保存keyframe数据
    {
        lioFrame() {}
        lioFrame(const SE3 &pose, std::vector<point3D> &cloud)
            : pose_(pose), pose_in_base_(pose)
        {
            cloud_.insert(cloud_.end(), cloud.begin(), cloud.end());
        }
        lioFrame(double time, IdType id, const SE3 &pose, std::vector<point3D> &cloud)
            : timestamp_(time), id_(id), pose_(pose), pose_in_base_(pose)
        {
            cloud_.insert(cloud_.end(), cloud.begin(), cloud.end());
        }

        double timestamp_ = 0;
        IdType id_ = -1;
        SE3 pose_;
        SE3 pose_in_base_;
        std::vector<point3D> cloud_;
    };

    struct Keyframe
    {
        Keyframe() {}
        Keyframe(double time, IdType id, const SE3 &lidar_pose) //  不保存激光数据，太费资源
            : timestamp_(time), id_(id), lidar_pose_(lidar_pose)
        {
        }
        Keyframe(double time, IdType id, const SE3 &lidar_pose, CloudPtr cloud)
            : timestamp_(time), id_(id), lidar_pose_(lidar_pose), cloud_(cloud) {}

        /// 将本帧点云存盘，从内存中清除
        void SaveAndUnloadScan(const std::string &path);

        void LoadScan(const std::string &path);

        /// 保存至文本文件
        void Save(std::ostream &os);

        /// 从文件读取
        void Load(std::istream &is);

        double timestamp_ = 0;             // 时间戳
        IdType id_ = 0;                    // 关键帧id，唯一
        SE3 lidar_pose_;                   // 雷达位姿
        SE3 rtk_pose_;                     // rtk 位姿
        SE3 opti_pose_1_;                  // 第一阶段优化pose
        SE3 opti_pose_2_;                  // 第二阶段优化pose
        Vec6d rtk_info_ = Vec6d::Zero(); // rtk info
        bool rtk_heading_valid_ = false;   // rtk是否含有旋转
        bool rtk_valid_ = true;            // rtk原始状态是否有效
        bool rtk_inlier_ = true;           // rtk在优化过程中是否为正常值

        CloudPtr cloud_ = nullptr;
    };

    bool LoadKeyFrames(const std::string &path, std::map<IdType, std::shared_ptr<Keyframe>> &keyframes);
} // namespace zjloc

using KFPtr = std::shared_ptr<zjloc::Keyframe>;

#endif // SLAM_IN_AUTO_DRIVING_KEYFRAME_H
