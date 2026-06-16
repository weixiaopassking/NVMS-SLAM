//
// Created by xiang on 22-12-7.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "common/keyframe.h"
#include "common/utils_trans.h"
#include "tools/tool_color_printf.hpp"

DEFINE_double(voxel_size, 0.01, "导出地图分辨率");
DEFINE_string(data_path, "/home/cc/catkin_ws/src/lio_nvm/log/kf_data/", "输入地址");
DEFINE_string(save_path, "/home/cc/catkin_ws/src/lio_nvm/log", "存储地址");
DEFINE_string(pose_source, "lidar", "存储地址");

/**
 * 将keyframes.txt中的地图和点云合并为一个pcd
 */

int main(int argc, char **argv)
{
    if (argc == 2 && strncmp(argv[1], "-h", 10) == 0)
    {
        std::cout << "usage: dump --voxel_size=0.1 --data_path=/home/cc/catkin_ws/src/lio_nvm/log/kf_data/ --save_path=/home/cc/catkin_ws/src/lio_nvm/log --pose_source=lidar" << std::endl;
        std::cout << "param1: voxel_size " << std::endl;
        std::cout << "param2: data_path" << std::endl;
        std::cout << "param3: save_path" << std::endl;
        std::cout << "param4: pose_source" << std::endl;
        return 0;
    }

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    std::string data_path = FLAGS_data_path;
    std::cout << "data_path:" << data_path << std::endl;
    std::string save_path = FLAGS_save_path;
    std::string pose_source = FLAGS_pose_source;

    using namespace zjloc;
    std::map<IdType, KFPtr> keyframes;
    if (!LoadKeyFrames(data_path + "keyframes.txt", keyframes))
    {
        LOG(ERROR) << "failed to load keyframes.txt";
        return -1;
    }

    if (keyframes.empty())
    {
        LOG(INFO) << "keyframes are empty";
        return 0;
    }

    // dump kf cloud and merge
    LOG(INFO) << "merging... " << pose_source;
    CloudPtr global_cloud(new PointCloudType);

    // pcl::VoxelGrid<PointType> voxel_grid_filter;
    float resolution = FLAGS_voxel_size;
    // voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    int cnt = 0;
    for (auto &kfp : keyframes)
    {
        auto kf = kfp.second;
        SE3 pose;
        if (pose_source == "rtk")
        {
            pose = kf->rtk_pose_;
        }
        else if (pose_source == "lidar")
        {
            pose = kf->lidar_pose_;
        }
        else if (pose_source == "opti1")
        {
            pose = kf->opti_pose_1_;
        }
        else if (pose_source == "opti2")
        {
            pose = kf->opti_pose_2_;
        }

        kf->LoadScan(data_path);

        CloudPtr cloud_trans(new PointCloudType);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix());

        // voxel size
        CloudPtr kf_cloud_voxeled(new PointCloudType);
        // voxel_grid_filter.setInputCloud(cloud_trans);
        // voxel_grid_filter.filter(*kf_cloud_voxeled);

        gridSampling0(cloud_trans, kf_cloud_voxeled, resolution);

        *global_cloud += *kf_cloud_voxeled;
        kf->cloud_ = nullptr;

        std::cout << ANSI_COLOR_GREEN << "merging " << cnt << " in " << keyframes.size() << ", pts: " << kf_cloud_voxeled->size()
                  << " global pts: " << global_cloud->size() << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_DELETE_LAST_LINE;
        cnt++;
    }
    pcl::io::savePCDFileBinaryCompressed(save_path + "/map_ori.pcd", *global_cloud);
    if (!global_cloud->empty())
    {
        CloudPtr save_cloud(new PointCloudType);
        gridSampling0(global_cloud, save_cloud, resolution);
        pcl::io::savePCDFileBinaryCompressed(save_path + "/map.pcd", *save_cloud);
        // pcl::io::savePCDFileBinaryCompressed(FLAGS_dump_to + "/map.pcd", *save_cloud);
    }

    LOG(INFO) << "done.";
    return 0;
}
