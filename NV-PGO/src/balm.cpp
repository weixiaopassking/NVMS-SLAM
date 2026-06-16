#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <malloc.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include "bavoxel.hpp"
#include <mutex>
#include <tbb/tbb.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <thread>
#include "aloam_velodyne/tic_toc.h"

// #include "scancontext/KDTreeVectorOfVectorsAdaptor.h"
// #include "nanoflann_pcl.hpp"

std::mutex mBuf;
bool Is_balm = false;

vector<IMUST> IMUST_list;
vector<geometry_msgs::PoseStamped> pose_list;
vector<pcl::PointCloud<PointType>::Ptr> cloud_buf;
vector<double> time_buf;

void path_handler(const nav_msgs::Path::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mBuf);
    pose_list = msg->poses;
}

void pointcloud_handler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mBuf);
    pcl::PointCloud<PointType>::Ptr pl(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg, *pl);
    cloud_buf.push_back(pl);
    time_buf.push_back(msg->header.stamp.toSec());
}

void IsLoop_handler(const std_msgs::Bool::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mBuf);
    cout << "BALM 订阅到Bool消息: " << msg->data << endl;
    Is_balm = msg->data;
}

void pub_pl_func(pcl::PointCloud<PointType> &pl,ros::Publisher &pub)
{
    pl.height = 1;
    pl.width = pl.size();
    std::string filename = "/home/myx/Fighting/Indoor-Multi-Session/src/Indoor_Multi_session/PCD/scans/after_balm.pcd";
    pcl::io::savePCDFileBinary(filename, (pl));
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pl, output);
    output.header.frame_id = "camera_init";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "balm_back_end");
    ros::NodeHandle n;

    ros::Subscriber sub_odom = n.subscribe<nav_msgs::Path>("/aft_pgo_path", 1000, path_handler);
    ros::Subscriber sub_ploud = n.subscribe<sensor_msgs::PointCloud2>("/keyframe_cloud", 1000, pointcloud_handler); // 改
    ros::Subscriber sub_isloop = n.subscribe<std_msgs::Bool>("/IsLoop", 1000, IsLoop_handler);

    ros::Publisher pubCloudAftBA = n.advertise<sensor_msgs::PointCloud2>("/map_AftBA", 1000);
    ros::Publisher pubCloudBefBA = n.advertise<sensor_msgs::PointCloud2>("/map_BefBA", 1000);
    ros::Publisher pubOdomBefBA = n.advertise<nav_msgs::Odometry>("/Odometry_BefBA", 1000);
    ros::Publisher pubOdomAftBA = n.advertise<nav_msgs::Odometry>("/Odometry_AftBA", 1000);
    ros::Publisher pubPathAftBA = n.advertise<nav_msgs::Path>("/Path_AftBA", 1000);

    while (ros::ok())
    {
        ros::spinOnce();
        if (Is_balm)
        {
            cout << "balm strat work-----------------" << endl;
            Is_balm = false;
            
            vector<geometry_msgs::PoseStamped> poses;
            vector<pcl::PointCloud<PointType>::Ptr> cloud;
            {
                std::lock_guard<std::mutex> lock(mBuf);
                poses = pose_list;
                cloud = cloud_buf;
            }
            cout << "pose size : " << poses.size() << endl;
            cout << "cloud size : " << cloud.size() << endl;
            win_size = poses.size();

            if (win_size == 0 || cloud.size() < win_size)
            {
                ROS_WARN("No pose or point cloud data available.");
                continue;
            }
            unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> surf_map;
            pcl::PointCloud<PointType> pl_befba;
            for (int i = 0; i < win_size; i++)
            {
                IMUST_list.reserve(win_size);
                if (cloud[i] == nullptr || cloud[i]->points.empty())
                {
                    ROS_WARN("Point cloud data at index %d is invalid or empty.", i);
                    continue;
                }
                const auto &pose = poses[i].pose;
                Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
                Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);

                IMUST curr;
                curr.t = time_buf[i];
                curr.R = q.toRotationMatrix();
                curr.p = translation;
                IMUST_list.push_back(curr);

                pcl::PointCloud<PointType> pl_tem = *cloud[i];
                pl_transform(pl_tem, curr);
                pl_befba += pl_tem;
                // cloud_computeNormal(cloud[i]);
                cut_voxel(surf_map, *cloud[i], curr, i);
            }
            pub_pl_func(pl_befba, pubCloudBefBA);
            pl_befba.clear();
            TicToc time_recut;
            VOX_HESS voxhess;
            for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
            {
                // iter->second->save_voxel(win_size);
                // iter->second->recut(win_size);
                iter->second->recut_normal(win_size);
                iter->second->tras_opt(voxhess, win_size);
            }
            cout << "time_recut time: " << time_recut.toc() << endl;
            TicToc damping_iter;
            BALM2 opt_lsv;
            opt_lsv.damping_iter(IMUST_list, voxhess);
            cout << "BALM优化完成!!!"  << endl;
            cout << "damping_iter time: " << damping_iter.toc() << endl;
            pcl::PointCloud<PointType> pl_aftba;
            pcl::PointCloud<PointType> pl_aftba_n;
            int num = 0;
            for(int i = 0;i<win_size;i++)
            {
                pcl::PointCloud<PointType> pl_temp = *cloud[i];
                pl_transform(pl_temp, IMUST_list[i]);
                pl_aftba += pl_temp;
                pl_aftba_n += pl_temp;    
                if(IMUST_list[i+1].t-IMUST_list[i].t > 80)
                {
                    pl_aftba_n.height = 1;
                    pl_aftba_n.width = pl_aftba_n.size();
                    std::string filename = "/home/myx/Fighting/Indoor-Multi-Session/src/Indoor_Multi_session/PCD/scans/after_balm" + std::to_string(num) + ".pcd";
                    pcl::io::savePCDFileBinary(filename, (pl_aftba_n));
                    pl_aftba_n.clear();
                    num++;
                }
            }

            pl_aftba_n.height = 1;
            pl_aftba_n.width = pl_aftba_n.size();
            std::string filename = "/home/myx/Fighting/Indoor-Multi-Session/src/Indoor_Multi_session/PCD/scans/after_balm" + std::to_string(num) + ".pcd";
            pcl::io::savePCDFileBinary(filename, (pl_aftba_n));

            pub_pl_func(pl_aftba, pubCloudAftBA);
            
            nav_msgs::Path pathAftBA;
            nav_msgs::Odometry odomAftBA;
            Eigen::Matrix4d trans(Eigen::Matrix4d::Identity());
            for (int i = 0; i < win_size; i++)
            {
                trans.block<3, 3>(0, 0) = IMUST_list[i].R;
                trans.block<3, 1>(0, 3) = IMUST_list[i].p;

                Eigen::Quaterniond q_w_curr(IMUST_list[i].R);
                nav_msgs::Odometry odom_AftBA;
                odom_AftBA.header.frame_id = "camera_init";
                odom_AftBA.child_frame_id = "/aft_balm";
                ros::Time time = ros::Time::now();
                odom_AftBA.header.stamp = ros::Time(IMUST_list[i].t);
                odom_AftBA.pose.pose.orientation.x = q_w_curr.x();
                odom_AftBA.pose.pose.orientation.y = q_w_curr.y();
                odom_AftBA.pose.pose.orientation.z = q_w_curr.z();
                odom_AftBA.pose.pose.orientation.w = q_w_curr.w();
                odom_AftBA.pose.pose.position.x = IMUST_list[i].p.x();
                odom_AftBA.pose.pose.position.y = IMUST_list[i].p.y();
                odom_AftBA.pose.pose.position.z = IMUST_list[i].p.z();
                odomAftBA = odom_AftBA;
                pubOdomAftBA.publish(odomAftBA);

                geometry_msgs::PoseStamped poseStampAftBA;
                poseStampAftBA.header = odom_AftBA.header;
                poseStampAftBA.pose = odom_AftBA.pose.pose;

                pathAftBA.header.stamp = odom_AftBA.header.stamp;
                pathAftBA.header.frame_id = "camera_init";
                pathAftBA.poses.push_back(poseStampAftBA);

                // pcl::PointCloud<PointType> pcloud;
                // pcl::transformPointCloud((*cloud_buf[i]), pcloud, trans);
                // sensor_msgs::PointCloud2 full_msg;
                // pcl::toROSMsg(pcloud, full_msg);
                // full_msg.header.stamp = time;
                // full_msg.header.frame_id = "camera_init";
                // ROS_INFO("Publishing PointCloud2 message with timestamp: %f", time.toSec()); 
                // pubCloudAftBA.publish(full_msg);
            }
            pubPathAftBA.publish(pathAftBA);

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            tf::Quaternion q;
            transform.setOrigin(tf::Vector3(odomAftBA.pose.pose.position.x, odomAftBA.pose.pose.position.y, odomAftBA.pose.pose.position.z));
            q.setW(odomAftBA.pose.pose.orientation.w);
            q.setX(odomAftBA.pose.pose.orientation.x);
            q.setY(odomAftBA.pose.pose.orientation.y);
            q.setZ(odomAftBA.pose.pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, odomAftBA.header.stamp, "camera_init", "/aft_balm"));

            for (auto iter = surf_map.begin(); iter != surf_map.end();)
            {
                delete iter->second;
                surf_map.erase(iter++);
            }
            surf_map.clear();
            IMUST_list.clear();
            malloc_trim(0);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
