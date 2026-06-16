#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <eigen3/Eigen/Dense>

#include <ceres/ceres.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "scancontext/Scancontext.h"
#include "mapmanager.hpp"
#include <std_msgs/Bool.h>
#include <tbb/tbb.h>
#include <quatro/quatro_module.h>
// #include <nano_gicp/point_type_nano_gicp.hpp>
// #include <nano_gicp/nano_gicp.hpp>
#include "nanoflann_pcl.hpp"
#include "se3.hpp"
#include "tool_color_printf.hpp"
#include "cloudMap.hpp"

using namespace gtsam;

using std::cout;
using std::endl;
using SE3 = Sophus::SE3d;
int SKIP_similar = 1;
int cnt = 0;
double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0;
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false;
bool Is_balm = false;
int sleepmapsize = 0;
int after_merge_idx = -1;
Pose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init
Pose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero

std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;
std::queue<std::pair<int, int>> scLoopICPBuf;
std::queue<double> scLoopYaw_diff;

std::mutex mBuf;
std::mutex SCBuf;
std::mutex mKF;
std::mutex mtransFinal;
std::mutex mrecentIdxUpdated;
std::condition_variable condvar_bool;
double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
std::mutex mgtSAMgraphMade;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr priorNoise_o;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustLoopNoise_o;
pcl::VoxelGrid<PointType> downSizeFilterScancontext;
double scDistThres, scMaximumRadius, SimilarSCdistThres, desc_THRES;
double loopICPThreshold, similarICPThreshold, Strengthening_constraints_ICPThreshold;
int loopICP_historyKeyframeSearchNum, similarICP_historyKeyframeSearchNum, Strengthening_ICP_historyKeyframeSearchNum;
/*Strengthening_LoopICPThreshold*/
double overlap_ratio, icp_eigval, size_voxel_map, min_distance_points, weight_alpha, weight_neighborhood, max_dist_to_plane_icp, thres_orientation_norm, thres_translation_norm, surf_res;
int capacity_, history_kf_buff, voxel_neighborhood, max_num_points_in_voxel, max_iteration, min_num_residuals, threshold_voxel_occupancy, max_number_neighbors, min_number_neighbors, max_num_residuals, num_closest_neighbors, power_planarity;
bool log_print, estimate_normal_from_neighborhood;
std::list<std::pair<voxel, std::shared_ptr<voxelBlock3>>> grids_cache_;
/*Strengthening_LoopICPThreshold*/
// NOTE:
mapmanager Mapmanager;
std::pair<std::pair<int, int>, std::pair<int, float>> Mapmergingindex = {{0, 0}, {-1, 0}};
std::queue<std::pair<std::pair<int, int>, std::pair<int, float>>> MapmergingindexBuf;
std::queue<std::pair<int, int>> Strengthening_constraints;
Eigen::Affine3f transFinal;
Eigen::Affine3f pretrans1, pretrans2, pretrans3;
int SleepmapkeyframeposesNum = 0;
bool isMapmerging = false;
bool isMapsleeping = false;
bool isMapbuilding = false;
bool issimilaricp_calculation = true;
bool isposegraph_slam = true;
bool isrviz_view = true;
bool isicp_calculation = true;
bool sc_density;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mapmergind;

std::mutex mtxMapsleeping;
std::mutex mtxMapbuilding;
std::mutex mtxsimilaricp_calculation;
std::mutex mtxicp_calculation;
std::mutex mtxposegraph_slam;
std::mutex mtxMapmerging;
std::mutex mtxView;

int pre_merge_idx;
int cur_merge_idx;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudMapPGO(new pcl::PointCloud<pcl::PointXYZI>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;

double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal;
ros::Publisher pubSimilarScanLocal, pubSimilarScanLocal_withpre, pubSimilarScanLocal_withquatro, pubSimilarSubmapLocal;
ros::Publisher pubOdomRepubVerifier;
ros::Publisher pubkeyframecloud, pubisloop;
ros::Publisher pub_sc_image, pub_sc_vispoints;

std::string test_pre, test_aft;

std::shared_ptr<quatro<pcl::PointXYZI>> quatro_handler_ = nullptr;
bool use_optimized_matching_ = true;
bool estimat_scale_ = false;
int quatro_max_num_corres_ = 1000;
int quatro_max_iter_ = 50;
double quatro_distance_threshold_ = 50.0;
double fpfh_normal_radius_ = 0.60; // It should be 2.5 - 3.0 * `voxel_res`
double fpfh_radius_ = 0.50;        // It should be 5.0 * `voxel_res`
double noise_bound_ = 0.30;
double rot_gnc_factor_ = 1.20;
double rot_cost_diff_thr_ = 0.0001;

cv::Mat Eigen2Mat_density(const Eigen::MatrixXd &matrix)
{
    cv::Mat image(matrix.rows(), matrix.cols(), CV_8UC1);
    for (int i = 0; i < matrix.rows(); i++)
        for (int j = 0; j < matrix.cols(); j++)
        {
            image.at<uint8_t>(i, j) = static_cast<uint8_t>(matrix(i, j) * 255);
        }

    return image;
}

cv::Mat Eigen2Mat(Eigen::MatrixXd &matrix)
{
    // 首先检查矩阵是否为空
    if (matrix.size() == 0)
    {
        throw std::invalid_argument("Input matrix is empty");
    }

    // 创建一个临时的 cv::Mat 来计算 min 和 max 值
    cv::Mat tempMat(matrix.rows(), matrix.cols(), CV_64F);
    for (int i = 0; i < matrix.rows(); ++i)
    {
        for (int j = 0; j < matrix.cols(); ++j)
        {
            tempMat.at<double>(i, j) = matrix(i, j);
        }
    }

    double minVal, maxVal;
    cv::minMaxLoc(tempMat, &minVal, &maxVal);

    // 归一化处理
    if (maxVal != minVal)
        matrix = (matrix.array() - minVal) * 255.0 / (maxVal - minVal);
    else
        matrix = matrix.array() * 0;

    // 创建最终的 cv::Mat 图像
    cv::Mat image(matrix.rows(), matrix.cols(), CV_8UC1);
    for (int i = 0; i < matrix.rows(); ++i)
    {
        for (int j = 0; j < matrix.cols(); ++j)
        {
            image.at<uint8_t>(i, j) = static_cast<uint8_t>(matrix(i, j));
        }
    }

    return image;
}

struct RegistrationOutput
{
    bool is_valid_ = false;
    bool is_converged_ = false;
    double score_ = std::numeric_limits<double>::max();
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
};

void IsLoop_handler(const std_msgs::Bool::ConstPtr &msg)
{
    bool condition = msg->data;
    {
        // std::lock_guard<std::mutex> lock(mBuf);
        cout << "PGO 订阅到Bool消息: " << condition << endl;
        Is_balm = condition;
    }
}

void IssleepingCallback(const std_msgs::Bool::ConstPtr &msg)
{
    bool condition = msg->data;
    {
        std::lock_guard<std::mutex> lock(mtxMapsleeping);
        isMapsleeping = condition;
    }
}
void IsbuildingCallback(const std_msgs::Bool::ConstPtr &msg)
{
    bool condition = msg->data;
    {
        std::lock_guard<std::mutex> lock(mtxMapbuilding);
        isMapbuilding = condition;
    }
}

std::string padZeros(int val, int num_digits = 6)
{
    std::ostringstream out;
    out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
    return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D &p)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z));
} // Pose6DtoGTSAMPose3

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
    std::lock_guard<std::mutex> lock(mBuf);
    odometryBuf.push(_laserOdometry);
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes)
{
    std::lock_guard<std::mutex> lock(mBuf);
    fullResBuf.push(_laserCloudFullRes);
} // laserCloudFullResHandler

void initNoises(void)
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector priorNoiseVector6_o(6);
    priorNoiseVector6_o << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;
    priorNoise_o = noiseModel::Diagonal::Variances(priorNoiseVector6_o);

    gtsam::Vector odomNoiseVector6(6);
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    double loopNoiseScore = 0.0002;      // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    robustLoopNoise_o = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

} // initNoises

Pose6D getOdom(nav_msgs::Odometry::ConstPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw};
} // getOdom

SE3 pose6DToSE3(const Pose6D &pose)
{
    // 1. 生成旋转矩阵（Z-Y-X顺序：yaw-pitch-roll）
    Eigen::AngleAxisd rollAngle(pose.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pose.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(pose.yaw, Eigen::Vector3d::UnitZ());

    // 注意乘法顺序：yaw * pitch * roll
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();

    // 2. 平移向量
    Eigen::Vector3d translation(pose.x, pose.y, pose.z);

    // 3. 构造 SE3 对象
    return SE3(rotation_matrix, translation);
}

Pose6D diffTransformation(const Pose6D &_p1, const Pose6D &_p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta;
    SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles(SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<pcl::PointXYZI>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr cloudIn, const Pose6D tf)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

#pragma omp parallel for num_threads(8)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr local2globalwithAffine3f(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudIn, const Eigen::Affine3f &transCur)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

Pose6D posetran(const Pose6D &poseIn, const Eigen::Affine3f &transCur)
{
    Pose6D poseOut;
    Eigen::Affine3f poseIn_a = pcl::getTransformation(poseIn.x, poseIn.y, poseIn.z, poseIn.roll, poseIn.pitch, poseIn.yaw);
    Eigen::Affine3f poseout_a = transCur * poseIn_a;
    float tx, ty, tz, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(poseout_a, tx, ty, tz, roll, pitch, yaw);
    poseOut.x = tx;
    poseOut.y = ty;
    poseOut.z = tz;
    poseOut.roll = roll;
    poseOut.pitch = pitch;
    poseOut.yaw = yaw;
    return poseOut;
}

void addPointToMap(voxelNormalHashMap &map, const point3D &pt, double voxel_size,
                   int max_num_points_in_voxel, double min_distance_points,
                   int min_num_points)
{
    short kx = static_cast<short>(pt.point[0] / voxel_size);
    short ky = static_cast<short>(pt.point[1] / voxel_size);
    short kz = static_cast<short>(pt.point[2] / voxel_size);

    voxelNormalHashMap::iterator search = map.find(voxel(kx, ky, kz));

    auto calculateAngle = [&](const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
    {
        double dot_product = v1.dot(v2);
        return std::acos(dot_product) * 180.0 / M_PI; //   degree
    };

    if (search != map.end())
    {
        auto &voxel_block = (search.value());

        double angle = calculateAngle(pt.normal, voxel_block->second->normal); //   和平均主方向夹角
        if (angle > 100.0)
        {
            voxel_block->second.reset(new voxelBlock3(max_num_points_in_voxel));
            voxel_block->second->AddPoint(normalPoint(pt));
        }
        else if (!voxel_block->second->IsFull())
        {
            double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
            for (int i(0); i < voxel_block->second->NumPoints(); ++i)
            {
                auto &_point = voxel_block->second->points[i];
                double sq_dist = (_point.getPosition() - pt.point).squaredNorm();
                if (sq_dist < sq_dist_min_to_points)
                {
                    sq_dist_min_to_points = sq_dist;
                }
            }
            if (sq_dist_min_to_points > (min_distance_points * min_distance_points))
            {
                if (min_num_points <= 0 || voxel_block->second->NumPoints() >= min_num_points)
                {
                    voxel_block->second->AddPoint(normalPoint(pt));
                    // addPointToPcl(points_world, point, intensity);
                }
            }
        }
        grids_cache_.splice(grids_cache_.begin(), grids_cache_, search->second);
        map[voxel(kx, ky, kz)] = grids_cache_.begin();
    }
    else
    {
        if (min_num_points <= 0)
        {
            std::shared_ptr<voxelBlock3> block = std::make_shared<voxelBlock3>(max_num_points_in_voxel);
            block->AddPoint(normalPoint(pt));

            //   LRU cache
            grids_cache_.push_front({voxel(kx, ky, kz), std::move(block)});
            map.insert({voxel(kx, ky, kz), grids_cache_.begin()});
            // map[voxel(kx, ky, kz)] = std::move(block);
            if (map.size() > capacity_)
            {
                map.erase(grids_cache_.back().first);
                grids_cache_.pop_back();
            }
        }
    }
}

void pubPath(void)
{
    // pub odom and path
    nav_msgs::Odometry odomAftPGO;
    nav_msgs::Path pathAftPGO;
    pathAftPGO.header.frame_id = "camera_init";
    double lastKeyframeTime = 0.0;
    {
        std::lock_guard<std::mutex> lock(mKF);
        if (Mapmanager.ActiveMap->keyframeTimes.empty())
            return;
        lastKeyframeTime = Mapmanager.ActiveMap->keyframeTimes.back();
        for (int node_idx = 0; node_idx < int(Mapmanager.ActiveMap->keyframeposesUpdated.size()) - 1; ++node_idx)
        {
            const Pose6D &pose_est = Mapmanager.ActiveMap->keyframeposesUpdated.at(node_idx); // upodated poses

            nav_msgs::Odometry odomAftPGOthis;
            odomAftPGOthis.header.frame_id = "camera_init";
            odomAftPGOthis.child_frame_id = "/aft_pgo";
            odomAftPGOthis.header.stamp = ros::Time().fromSec(Mapmanager.ActiveMap->keyframeTimes.at(node_idx));
            odomAftPGOthis.pose.pose.position.x = pose_est.x;
            odomAftPGOthis.pose.pose.position.y = pose_est.y;
            odomAftPGOthis.pose.pose.position.z = pose_est.z;
            odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
            odomAftPGO = odomAftPGOthis;

            geometry_msgs::PoseStamped poseStampAftPGO;
            poseStampAftPGO.header = odomAftPGOthis.header;
            poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

            pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
            pathAftPGO.header.frame_id = "camera_init";
            pathAftPGO.poses.push_back(poseStampAftPGO);
        }
    }
    pubOdomAftPGO.publish(odomAftPGO); // last pose
    pubPathAftPGO.publish(pathAftPGO); // poses

    static tf::TransformBroadcaster br;
    static ros::Time last_tf_stamp;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
    q.setW(odomAftPGO.pose.pose.orientation.w);
    q.setX(odomAftPGO.pose.pose.orientation.x);
    q.setY(odomAftPGO.pose.pose.orientation.y);
    q.setZ(odomAftPGO.pose.pose.orientation.z);
    transform.setRotation(q);
    ros::Time stamp = ros::Time().fromSec(lastKeyframeTime);
    if (stamp != last_tf_stamp) {
        br.sendTransform(tf::StampedTransform(transform, stamp, "camera_init", "/aft_pgo"));
        last_tf_stamp = stamp;
    }
} // pubPath

void updatePoses(void)
{
    {
        std::lock_guard<std::mutex> lock(mKF);
        for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size()); ++node_idx)
        {
            Pose6D &p = Mapmanager.ActiveMap->keyframeposesUpdated[node_idx];
            p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
            p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
            p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
            p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
            p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
            p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
        }
    }
    {
        std::lock_guard<std::mutex> lock(mrecentIdxUpdated);
        recentIdxUpdated = int(Mapmanager.ActiveMap->keyframeposesUpdated.size()) - 1;
    }
} // updatePoses

void runISAM2opt(void)
{
    // try {
        std::lock_guard<std::mutex> lock(mtxPosegraph);
        // called when a variable added
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        isamCurrentEstimate = isam->calculateEstimate();
    // }
    // catch (const gtsam::IndeterminantLinearSystemException& e) {
    //     ROS_ERROR("ISAM2 optimization failed: Indeterminant linear system at variable %d", 
    //               int(e.nearVariable()));
    //     // Clear the graph to prevent further errors
    //     gtSAMgraph.resize(0);
    //     initialEstimate.clear();
    //     return;
    // }
    // catch (const std::exception& e) {
    //     ROS_ERROR("ISAM2 optimization failed: %s", e.what());
    //     gtSAMgraph.resize(0);
    //     initialEstimate.clear();
    //     return;
    // }
    updatePoses();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
        transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(),
        transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw());

    int numberOfCores = 8;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom->x + transCur(0, 1) * pointFrom->y + transCur(0, 2) * pointFrom->z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom->x + transCur(1, 1) * pointFrom->y + transCur(1, 2) * pointFrom->z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom->x + transCur(2, 1) * pointFrom->y + transCur(2, 2) * pointFrom->z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void FindNearKeyframesCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr nearKeyframes, const int key, const int submap_size)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, Pose6D>> frames;
    {
        std::lock_guard<std::mutex> lock(mKF);
        for (int i = -submap_size; i <= submap_size; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= int(Mapmanager.ActiveMap->keyframeClouds.size()))
                continue;
            if (int(Mapmanager.ActiveMap->keyframeClouds[keyNear]->size()) <= 0)
                continue;
            frames.emplace_back(Mapmanager.ActiveMap->keyframeClouds[keyNear],
                                Mapmanager.ActiveMap->keyframeposesUpdated[keyNear]);
        }
    }
    for (auto &[cloud, pose] : frames)
        *nearKeyframes += *local2global(cloud, pose);

    if (nearKeyframes->empty())
        return;
    // downsample near keyframes
    {
        std::lock_guard<std::mutex> lock(mtxICP);
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*nearKeyframes);
    }
}

void sleepmapFindNearKeyframesCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &nearKeyframes, const int &key, const int &submap_size, const int &sleepmap_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, Pose6D>> frames;
    {
        std::lock_guard<std::mutex> lock(mKF);
        for (int i = -submap_size; i <= submap_size; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= int(Mapmanager.SleepMapList[sleepmap_idx]->keyframeClouds.size()))
                continue;
            frames.emplace_back(Mapmanager.SleepMapList[sleepmap_idx]->keyframeClouds[keyNear],
                                Mapmanager.SleepMapList[sleepmap_idx]->keyframeposesUpdated[keyNear]);
        }
    }
    for (auto &[cloud, pose] : frames)
        *nearKeyframes += *local2global(cloud, pose);

    if (nearKeyframes->empty())
        return;
    // downsample near keyframes
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
    {
        std::lock_guard<std::mutex> lock(mtxICP);
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
    }
    *nearKeyframes = *cloud_temp;
}

void activemapFindNearKeyframesCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &nearKeyframes, const int &key, const int &submap_size)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    std::vector<std::pair<pcl::PointCloud<PointType>::Ptr, Pose6D>> frames;
    {
        std::lock_guard<std::mutex> lock(mKF);
        for (int i = -submap_size; i <= submap_size; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= int(Mapmanager.ActiveMap->keyframeClouds.size()))
                continue;
            frames.emplace_back(Mapmanager.ActiveMap->keyframeClouds[keyNear],
                                Mapmanager.ActiveMap->keyframeposesUpdated[keyNear]);
        }
    }
    for (auto &[cloud, pose] : frames)
        *nearKeyframes += *local2global(cloud, pose);

    if (nearKeyframes->empty())
        return;
    // downsample near keyframes
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
    {
        std::lock_guard<std::mutex> lock(mtxICP);
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
    }
    *nearKeyframes = *cloud_temp;
}

///  ===================  for search neighbor  ===================================================
using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

struct comparator
{
    bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
    {
        return std::get<0>(left) < std::get<0>(right);
    }
};
using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> searchNeighbors(const voxelNormalHashMap &map, const Eigen::Vector3d &point,
                                                                                        const Eigen::Vector3d &normal, int nb_voxels_visited,
                                                                                        double size_voxel_map, int max_num_neighbors,
                                                                                        int threshold_voxel_capacity, std::vector<voxel> *voxels)
{

    if (voxels != nullptr)
        voxels->reserve(max_num_neighbors);

    short kx = static_cast<short>(point[0] / size_voxel_map);
    short ky = static_cast<short>(point[1] / size_voxel_map);
    short kz = static_cast<short>(point[2] / size_voxel_map);

    auto calculateAngle = [&](const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
    {
        double dot_product = v1.dot(v2);
        return std::acos(dot_product) * 180.0 / M_PI; //   degree
    };

    priority_queue_t priority_queue;

    voxel voxel_temp(kx, ky, kz);
    for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
    {
        for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
        {
            for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
            {
                voxel_temp.x = kxx;
                voxel_temp.y = kyy;
                voxel_temp.z = kzz;

                auto search = map.find(voxel_temp);
                if (search != map.end())
                {
                    const auto &voxel_block = search.value();
                    if (voxel_block->second->NumPoints() < threshold_voxel_capacity)
                        continue;
                    for (int i(0); i < voxel_block->second->NumPoints(); ++i)
                    {
                        auto &neighbor = voxel_block->second->points[i];
                        double angle = calculateAngle(neighbor.getNormal(), normal);
                        if (angle > 100.0)
                            continue;

                        Eigen::Vector3d neighbor_point = neighbor.getPosition();
                        double distance = (neighbor_point - point).norm();
                        if (priority_queue.size() == max_num_neighbors)
                        {
                            if (distance < std::get<0>(priority_queue.top()))
                            {
                                priority_queue.pop();
                                priority_queue.emplace(distance, neighbor_point, voxel_temp);
                            }
                        }
                        else
                            priority_queue.emplace(distance, neighbor_point, voxel_temp);
                    }
                }
            }
        }
    }

    auto size = priority_queue.size();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
    if (voxels != nullptr)
    {
        voxels->resize(size);
    }
    for (auto i = 0; i < size; ++i)
    {
        closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
        if (voxels != nullptr)
            (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
        priority_queue.pop();
    }

    return closest_neighbors;
}

Neighborhood computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
{
    Neighborhood neighborhood;
    // Compute the normals
    Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
    for (auto &point : points)
    {
        barycenter += point;
    }

    barycenter /= (double)points.size();
    neighborhood.center = barycenter;

    Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
    for (auto &point : points)
    {
        for (int k = 0; k < 3; ++k)
            for (int l = k; l < 3; ++l)
                covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                           (point(l) - barycenter(l));
    }
    covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
    covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
    covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
    neighborhood.covariance = covariance_Matrix;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
    Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
    neighborhood.normal = normal;

    double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
    double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
    double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
    neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

    if (neighborhood.a2D != neighborhood.a2D)
    {
        throw std::runtime_error("error");
    }

    return neighborhood;
}

void addSurfCostFactor(voxelNormalHashMap &target_map,
                       std::vector<ceres::CostFunction *> &surf,
                       std::vector<point3D> &keypoints,
                       const SE3 predict_pose, int &iter,
                       Eigen::Matrix3d &mat_norm)
{
    auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                         Eigen::Vector3d &location, double &planarity_weight)
    {
        auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
        planarity_weight = std::pow(neighborhood.a2D, power_planarity);

        if (neighborhood.normal.dot(predict_pose.translation() - location) < 0)
        {
            neighborhood.normal = -1.0 * neighborhood.normal;
        }
        return neighborhood;
    };

    double lambda_weight = std::abs(weight_alpha);
    double lambda_neighborhood = std::abs(weight_neighborhood);
    const double kMaxPointToPlane = max_dist_to_plane_icp;
    const double sum = lambda_weight + lambda_neighborhood;

    lambda_weight /= sum;
    lambda_neighborhood /= sum;
    const short nb_voxels_visited = voxel_neighborhood;
    const int kThresholdCapacity = threshold_voxel_occupancy;

    size_t num = keypoints.size();
    int num_residuals = 0;

    double res_dist = max_dist_to_plane_icp;
    // if (iter < 1) //   首次距离，放宽点
    //      res_dist = 3 * res_dist;
    if (iter < 2)
        res_dist = 2.0 * res_dist;
    else if (iter < 3)
        res_dist = 1.5 * res_dist;

    for (int k = 0; k < num; k++)
    {
        auto &keypoint = keypoints[k];
        auto &raw_point = keypoint.raw_point;
        keypoint.point = predict_pose * raw_point;
        keypoint.normal = predict_pose.unit_quaternion() * keypoint.raw_normal;

        std::vector<voxel> voxels;
        auto vector_neighbors = searchNeighbors(target_map, keypoint.point,
                                                keypoint.normal, nb_voxels_visited,
                                                size_voxel_map,
                                                max_number_neighbors,
                                                kThresholdCapacity,
                                                estimate_normal_from_neighborhood
                                                    ? nullptr
                                                    : &voxels);

        if (vector_neighbors.size() < min_number_neighbors)
            continue;

        double weight;

        Eigen::Vector3d location = raw_point;
        auto neighborhood = estimatePointNeighborhood(vector_neighbors, location /*raw_point*/, weight);

        weight = lambda_weight * weight + lambda_neighborhood *
                                              std::exp(-(vector_neighbors[0] -
                                                         keypoint.point)
                                                            .norm() /
                                                       (kMaxPointToPlane *
                                                        min_number_neighbors));

        double point_to_plane_dist;
        std::set<voxel> neighbor_voxels;

        for (int i(0); i < num_closest_neighbors; ++i)
        {
            point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

            if (point_to_plane_dist < res_dist)
            {

                num_residuals++;

                Eigen::Vector3d norm_vector = neighborhood.normal;
                norm_vector.normalize();

                mat_norm += norm_vector * norm_vector.transpose(); //   统计法向量分布

                double norm_offset = -norm_vector.dot(vector_neighbors[i]);

                Eigen::Vector3d point_end = predict_pose.unit_quaternion().inverse() * keypoints[k].point -
                                            predict_pose.unit_quaternion().inverse() * predict_pose.translation();

                auto *cost_function = PointToPlaneFunctor::Create(vector_neighbors[0],
                                                                  point_end, norm_vector, weight);

                surf.push_back(cost_function);
            }
        }

        if (num_residuals >= max_num_residuals)
            break;
    }
}

double computeOverlap(voxelNormalHashMap &target_map,
                      std::vector<point3D> &surf_keypoints,
                      SE3 &predict_pose)
{
    auto pt2idx = [&](const Eigen::Vector3d &point, const double &voxel_size)
    {
        return voxel{short(point.x() / voxel_size),
                     short(point.y() / voxel_size),
                     short(point.z() / voxel_size)};
    };

    int occu_num = 0;
    for (auto &pt : surf_keypoints)
    {
        pt.point = predict_pose * pt.raw_point;
        voxel vox = pt2idx(pt.point, size_voxel_map);
        if (target_map.find(vox) == target_map.end())
        {
            ;
        }
        else
        {
            occu_num++;
        }
    }
    double ra = double(occu_num) / surf_keypoints.size();
    return ra;
}

void makeTargetMap(voxelNormalHashMap &target_map, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &map_cloud,
                   const int &key, const int &submap_size)
{
    for (int i = -submap_size; i <= submap_size; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= int(Mapmanager.ActiveMap->keyframePointClouds.size()))
            continue;
        SE3 T_WL = pose6DToSE3(Mapmanager.ActiveMap->keyframeposesUpdated[keyNear]);
        for (auto &point : Mapmanager.ActiveMap->keyframePointClouds[keyNear])
        {
            point.point = T_WL * point.raw_point;
            point.normal = T_WL.unit_quaternion() * point.raw_normal;
            point.normal.normalize();

            addPointToMap(target_map, point,
                          size_voxel_map, max_num_points_in_voxel,
                          min_distance_points, 0);

            PointType pt;
            pt.x = point.point[0];
            pt.y = point.point[1];
            pt.z = point.point[2];
            pt.intensity = point.intensity;
            map_cloud->push_back(pt);
        }
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZINormal>);

    // downSizeFilterICP.setLeafSize(0.1, 0.1, 0.1);
    // downSizeFilterICP.setInputCloud(map_cloud);
    // downSizeFilterICP.filter(*cloud_temp);
    // *map_cloud = *cloud_temp;
}

inline double AngularDistance(
    const Eigen::Quaterniond &q1,
    const Eigen::Quaterniond &q2)
{
    // 计算相对四元数
    Eigen::Quaterniond dq = q1.conjugate() * q2;
    dq.normalize(); // 保证数值稳定

    // w 分量可能略超出 [-1,1]，先 clamp
    double w = std::clamp(dq.w(), -1.0, 1.0);

    // 由 w = cos(theta/2) → theta = 2 * acos(w)
    return 2.0 * std::acos(w) * 180.0 / M_PI;
}

bool scan2map_optimize(voxelNormalHashMap &target_map,
                       std::vector<point3D> &surf_keypoints,
                       SE3 &predict_pose)
{
    Eigen::Quaterniond end_quat = predict_pose.unit_quaternion();
    Eigen::Vector3d end_t = predict_pose.translation();

    auto transformKeypoints = [&](std::vector<point3D> &point_frame)
    {
        Eigen::Matrix3d R = end_quat.normalized().toRotationMatrix();
        Eigen::Vector3d t = end_t;
#ifdef P_USE_TBB_
        tbb::parallel_for(size_t(0), point_frame.size(), [&](size_t id)
                          { point_frame[id].point = R * point_frame[id].raw_point + t;
        point_frame[id].normal = R *  point_frame[id].raw_normal; });
#else
        for (auto &keypoint : point_frame)
        {
            keypoint.point = R * keypoint.raw_point + t;
            keypoint.normal = R * keypoint.raw_normal;
        }
#endif
    };

    Eigen::Matrix3d mat_norm;
    int is_converge = 0;
    for (int iter(0); iter < max_iteration; iter++)
    {
        // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1 / (1.5e-3));
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        mat_norm.setZero();

        auto *parameterization = new ceres::EigenQuaternionParameterization();

        problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
        problem.AddParameterBlock(&end_t.x(), 3);

        std::vector<ceres::CostFunction *> surfFactor;
        addSurfCostFactor(target_map, surfFactor, surf_keypoints, predict_pose, iter, mat_norm);

        int surf_num = 0;
        if (log_print)
            std::cout << "get factor: " << surfFactor.size() << std::endl;
        for (auto &e : surfFactor)
        {
            surf_num++;
            problem.AddResidualBlock(e, loss_function, &end_t.x(), &end_quat.x());
        }
        //   release
        std::vector<ceres::CostFunction *>().swap(surfFactor);

        if (surf_num < min_num_residuals)
        {
            std::stringstream ss_out;
            ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
            ss_out << "[Optimization] number_of_residuals : " << surf_num << std::endl;
            std::cout << "ERROR: " << ss_out.str();
            return false;
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 5;
        options.num_threads = 6;
        options.minimizer_progress_to_stdout = false;
        options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        double diff_trans = 0, diff_rot = 0;
        diff_trans += (predict_pose.translation() - end_t).norm();
        diff_rot += AngularDistance(predict_pose.unit_quaternion(), end_quat);

        predict_pose = SE3(end_quat, end_t);

        if (diff_rot < thres_orientation_norm &&
            diff_trans < thres_translation_norm)
        {
            is_converge = 1;
            if (log_print)
                std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
            break;
        }
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(mat_norm);
    Eigen::Vector3d eig_vec = saes.eigenvalues();
    std::cout << ANSI_COLOR_YELLOW << "eigvalue:" << eig_vec.transpose()
              << " " << is_converge << ANSI_COLOR_RESET << std::endl;

    return eig_vec[0] > icp_eigval && is_converge == 1;
}

std::optional<gtsam::Pose3> doICP_enhance(int _loop_kf_idx, int _curr_kf_idx)
{
    voxelNormalHashMap target_voxel_map;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    makeTargetMap(target_voxel_map, targetCloud, _loop_kf_idx, history_kf_buff);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr curCloud(new pcl::PointCloud<pcl::PointXYZINormal>());

    SE3 current_pose = pose6DToSE3(Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx]);

    //   下采样
    std::vector<point3D> surf_keypoints;
    gridSampling(Mapmanager.ActiveMap->keyframePointClouds[_curr_kf_idx], surf_keypoints, surf_res);
    {
        std::mt19937_64 g;
        std::shuffle(surf_keypoints.begin(), surf_keypoints.end(), g);
    }

    //   增加计算重叠率
    double ratio = computeOverlap(target_voxel_map, surf_keypoints, current_pose);
    if (ratio < overlap_ratio)
    {
        std::cout << "overlap is " << ratio << std::endl;
        return std::nullopt;
    }

    bool opt_res = scan2map_optimize(target_voxel_map, surf_keypoints, current_pose);
    if (!opt_res)
        return std::nullopt;

    SE3 incre_pose = current_pose * pose6DToSE3(Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx]).inverse(); //   FIXME: 需要验证

    Eigen::Matrix4f transform = incre_pose.matrix().cast<float>();

    Eigen::Affine3f correctionLidarFrame(transform);
    float x, y, z, roll, pitch, yaw;

    Eigen::Affine3f twrong = pcl::getTransformation(Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].x,
                                                    Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].y,
                                                    Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].z,
                                                    Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].roll,
                                                    Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].pitch,
                                                    Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].yaw);
    Eigen::Affine3f tCorrect = correctionLidarFrame * twrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated[_loop_kf_idx]);

    return poseFrom.between(poseTo);
}

std::optional<gtsam::Pose3> doICPVirtualRelative(int _loop_kf_idx, int _curr_kf_idx)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cureKeyframeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr targetKeyframeCloud(new pcl::PointCloud<pcl::PointXYZI>());

    FindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 1);
    FindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, loopICP_historyKeyframeSearchNum);

    sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopScanLocal.publish(cureKeyframeCloudMsg);

    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(10);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(2);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*unused_result);

    double loopFitnessScoreThreshold = loopICPThreshold;
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        // std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    }
    else
    {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    {
        std::lock_guard<std::mutex> lock(mKF);
        Eigen::Affine3f tWrong = pcl::getTransformation(Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].x,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].y,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].z,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].roll,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].pitch,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].yaw);
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated[_loop_kf_idx]);
        return poseFrom.between(poseTo);
    }
}

std::optional<gtsam::Pose3> doICPsimliarOpt(int _loop_kf_idx, int _curr_kf_idx)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cureKeyframeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr targetKeyframeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    FindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 2);
    FindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, Strengthening_ICP_historyKeyframeSearchNum);
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(500);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*unused_result);
    double loopFitnessScoreThreshold = Strengthening_constraints_ICPThreshold; // user parameter but fixed low value is safe.
    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold)
    {
        // std::cout << "[Loop after merging maps] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    }
    else
    {
        // std::cout << "[Loop after merging maps] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    {
        std::lock_guard<std::mutex> lock(mKF);
        Eigen::Affine3f tWrong = pcl::getTransformation(Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].x,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].y,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].z,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].roll,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].pitch,
                                                        Mapmanager.ActiveMap->keyframeposesUpdated[_curr_kf_idx].yaw);
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated[_loop_kf_idx]);
        return poseFrom.between(poseTo);
    }
}

bool doICPslmilarmatch(int sleepmapindex, int sleepind, int activeind, Eigen::Affine3f pre)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cureKeyframeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cureKeyframeCloud_withsc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cureKeyframeCloud_withquatro(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr targetKeyframeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    activemapFindNearKeyframesCloud(cureKeyframeCloud, activeind, similarICP_historyKeyframeSearchNum);
    sleepmapFindNearKeyframesCloud(targetKeyframeCloud, sleepind, similarICP_historyKeyframeSearchNum, sleepmapindex);

    sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = "camera_init";
    pubSimilarScanLocal.publish(cureKeyframeCloudMsg);

    *cureKeyframeCloud_withsc = *local2globalwithAffine3f(cureKeyframeCloud, pre);

    sensor_msgs::PointCloud2 cureKeyframeCloudMsg_withpre;
    pcl::toROSMsg(*cureKeyframeCloud_withsc, cureKeyframeCloudMsg_withpre);
    cureKeyframeCloudMsg_withpre.header.frame_id = "camera_init";
    pubSimilarScanLocal_withpre.publish(cureKeyframeCloudMsg_withpre);

    sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = "camera_init";
    pubSimilarSubmapLocal.publish(targetKeyframeCloudMsg);

    if (1)
    {
        RegistrationOutput result;
        quatro_handler_ = std::make_shared<quatro<pcl::PointXYZI>>(fpfh_normal_radius_,
                                                                   fpfh_radius_,
                                                                   noise_bound_,
                                                                   rot_gnc_factor_,
                                                                   rot_cost_diff_thr_,
                                                                   quatro_max_iter_,
                                                                   estimat_scale_,
                                                                   use_optimized_matching_,
                                                                   quatro_distance_threshold_,
                                                                   quatro_max_num_corres_);

        Eigen::Matrix4d pose_between_ = quatro_handler_->align(*cureKeyframeCloud_withsc, *targetKeyframeCloud, result.is_converged_);
        if (!result.is_converged_)
        {
            cout << "Don't hold back, it's so embarrassing!" << "*******" << endl;
            return false;
        }
        else
        {
            Eigen::Affine3f correctionLidarFrame(pose_between_.cast<float>());
            *cureKeyframeCloud_withquatro = *local2globalwithAffine3f(cureKeyframeCloud_withsc, correctionLidarFrame);
            sensor_msgs::PointCloud2 cureKeyframeCloudMsg_withquatro;
            pcl::toROSMsg(*cureKeyframeCloud_withquatro, cureKeyframeCloudMsg_withquatro);
            cureKeyframeCloudMsg_withquatro.header.frame_id = "camera_init";
            pubSimilarScanLocal_withquatro.publish(cureKeyframeCloudMsg_withquatro);
            pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
            icp.setMaxCorrespondenceDistance(200);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(2);

            icp.setInputSource(cureKeyframeCloud_withquatro);
            icp.setInputTarget(targetKeyframeCloud);
            pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
            icp.align(*unused_result);

            double loopFitnessScoreThreshold = similarICPThreshold;
            if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold)
            {
                cout << "Don't hold back, it's so embarrassing!" << "*******" << (icp.getFitnessScore()) << endl;
                return false;
            }
            else
            {
                cout << "Hold back, this is great!" << "*******" << (icp.getFitnessScore()) << endl;
                Eigen::Affine3f correctionLidarFramefin;
                correctionLidarFramefin = icp.getFinalTransformation();
                {
                    std::lock_guard<std::mutex> lock(mtransFinal);
                    transFinal = correctionLidarFramefin * correctionLidarFrame * pre;
                }
                return true;
            }
        }
    }
    else
    {
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance(200);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(2);

        icp.setInputSource(cureKeyframeCloud_withsc);
        icp.setInputTarget(targetKeyframeCloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
        icp.align(*unused_result);

        double loopFitnessScoreThreshold = similarICPThreshold;
        if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold)
        {
            cout << "Don't hold back, it's so embarrassing!" << "*******" << (icp.getFitnessScore()) << endl;
            return false;
        }
        else
        {
            cout << "Hold back, this is great!" << "*******" << (icp.getFitnessScore()) << endl;
            Eigen::Affine3f correctionLidarFrame;
            correctionLidarFrame = icp.getFinalTransformation();
            {
                std::lock_guard<std::mutex> lock(mtransFinal);
                transFinal = correctionLidarFrame * pre;
            }
            return true;
        }
    }
}

void process_map(void)
{
    {
        std::lock_guard<std::mutex> lock(mKF);
        {
            std::lock_guard<std::mutex> lockTrans(mtransFinal);
            Mapmanager.transformmap(transFinal);
        }
        Mapmanager.mergemap();
        Mapmanager.sleep_delete();
        after_merge_idx = Mapmanager.ActiveMap->keyframeposes.size();
    }
    {
        std::lock_guard<std::mutex> lock(mtxPosegraph);
        delete isam;
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.01;
        optParameters.relinearizeSkip = 1;
        isam = new ISAM2(optParameters);
        gtsam::NonlinearFactorGraph newGraphFactors;
        gtSAMgraph = newGraphFactors;
        gtsam::Values NewGraphValues;
        initialEstimate = NewGraphValues;
    }
    gtsam::Pose3 poseOrigin;
    gtsam::Pose3 poseOrigin_o;
    {
        std::lock_guard<std::mutex> lock(mKF);
        poseOrigin = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(0));
        poseOrigin_o = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(sleepmapsize));
    }
    {
        std::lock_guard<std::mutex> lock(mtxPosegraph);
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, poseOrigin, priorNoise));
        initialEstimate.insert(0, poseOrigin);
        // gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(sleepmapsize, poseOrigin_o, priorNoise_o));
        // initialEstimate.insert(sleepmapsize, poseOrigin_o);
        for (int i = 1; i < sleepmapsize; ++i)
        {
            gtsam::Pose3 poseFrom;
            gtsam::Pose3 poseTo;
            {
                std::lock_guard<std::mutex> lockKF(mKF);
                poseFrom = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(i - 1));
                poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(i));
            }
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>((i - 1), (i), poseFrom.between(poseTo), odomNoise));
            initialEstimate.insert((i), poseTo);
        }
        {
            std::lock_guard<std::mutex> lockKF(mKF);
            initialEstimate.insert((sleepmapsize), Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(sleepmapsize)));
            for (int i = (sleepmapsize + 1); i < int(Mapmanager.ActiveMap->keyframeposesUpdated.size()); ++i)
            {
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(i - 1));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(i));
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>((i - 1), (i), poseFrom.between(poseTo), odomNoise));
                initialEstimate.insert((i), poseTo);
            }
        }
        auto simliarOpt_optional = doICPsimliarOpt(pre_merge_idx, cur_merge_idx);
        if (simliarOpt_optional)
        {
            gtsam::Pose3 relativeOpt_pose = simliarOpt_optional.value();
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cur_merge_idx, pre_merge_idx, relativeOpt_pose, robustLoopNoise_o));
        }
        else
        {
            ROS_WARN("doICPsimliarOpt failed in process_map! Using pre-aligned poses as fallback constraint to prevent graph disconnection.");
            
            gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(cur_merge_idx));
            gtsam::Pose3 poseTo   = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposesUpdated.at(pre_merge_idx));
            
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cur_merge_idx, pre_merge_idx, poseFrom.between(poseTo), robustLoopNoise_o));
        }
        int total_keyframes = Mapmanager.ActiveMap->keyframeposesUpdated.size();
        
        while (!Strengthening_constraints.empty())
        {
            cout << "PGO:" << Strengthening_constraints.size() << endl;
            
            int idx1 = Strengthening_constraints.front().first;
            int idx2 = Strengthening_constraints.front().second;
            
            if (idx1 < 0 || idx1 >= total_keyframes || idx2 < 0 || idx2 >= total_keyframes)
            {
                ROS_WARN("Skipping invalid Strengthening constraint: (%d, %d), total keyframes: %d", 
                         idx1, idx2, total_keyframes);
                Strengthening_constraints.pop();
                continue;
            }
            
            auto simliarOpt_additional = doICPsimliarOpt(idx1, idx2);
            if (simliarOpt_additional)
            {
                gtsam::Pose3 relativeOpt_poseadditional = simliarOpt_additional.value();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx2, idx1,
                                                                  relativeOpt_poseadditional,
                                                                  robustLoopNoise_o));
            }
            Strengthening_constraints.pop();
        }
    }

    {
        std::lock_guard<std::mutex> lock(mtxposegraph_slam);
        isposegraph_slam = true;
    }
    {
        std::lock_guard<std::mutex> lock(mtxsimilaricp_calculation);
        issimilaricp_calculation = true;
    }
    {
        std::lock_guard<std::mutex> lock(mtxicp_calculation);
        isicp_calculation = true;
    }

    SleepmapkeyframeposesNum = 0;
    {
        std::lock_guard<std::mutex> lock(mKF);
        cout << "Number of sleeping maps after fusion: " << Mapmanager.SleepMapList.size() << endl;
        cout << "The number of keyframes of the currently active map after fusion: " << Mapmanager.ActiveMap->keyframeposes.size() << endl;
    }
}

void performSimilarClosure(void)
{
    {
        std::lock_guard<std::mutex> lock(mKF);
        if (Mapmanager.SleepMapList.empty() || Mapmanager.ActiveMap->scmanager.polarcontext_invkeys_mat_.size() < 2)
        {
            return;
        }
    }
    std::vector<double> similar_mapindexes;
    std::vector<std::pair<int, float>> detectResult_sleepmap;
    {
        std::lock_guard<std::mutex> lock(mKF);
        for (size_t i = 0; i < Mapmanager.SleepMapList.size(); ++i)
        {
            detectResult_sleepmap.push_back(Mapmanager.SleepMapList[i]->scmanager.similar_detectLoopClosureID(Mapmanager.ActiveMap->scmanager.polarcontext_invkeys_mat_, Mapmanager.ActiveMap->scmanager.polarcontexts_));
            similar_mapindexes.push_back(Mapmanager.SleepMapList[i]->optimal_matchvalue);
        }
    }
    int min_index = 0;
    int min_value = similar_mapindexes[0];
    for (size_t i = 1; i < similar_mapindexes.size(); ++i)
    {
        if (similar_mapindexes[i] < min_value)
        {
            min_value = similar_mapindexes[i];
            min_index = i;
        }
    }
    int activeind;
    {
        std::lock_guard<std::mutex> lock(mKF);
        activeind = Mapmanager.ActiveMap->scmanager.polarcontexts_.size() - 1;
    }
    auto detectResult = detectResult_sleepmap[min_index];
    if (detectResult.first > 0)
    {
        {
            std::lock_guard<std::mutex> lock(mKF);
            Mapmanager.Wakeup_Mapindex = min_index;
        }
        std::pair<int, int> index{min_index, activeind};
        Mapmergingindex.first = index;
        Mapmergingindex.second = detectResult;
        {
            std::lock_guard<std::mutex> lock(mapmergind);
            MapmergingindexBuf.push(Mapmergingindex);
        }
    }
}

void performSCLoopClosure(void)
{
    {
        std::lock_guard<std::mutex> lock(mKF);
        if (int(Mapmanager.ActiveMap->keyframeposes.size()) < Mapmanager.ActiveMap->scmanager.NUM_EXCLUDE_RECENT) // do not try too early
        {
            return;
        }
    }
    auto detectResult = Mapmanager.ActiveMap->scmanager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first; // SCclosestHistoryFrameID
    float SCyaw_diff = detectResult.second;
    if (SCclosestHistoryFrameID != -1)
    {
        const int prev_node_idx = SCclosestHistoryFrameID;
        int curr_node_idx;
        {
            std::lock_guard<std::mutex> lock(mKF);
            curr_node_idx = Mapmanager.ActiveMap->keyframeposes.size() - 1;
        }
        // cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        {
            std::lock_guard<std::mutex> lock(SCBuf);
            scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx)); // scLoopICPBuf
            scLoopYaw_diff.push(SCyaw_diff);
        }
    }
}

void process_similaricp(void)
{
    float loopClosureFrequency = 1; // can change
    while (ros::ok())
    {
        bool should_process = false;
        {
            std::lock_guard<std::mutex> lock(mtxsimilaricp_calculation);
            should_process = issimilaricp_calculation;
        }
        if (should_process)
        {
            while (!MapmergingindexBuf.empty())
            {
                if (MapmergingindexBuf.size() > 30)
                {
                    cout << "silmiler_num" << MapmergingindexBuf.size() << endl;
                    ROS_WARN("Too many silmiler clousre is waiting ... Do process_similar less frequently (adjust loopClosureFrequency)");
                }

                std::pair<std::pair<int, int>, std::pair<int, float>> similar_pair;
                {
                    std::lock_guard<std::mutex> lock(mapmergind);
                    similar_pair = MapmergingindexBuf.front();
                    MapmergingindexBuf.pop();
                }
                std::pair<int, int> similar_idx_pair = similar_pair.second;
                const float rot = similar_idx_pair.second;
                const int sleepmap_node_idx = similar_idx_pair.first;
                const int activepmap_node_idx = similar_pair.first.second;
                int similarmapindex = similar_pair.first.first;
                {
                    std::lock_guard<std::mutex> lock(mKF);
                    Pose6D t1 = Mapmanager.SleepMapList[similarmapindex]->keyframeposesUpdated[sleepmap_node_idx];
                    Pose6D t2 = Mapmanager.SleepMapList[similarmapindex]->keyframeposesUpdated[(Mapmanager.SleepMapList[similarmapindex]->keyframeposesUpdated.size() - 1)];
                    Eigen::Affine3f sc_initial = pcl::getTransformation(0, 0, 0, 0, 0, rot);
                    Eigen::Affine3f pretrans_temp = pcl::getTransformation(t2.x, t2.y, t2.z, t2.roll, t2.pitch, 0);
                    pretrans3 = sc_initial * pretrans_temp;
                    pretrans2 = pcl::getTransformation(t2.x, t2.y, t2.z, t2.roll, t2.pitch, t2.yaw);
                    pretrans1 = pcl::getTransformation(t1.x, t1.y, t1.z, t1.roll, t1.pitch, t1.yaw);
                }
                if (doICPslmilarmatch(similarmapindex, sleepmap_node_idx, activepmap_node_idx, pcl::getTransformation(0, 0, 0, 0, 0, rot)))
                {
                    {
                        std::lock_guard<std::mutex> lock(mKF);
                        sleepmapsize = Mapmanager.SleepMapList[similarmapindex]->keyframeposes.size();
                    }
                    pre_merge_idx = sleepmap_node_idx;
                    cur_merge_idx = sleepmapsize + activepmap_node_idx;

                    {
                        std::lock_guard<std::mutex> lock(mtxposegraph_slam);
                        isposegraph_slam = false;
                    }
                    {
                        std::lock_guard<std::mutex> lock(mtxsimilaricp_calculation);
                        issimilaricp_calculation = false;
                    }
                    {
                        std::lock_guard<std::mutex> lock(mtxicp_calculation);
                        isicp_calculation = false;
                    }

                    int n = 0;
                    {
                        std::lock_guard<std::mutex> lock(mapmergind);
                        while (!MapmergingindexBuf.empty())
                        {
                            if (n <= 5)
                            {
                                std::pair<int, int> map_pair;
                                map_pair.first = MapmergingindexBuf.front().second.first;
                                map_pair.second = MapmergingindexBuf.front().first.second + sleepmapsize;
                                Strengthening_constraints.push(map_pair);
                            }
                            MapmergingindexBuf.pop();
                            n++;
                        }
                    }
                    TicToc time1;
                    process_map();
                    cout << "process_map time: " << time1.toc() << endl;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void process_map_surveillance(void)
{
    // while (1)
    // {
    bool sleeping = false;
    {
        std::lock_guard<std::mutex> lock(mtxMapsleeping);
        sleeping = isMapsleeping;
        if (isMapsleeping)
        {
            std::cout << "Whether to sleep the currently active map: " << isMapsleeping << std::endl;
            isMapsleeping = false;
        }
    }
    if (sleeping)
    {
        {
            std::lock_guard<std::mutex> lock(mtxposegraph_slam);
            isposegraph_slam = false;
        }
        {
            std::lock_guard<std::mutex> lock(mtxsimilaricp_calculation);
            issimilaricp_calculation = false;
        }
        {
            std::lock_guard<std::mutex> lock(mtxicp_calculation);
            isicp_calculation = false;
        }
        {
            std::lock_guard<std::mutex> lock(mtxView);
            isrviz_view = false;
        }

        {
            std::lock_guard<std::mutex> lock(mKF);
            Mapmanager.act_change();
        }
        {
            std::lock_guard<std::mutex> lock(mtransFinal);
            transFinal.setIdentity();
        }

        {
            std::lock_guard<std::mutex> lock(SCBuf);
            while (!scLoopICPBuf.empty())
            {
                scLoopICPBuf.pop();
            }
        }
    }

    bool building = false;
    {
        std::lock_guard<std::mutex> lock(mtxMapbuilding);
        building = isMapbuilding;
        if (isMapbuilding)
        {
            std::cout << "Whether to start a new active map: " << isMapbuilding << std::endl;
            isMapbuilding = false;
        }
    }
    if (building)
    {
        {
            std::lock_guard<std::mutex> lock(mtxposegraph_slam);
            isposegraph_slam = true;
        }
        {
            std::lock_guard<std::mutex> lock(mtxsimilaricp_calculation);
            issimilaricp_calculation = true;
        }
        {
            std::lock_guard<std::mutex> lock(mtxicp_calculation);
            isicp_calculation = true;
        }
        {
            std::lock_guard<std::mutex> lock(mtxView);
            isrviz_view = true;
        }

        {
            std::lock_guard<std::mutex> lock(mKF);
            Mapmanager.act_reset();
            Mapmanager.ActiveMap->scmanager.setSCdistThres(scDistThres);
            Mapmanager.ActiveMap->scmanager.setMaximumRadius(scMaximumRadius);
            Mapmanager.ActiveMap->scmanager.setSimilarCdistThres(SimilarSCdistThres);
        }
        {
            std::lock_guard<std::mutex> lock(mtxPosegraph);
            delete isam;
            gtsam::ISAM2Params optParameters;
            optParameters.relinearizeThreshold = 0.1;
            optParameters.relinearizeSkip = 1;
            isam = new ISAM2(optParameters);
            gtsam::NonlinearFactorGraph newGraphFactors;
            gtSAMgraph = newGraphFactors;
            gtsam::Values NewGraphValues;
            initialEstimate = NewGraphValues;
        }
        {
            std::lock_guard<std::mutex> lock(mgtSAMgraphMade);
            gtSAMgraphMade = false;
        }
        {
            std::lock_guard<std::mutex> lock(mrecentIdxUpdated);
            recentIdxUpdated = 0;
        }
    }
    // wait (must required for running the while loop)
    //     std::chrono::milliseconds dura(2);
    //     std::this_thread::sleep_for(dura);
    // }
}

void pubMap(void)
{
    int SKIP_FRAMES = 8; // sparse map visulalization to save computations
    int counter = 0;

    laserCloudMapPGO->clear();

    std::vector<pcl::PointCloud<PointType>::Ptr> clouds_to_stitch;
    std::vector<Pose6D> poses_to_stitch;

    {
        std::lock_guard<std::mutex> lock(mKF);
        for (int node_idx = 0; node_idx < int(Mapmanager.ActiveMap->keyframeposesUpdated.size()); ++node_idx)
        {
            if (counter % SKIP_FRAMES == 0)
            {
                clouds_to_stitch.push_back(Mapmanager.ActiveMap->keyframeClouds[node_idx]);
                poses_to_stitch.push_back(Mapmanager.ActiveMap->keyframeposesUpdated[node_idx]);
            }
            counter++;
        }
    }

    for (size_t i = 0; i < clouds_to_stitch.size(); ++i)
    {
        *laserCloudMapPGO += *local2global(clouds_to_stitch[i], poses_to_stitch[i]);
    }

    if (!laserCloudMapPGO->empty())
    {
        // pcl::PointCloud<pcl::PointXYZI>::Ptr thisKeyFrame_fir(new pcl::PointCloud<pcl::PointXYZI>());
        // *thisKeyFrame_fir = *laserCloudMapPGO;
    }

    if (Is_balm)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_wait_save(new pcl::PointCloud<pcl::PointXYZI>());
        *pcl_wait_save = *laserCloudMapPGO;
        pcl_wait_save->height = 1;
        pcl_wait_save->width = pcl_wait_save->size();
        
        cout << "begin saving massive PCD map..." << endl;
        std::string filename = "/home/myx/Fighting/Indoor-Multi-Session/src/Indoor_Multi_session/PCD/scans/before_balm.pcd";
        
        pcl::io::savePCDFileBinary(filename, *(pcl_wait_save));
        
        cout << "end saving!!" << endl;
        Is_balm = false;
    }

    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "camera_init";
    laserCloudMapPGOMsg.header.stamp = ros::Time::now(); // 补充了时间戳
    pubMapAftPGO.publish(laserCloudMapPGOMsg);
}
void process_pg()
{
    while (1)
    {
        process_map_surveillance();
        bool should_process = false;
        {
            std::lock_guard<std::mutex> lock(mtxposegraph_slam);
            should_process = isposegraph_slam;
        }
        if (should_process)
        {
            while (!odometryBuf.empty() && !fullResBuf.empty())
            {
                double timeLaserOdometry_local;
                double timeLaser_local;
                ros::Time time;
                pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
                Pose6D pose_curr;

                {
                    std::lock_guard<std::mutex> lock(mBuf);
                    while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < fullResBuf.front()->header.stamp.toSec())
                        odometryBuf.pop();
                    if (odometryBuf.empty())
                    {
                        break;
                    }

                    timeLaserOdometry_local = odometryBuf.front()->header.stamp.toSec();
                    timeLaser_local = fullResBuf.front()->header.stamp.toSec();
                    time = fullResBuf.front()->header.stamp;

                    laserCloudFullRes->clear();
                    pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
                    fullResBuf.pop();

                    pose_curr = getOdom(odometryBuf.front());

                    odometryBuf.pop();
                }

                {
                    std::lock_guard<std::mutex> lock(mtransFinal);
                    pose_curr = posetran(pose_curr, transFinal);
                }
                odom_pose_prev = odom_pose_curr;
                odom_pose_curr = pose_curr;
                Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

                double delta_translation = sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z); // note: absolute value.
                translationAccumulated += delta_translation;
                rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.

                if (translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap)
                {
                    isNowKeyFrame = true;
                    translationAccumulated = 0.0; // reset
                    rotaionAccumulated = 0.0;     // reset
                }
                else
                {
                    isNowKeyFrame = false;
                }

                if (!isNowKeyFrame)
                    continue;
                cnt++;
                pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
                downSizeFilterScancontext.setInputCloud(thisKeyFrame);
                downSizeFilterScancontext.filter(*thisKeyFrameDS);

                std::vector<point3D> key_points;
                for (int i = 0; i < thisKeyFrameDS->size(); i++)
                {
                    point3D point_temp;
                    point_temp.raw_point = Eigen::Vector3d(thisKeyFrameDS->points[i].x, thisKeyFrameDS->points[i].y, thisKeyFrameDS->points[i].z);
                    point_temp.point = point_temp.raw_point;
                    point_temp.raw_normal = Eigen::Vector3d(thisKeyFrameDS->points[i].normal_x, thisKeyFrameDS->points[i].normal_y, thisKeyFrameDS->points[i].normal_z);
                    point_temp.normal = point_temp.raw_normal;
                    point_temp.relative_time = 0.0; // curvature unit: ms
                    point_temp.intensity = thisKeyFrameDS->points[i].intensity;

                    point_temp.timestamp = timeLaser_local;
                    point_temp.alpha_time = 0.0;
                    point_temp.timespan = 0.0;
                    point_temp.ring = 0;
                    point_temp.lid = 1;

                    key_points.push_back(point_temp);
                }

                sensor_msgs::PointCloud2 keyframecloud_msg;
                pcl::toROSMsg(*thisKeyFrame, keyframecloud_msg);
                keyframecloud_msg.header.stamp = time;
                keyframecloud_msg.header.frame_id = "body";
                pubkeyframecloud.publish(keyframecloud_msg);

                cv::Mat sc_image_cv;
                cv::Mat colorImage;
                pcl::PointCloud<pcl::PointXYZI> vis_points;
                {
                    std::lock_guard<std::mutex> lock(mKF);
                    if (sc_density)
                    {
                        vis_points = Mapmanager.addkeyframewithScancontext_density(thisKeyFrameDS, key_points, pose_curr, timeLaserOdometry_local);
                        Eigen::MatrixXd sc_image = Mapmanager.ActiveMap->scmanager.polarcontexts_.back();
                        sc_image_cv = Eigen2Mat_density(sc_image);
                    }else
                    {
                        vis_points = Mapmanager.addkeyframewithScancontext(thisKeyFrameDS, key_points, pose_curr, timeLaserOdometry_local);
                        Eigen::MatrixXd sc_image = Mapmanager.ActiveMap->scmanager.polarcontexts_.back();
                        sc_image_cv = Eigen2Mat(sc_image);
                    }
                }
                cv::applyColorMap(sc_image_cv, colorImage, cv::COLORMAP_JET);
                std_msgs::Header header;
                header.frame_id = "camera_init";
                sensor_msgs::ImagePtr sc_image_msg = cv_bridge::CvImage(header, "bgr8", colorImage).toImageMsg();
                pub_sc_image.publish(sc_image_msg);
                //  发布SC可视化点云
                sensor_msgs::PointCloud2 vis_points_msg;
                pcl::toROSMsg(vis_points, vis_points_msg);
                vis_points_msg.header.frame_id = "camera_init";
                pub_sc_vispoints.publish(vis_points_msg);

                SleepmapkeyframeposesNum = 0;
                {
                    std::lock_guard<std::mutex> lock(mKF);
                    for (size_t i = 0; i < Mapmanager.SleepMapList.size(); ++i)
                    {
                        SleepmapkeyframeposesNum += Mapmanager.SleepMapList[i]->keyframeposes.size();
                    }
                }
                int prev_node_idx;
                int curr_node_idx;
                {
                    std::lock_guard<std::mutex> lock(mKF);
                    prev_node_idx = Mapmanager.ActiveMap->keyframeposes.size() - 2;
                    curr_node_idx = Mapmanager.ActiveMap->keyframeposes.size() - 1;
                }
                // cout << "curr_node_idx,after_merge_idx" << curr_node_idx << "," << after_merge_idx << endl;
                // if (curr_node_idx == after_merge_idx)
                //     break;

                bool graph_made;
                {
                    std::lock_guard<std::mutex> lock(mgtSAMgraphMade);
                    graph_made = gtSAMgraphMade;
                }
                if (!graph_made /* prior node */)
                {
                    const int init_node_idx = 0;
                    gtsam::Pose3 poseOrigin;
                    {
                        std::lock_guard<std::mutex> lock(mKF);
                        poseOrigin = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposes.at(0));
                    }
                    {
                        std::lock_guard<std::mutex> lock(mtxPosegraph);
                        // prior factor
                        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                        initialEstimate.insert(init_node_idx, poseOrigin);
                        // runISAM2opt();
                    }

                    {
                        std::lock_guard<std::mutex> lock(mgtSAMgraphMade);
                        gtSAMgraphMade = true;
                    }
                    // cout << "posegraph prior node " << init_node_idx << " added" << endl; //
                }
                else /* consecutive node (and odom factor) after the prior added */
                {    // == keyframePoses.size() > 1
                    gtsam::Pose3 poseFrom;
                    gtsam::Pose3 poseTo;
                    {
                        std::lock_guard<std::mutex> lock(mKF);
                        poseFrom = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposes.at(prev_node_idx));
                        poseTo = Pose6DtoGTSAMPose3(Mapmanager.ActiveMap->keyframeposes.at(curr_node_idx));
                    }
                    {
                        std::lock_guard<std::mutex> lock(mtxPosegraph);
                        // odom factor
                        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>((prev_node_idx), (curr_node_idx), poseFrom.between(poseTo), odomNoise));
                        initialEstimate.insert((curr_node_idx), poseTo);
                    }
                    if ((curr_node_idx) % 100 == 0)
                        cout << "posegraph odom node " << (curr_node_idx) << " added." << endl;
                }
                performSCLoopClosure();
                if (cnt % SKIP_similar == 0)
                {
                    performSimilarClosure();
                }
                {
                    std::lock_guard<std::mutex> lock(mgtSAMgraphMade);
                    if (gtSAMgraphMade)
                    {
                        runISAM2opt();
                        // cout << "running isam2 optimization ..." << endl;
                    }
                }
            }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void process_icp(void)
{
    float loopClosureFrequency = 1; // can change
    while (ros::ok())
    {
        bool should_process = false;
        {
            std::lock_guard<std::mutex> lock(mtxicp_calculation);
            should_process = isicp_calculation;
        }
        if (should_process)
        {
            while (true)
            {
                std::pair<int, int> loop_idx_pair;
                bool has_loop = false;

                // 仅仅在出队时加锁！
                {
                    std::lock_guard<std::mutex> lock(SCBuf);
                    if (!scLoopICPBuf.empty())
                    {
                        if (scLoopICPBuf.size() > 30)
                        {
                            ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
                        }
                        loop_idx_pair = scLoopICPBuf.front();
                        scLoopYaw_diff.pop();
                        scLoopICPBuf.pop();
                        has_loop = true;
                    }
                }

                if (!has_loop) break; // 队列空了，退出内部while

                const int prev_node_idx = loop_idx_pair.first;
                const int curr_node_idx = loop_idx_pair.second;
                
                auto relative_pose_optional = doICP_enhance(prev_node_idx, curr_node_idx);
                
                if (relative_pose_optional)
                {
                    gtsam::Pose3 relative_pose = relative_pose_optional.value();
                    {
                        std::lock_guard<std::mutex> lock(mtxPosegraph);
                        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx, prev_node_idx, relative_pose, robustLoopNoise));
                        // runISAM2opt();
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
} // process_icp

void process_view(void)
{
    float vizmapFrequency = 0.1; // 0.1 means run onces every 10s
    while (1)
    {
        bool should_view = false;
        {
            std::lock_guard<std::mutex> lock(mtxView);
            should_view = isrviz_view;
        }
        if (should_view)
        {
            bool should_pub = false;
            {
                std::lock_guard<std::mutex> lock(mrecentIdxUpdated);
                should_pub = recentIdxUpdated > 10;
            }
            if (should_pub)
            {
                pubPath();
                pubMap();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
} // pointcloud_viz

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserPGO");
    ros::NodeHandle nh;

    nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 0.5);
    nh.param<double>("keyframe_deg_gap", keyframeDegGap, 5.0);
    keyframeRadGap = rad2deg_(keyframeDegGap);
    nh.param<double>("sc_dist_thres", scDistThres, 0.6);
    nh.param<double>("sc_max_radius", scMaximumRadius, 80.0); // 80 is recommended for outdoor, and lower (ex, 20, 40) values are recommended for indoor
    nh.param<double>("similar_sc_dist_thres", SimilarSCdistThres, 0.1);
    nh.param<double>("desc_THRES", desc_THRES, 25);
    nh.param<double>("loopICPThreshold", loopICPThreshold, 0.6);
    nh.param<int>("loopICP_historyKeyframeSearchNum", loopICP_historyKeyframeSearchNum, 3);

    nh.param<double>("size_voxel_map", size_voxel_map, 0.6);
    nh.param<int>("max_num_points_in_voxel", max_num_points_in_voxel, 1);
    nh.param<double>("min_distance_points", min_distance_points, 0.6);
    nh.param<int>("max_iteration", max_iteration, 10);
    nh.param<bool>("log_print", log_print, false);
    nh.param<int>("min_num_residuals", min_num_residuals, 50);
    nh.param<double>("weight_alpha", weight_alpha, 0.9);
    nh.param<double>("weight_neighborhood", weight_neighborhood, 0.1);
    nh.param<double>("max_dist_to_plane_icp", max_dist_to_plane_icp, 0.15);
    nh.param<int>("voxel_neighborhood", voxel_neighborhood, 1);
    nh.param<int>("threshold_voxel_occupancy", threshold_voxel_occupancy, 1);
    nh.param<double>("thres_orientation_norm", thres_orientation_norm, 0.1);
    nh.param<double>("thres_translation_norm", thres_translation_norm, 0.01);
    nh.param<int>("max_number_neighbors", max_number_neighbors, 20);
    nh.param<bool>("estimate_normal_from_neighborhood", estimate_normal_from_neighborhood, true);
    nh.param<int>("min_number_neighbors", min_number_neighbors, 20);
    nh.param<int>("max_num_residuals", max_num_residuals, 1000);
    nh.param<int>("num_closest_neighbors", num_closest_neighbors, 1);
    nh.param<int>("power_planarity", power_planarity, 2);
    nh.param<double>("surf_res", surf_res, 0.3);
    nh.param<double>("icp_eigval", icp_eigval, 60.0);
    nh.param<int>("history_kf_buff", history_kf_buff, 4);
    nh.param<double>("overlap_ratio", overlap_ratio, 0.35);
    nh.param<int>("capacity_", capacity_, 4000000);

    nh.param<double>("similarICPThreshold", similarICPThreshold, 0.3);
    nh.param<int>("similarICP_historyKeyframeSearchNum", similarICP_historyKeyframeSearchNum, 3);

    nh.param<double>("Strengthening_constraints_ICPThreshold", Strengthening_constraints_ICPThreshold, 0.6);
    nh.param<int>("Strengthening_ICP_historyKeyframeSearchNum", Strengthening_ICP_historyKeyframeSearchNum, 3);

    nh.param<bool>("sc_density", sc_density, 0);

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();
    transFinal.setIdentity();
    pretrans1.setIdentity();
    pretrans2.setIdentity();
    pretrans3.setIdentity();

    Mapmanager.ActiveMap->scmanager.setSCdistThres(scDistThres);
    Mapmanager.ActiveMap->scmanager.setMaximumRadius(scMaximumRadius);
    Mapmanager.ActiveMap->scmanager.setSimilarCdistThres(SimilarSCdistThres);
    Mapmanager.ActiveMap->scmanager.setdesc_THRES(desc_THRES);

    double filter_size;
    nh.param<double>("filter_size", filter_size, 0.4);
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    double mapVizFilterSize;
    nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.4); // pose assignment every k frames
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered_local", 100, laserCloudFullResHandler);
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, laserOdometryHandler);

    ros::Subscriber subissleep = nh.subscribe<std_msgs::Bool>("Issleeping", 10, IssleepingCallback);
    ros::Subscriber subisbuild = nh.subscribe<std_msgs::Bool>("Isbuilding", 10, IsbuildingCallback);
    ros::Subscriber sub_isloop = nh.subscribe<std_msgs::Bool>("/IsLoop", 1000, IsLoop_handler);

    pubisloop = nh.advertise<std_msgs::Bool>("/IsLoop", 100);
    pubkeyframecloud = nh.advertise<sensor_msgs::PointCloud2>("/keyframe_cloud", 100);

    pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
    pubOdomRepubVerifier = nh.advertise<nav_msgs::Odometry>("/repub_odom", 100);
    pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
    pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);

    pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
    pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);

    pubSimilarScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/similar_scan_local", 100);
    pubSimilarScanLocal_withpre = nh.advertise<sensor_msgs::PointCloud2>("/similar_scan_local_withpre", 100);
    pubSimilarScanLocal_withquatro = nh.advertise<sensor_msgs::PointCloud2>("/similar_scan_local_withquatro", 100);
    pubSimilarSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/similar_submap_local", 100);
    
    pub_sc_image = nh.advertise<sensor_msgs::Image>("/sc_image", 2000);
    pub_sc_vispoints = nh.advertise<sensor_msgs::PointCloud2>("/sc_vispoints", 100);
    
    std::thread posegraph_slam{process_pg};
    std::thread icp_calculation{process_icp};
    std::thread similaricp_calculation{process_similaricp};
    // std::thread map_merging{process_map};
    std::thread rviz_view{process_view};
    // std::thread map_surveillance{process_map_surveillance};
    ros::spin();
    return 0;
}
