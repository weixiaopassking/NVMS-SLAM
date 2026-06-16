// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>
#include <csignal>
// ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>


#include <pcl/filters/voxel_grid.h>

#include "common/utility.h"
// #include "preprocess/cloud_convert/cloud_convert.h"
#include "preprocess/cloud_convert2/cloud_convert2.h"

#define MAP_NORMAL 1

#ifdef MAP_NORMAL
#include "lio/loam_tbb_ic_normal/lidarodom.h"
zjloc::loam_tbbic_normal::lidarodom *lio;
#else
#include "lio/loam_tbb_ic/lidarodom.h"
zjloc::loam_tbbic::lidarodom *lio;
#endif

nav_msgs::Path laserOdoPath;

zjloc::CloudConvert2 *convert;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZINormal>());
int map_id = 0;
std::string filename = "/home/myx/Fighting/Indoor-Multi-Session/src/Indoor_Multi_session/PCD/scans/"+ std::to_string(9)+ ".pcd";
double g_gravity = 9.805;
std::condition_variable sig_buffer; 
bool flg_exit = false;

DEFINE_string(config_directory, "/home/myx/Fighting/Indoor-Multi-Session/src/Indoor_Multi_session/NV-LIO/config/", "配置文件目录");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

ros::Publisher pubisBalm;
void SigHandle(int sig)
{
    flg_exit = true;
    cout << "进入BALM" << endl;
    std_msgs::Bool isBalm;
    isBalm.data = true;
    pubisBalm.publish(isBalm);
    pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterMap;
    downSizeFilterMap.setLeafSize(0.1, 0.1, 0.1);
    downSizeFilterMap.setInputCloud(clouds);
    downSizeFilterMap.filter(*clouds);
    clouds->height = 1;
    clouds->width = clouds->size();
    pcl::io::savePCDFileBinary(filename, *(clouds));
    // 添加 ROS 关闭调用
    ros::shutdown();  // 关键修复：使 ros::spin() 退出
    // 通知其他线程
    sig_buffer.notify_all(); 
    // 添加日志以便调试
    cout << "ROS shutdown initiated" << endl;
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{

    std::vector<point3D> cloud_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_out); },
                                   "laser convert");

    zjloc::common::Timer::Evaluate([&]()
                                   {  
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.02;
        // double sample_size = 0.01;
        std::mt19937_64 g;
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame2(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                   "laser ds");

    lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), convert->getTimeSpan()));
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // std::string laser_file = std::string(ROOT_DIR) + "laser.txt";
    // static std::ofstream fout(laser_file, std::ios::out | std::ios::app);
    // fout << std::setprecision(18) << (msg->header.stamp.toSec() - ros::Time::now().toSec()) * 1000 << " ms" << std::endl;

    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    static int c = 0;
    // if (c % 2 == 0 && use_velodyne)
    {
        std::vector<point3D> cloud_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert->Process(msg, cloud_out); },
                                       "laser convert");

        double sample_size = lio->getIndex() < 30 ? 0.02 : 0.1;

        zjloc::common::Timer::Evaluate([&]()
                                       { 
            // double sample_size = 0.01;
            std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            subSampleFrame2(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                       "laser ds");

        lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), convert->getTimeSpan()));
    }
    c++;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    // std::string imu_file = std::string(ROOT_DIR) + "imu.txt";
    // static std::ofstream fout2(imu_file, std::ios::out | std::ios::app);
    // fout2 << std::setprecision(18) << (msg->header.stamp.toSec() - ros::Time::now().toSec()) * 1000 << " ms" << std::endl;

    Vec3d acc = Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    acc = acc * g_gravity / acc.norm();
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.toSec(),
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), acc);
    lio->pushData(imu);
}

void updateStatus(const std_msgs::Int32::ConstPtr &msg)
{
    int type = msg->data;
    if (type == 1)
    {
    }
    else if (type == 2)
    {
    }
    else if (type == 3)
        ;
    else if (type == 4)
        ;
    else
        ;
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::cout << "config file:" << FLAGS_config_directory << std::endl;

    std::string config_file = std::string(FLAGS_config_directory) + "/mapping.yaml";
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;
#ifdef MAP_NORMAL
    lio = new zjloc::loam_tbbic_normal::lidarodom();
#else
    lio = new zjloc::loam_tbbic::lidarodom();
#endif
    if (!lio->init(config_file))
    {
        return -1;
    }

    ros::Publisher pub_scan = nh.advertise<sensor_msgs::PointCloud2>("scan", 10);
    ros::Publisher pub_normal_scan = nh.advertise<sensor_msgs::PointCloud2>("normal_scan", 10);
    auto cloud_pub_func = std::function<bool(std::string & topic_name, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud, double time)>(
        [&](std::string &topic_name, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud, double time)
        {
            sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ptr_output);

            cloud_ptr_output->header.stamp = ros::Time().fromSec(time);
            cloud_ptr_output->header.frame_id = "map";
            if (topic_name == "laser"){
                    pub_scan.publish(*cloud_ptr_output);
                    *clouds += *cloud;
                }   
            else
                pub_normal_scan.publish(*cloud_ptr_output);; // publisher_.publish(*cloud_ptr_output);
            return true;
        }

    );

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/lio_odom", 100);
    ros::Publisher pubLaserOdometryPath = nh.advertise<nav_msgs::Path>("/odometry_path", 5);
    pubisBalm = nh.advertise<std_msgs::Bool>("/IsLoop", 100);
    signal(SIGINT, SigHandle);
    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, double stamp)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            Eigen::Quaterniond q_current(pose.so3().matrix());
            transform.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            if (topic_name == "laser")
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "base_link"));

                // publish odometry
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "map";
                laserOdometry.child_frame_id = "base_link";
                laserOdometry.header.stamp = ros::Time().fromSec(stamp);

                laserOdometry.pose.pose.orientation.x = q_current.x();
                laserOdometry.pose.pose.orientation.y = q_current.y();
                laserOdometry.pose.pose.orientation.z = q_current.z();
                laserOdometry.pose.pose.orientation.w = q_current.w();
                laserOdometry.pose.pose.position.x = pose.translation().x();
                laserOdometry.pose.pose.position.y = pose.translation().y();
                laserOdometry.pose.pose.position.z = pose.translation().z();
                pubLaserOdometry.publish(laserOdometry);

                { //  FIXME:
                    std::ofstream fout(std::string(FLAGS_config_directory) + "/traj_pose.txt", std::ios::app);
                    fout << std::setprecision(18) << stamp << " " << std::setprecision(7) << pose.translation()[0] << " " << pose.translation()[1]
                         << " " << pose.translation()[2] << " " << q_current.x() << " "
                         << q_current.y() << " " << q_current.z() << " " << q_current.w() << std::endl;
                }

                //  publish path
                geometry_msgs::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                laserOdoPath.header.stamp = laserOdometry.header.stamp;
                laserOdoPath.poses.push_back(laserPose);
                laserOdoPath.header.frame_id = "/map";
                pubLaserOdometryPath.publish(laserOdoPath);
            }
            else if (topic_name == "world")
            {
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "world", "map"));
            }
            return true;
        }

    );

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("/velocity", 1);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("/move_dist", 1);

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string &topic_name, double time1, double time2)
        {
            std_msgs::Float32 time_rviz;

            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub.publish(time_rviz);
            else
                dist_pub.publish(time_rviz);

            return true;
        }
    );

    ros::Publisher pubissleeping = nh.advertise<std_msgs::Bool>("Issleeping", 10);
    ros::Publisher pubisbuilding = nh.advertise<std_msgs::Bool>("Isbuilding", 10);
    auto bool_pub_func = std::function<bool(std::string & topic_name, std_msgs::Bool isbegin_seeeion, double time2)>(
        [&](std::string &topic_name, std_msgs::Bool isbegin_seeeion, double time)
        {
            std_msgs::Bool Issleeping;

            Issleeping = isbegin_seeeion;
            if (topic_name == "Issleeping"){
                pubissleeping.publish(Issleeping);
                pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterMap;
                downSizeFilterMap.setLeafSize(0.1, 0.1, 0.1);
                downSizeFilterMap.setInputCloud(clouds);
                downSizeFilterMap.filter(*clouds);
                clouds->height = 1;
                clouds->width = clouds->size();
                std::string file = "/home/myx/Fighting/Indoor-Multi-Session/src/Indoor_Multi_session/PCD/scans/"+ std::to_string(map_id)+ ".pcd";
                map_id++;
                pcl::io::savePCDFileBinary(file, *(clouds));
                clouds->clear();
            }
            else
                pubisbuilding.publish(Issleeping);

            return true;
        }
    );

    lio->setFunc(cloud_pub_func);
    lio->setFunc(pose_pub_func);
    lio->setFunc(data_pub_func);
    lio->setFunc(bool_pub_func);
    convert = new zjloc::CloudConvert2;
    convert->LoadFromYAML(config_file);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
    std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    g_gravity = yaml["common"]["gravity"].as<double>();

    ros::Subscriber subLaserCloud = convert->lidar_type_ == zjloc::LidarType::AVIA
                                        ? nh.subscribe(laser_topic, 100, livox_pcl_cbk)
                                        : nh.subscribe<sensor_msgs::PointCloud2>(laser_topic, 100, standard_pcl_cbk);

    ros::Subscriber sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, imuHandler);

    ros::Subscriber sub_type = nh.subscribe<std_msgs::Int32>("/change_status", 2, updateStatus);

#ifdef MAP_NORMAL
    std::thread measurement_process(&zjloc::loam_tbbic_normal::lidarodom::run, lio);
    std::cout << ANSI_COLOR_GREEN_BOLD << " lidarodom use tbb normal. " << ANSI_COLOR_RESET << std::endl;
#else
    std::thread measurement_process(&zjloc::loam_tbbic::lidarodom::run, lio);
    std::cout << ANSI_COLOR_GREEN_BOLD << " lidarodom use tbb. " << ANSI_COLOR_RESET << std::endl;
#endif

    ros::spin();

    zjloc::common::Timer::PrintAll();
    // zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));
    zjloc::common::Timer::DumpIntoFile(std::string(FLAGS_config_directory) + "/log_time.txt");
    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}
