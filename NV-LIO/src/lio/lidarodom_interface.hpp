/*
 * @Author: lian-yue0515 506630928@qq.com
 * @Date: 2025-01-08 21:37:32
 * @LastEditors: lian-yue0515 506630928@qq.com
 * @LastEditTime: 2025-01-08 22:48:54
 * @FilePath: /LIO_NVM/src/lio_nvm-main/src/lio/lidarodom_interface.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef LIDAR_ODOM_INTERFACE_HPP__
#define LIDAR_ODOM_INTERFACE_HPP__

#include "tools/imu.h"
#include <pcl/io/pcd_io.h>
#include <std_msgs/Bool.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace zjloc
{

     class lidarodomInterface
     {
     public:
          lidarodomInterface() {}
          virtual ~lidarodomInterface() {}

          lidarodomInterface(const lidarodomInterface &) = delete;
          lidarodomInterface &operator=(const lidarodomInterface &) = delete;

          virtual bool init(const std::string &config_yaml) = 0;

          virtual void pushData(std::vector<point3D>, std::pair<double, double> data) = 0;
          virtual void pushData(IMUPtr imu) = 0;

          virtual void run() = 0;

          virtual int getIndex() = 0;

          virtual void setFunc(std::function<bool(std::string &topic_name, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud, double time)> &fun) = 0;
          virtual void setFunc(std::function<bool(std::string &topic_name, CloudPtr &cloud, double time)> &fun) = 0;
          virtual void setFunc(std::function<bool(std::string &topic_name, SE3 &pose, double time)> &fun) = 0;
          virtual void setFunc(std::function<bool(std::string &topic_name, double time1, double time2)> &fun) = 0;
          virtual void setFunc(std::function<bool(std::string &topic_name, std_msgs::Bool isbegin_seeeion, double time)> &fun) = 0;
     private:
     };
}

#endif