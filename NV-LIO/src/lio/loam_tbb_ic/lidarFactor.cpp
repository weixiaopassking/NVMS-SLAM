#include "lio/loam_tbb_ic/lidarFactor.h"

namespace zjloc::loam
{
     double LidarPlaneNormFactor::sqrt_info;
     Eigen::Vector3d LidarPlaneNormFactor::t_il;
     Eigen::Quaterniond LidarPlaneNormFactor::q_il;

     LidarPlaneNormFactor::LidarPlaneNormFactor(const Eigen::Vector3d &point_body_, const Eigen::Vector3d &norm_vector_, const double norm_offset_, double weight_)
         : point_body(point_body_), norm_vector(norm_vector_), norm_offset(norm_offset_), weight(weight_)
     {
     }

     bool LidarPlaneNormFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
     {

          Eigen::Vector3d translation(parameters[0][0], parameters[0][1], parameters[0][2]);
          Eigen::Quaterniond rotation(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);

          Eigen::Vector3d point_world = rotation * point_body + translation;
          double distance = norm_vector.dot(point_world) + norm_offset;

          residuals[0] = sqrt_info * weight * distance;

          if (jacobians)
          {
               if (jacobians[0])
               {
                    Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran(jacobians[0]);
                    jacobian_tran.setZero();

                    jacobian_tran.block<1, 3>(0, 0) = sqrt_info * norm_vector.transpose() * weight;
                    // std::cout << "1:" << sqrt_info * norm_vector.transpose() * weight << std::endl;
               }
               if (jacobians[1])
               {
                    Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot(jacobians[1]);
                    jacobian_rot.setZero();

                    jacobian_rot.block<1, 3>(0, 0) = -sqrt_info * norm_vector.transpose() * rotation.toRotationMatrix() * zjloc::loam::numType::skewSymmetric(point_body) * weight;
                    // std::cout << "2:" << -sqrt_info * norm_vector.transpose() * rotation.toRotationMatrix() * zjloc::numType::skewSymmetric(point_body) * weight << std::endl;
               }
          }

          return true;
     }

}