#pragma once
// c++
#include <iostream>

// eigen
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "lio/loam_tbb_ic/lio_utils.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// GEOMETRIC COST FUNCTORS
/// FIXME: ct-icp 原始的cost_function
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace zjloc::loam
{
     struct FunctorPointToPlane
     {
          static constexpr int NumResiduals() { return 1; }

          FunctorPointToPlane(const Eigen::Vector3d &reference,
                              const Eigen::Vector3d &target,
                              const Eigen::Vector3d normal,
                              double weight = 1.0) : world_reference_(reference),
                                                     raw_point_(target),
                                                     reference_normal_(normal),
                                                     weight_(weight) {}

          template <typename T>
          bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const
          {
               Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
               Eigen::Matrix<T, 3, 1> transformed = quat.normalized() * raw_point_.template cast<T>();
               transformed(0, 0) += trans_params[0];
               transformed(1, 0) += trans_params[1];
               transformed(2, 0) += trans_params[2];

               T product = (world_reference_.template cast<T>() - transformed).transpose() *
                           reference_normal_.template cast<T>();
               residual[0] = T(weight_) * product;
               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Vector3d &point_world_,
                                             const Eigen::Vector3d &point_body_,
                                             const Eigen::Vector3d &norm_vector_,
                                             double weight_ = 1.0)
          {
               return (new ceres::AutoDiffCostFunction<FunctorPointToPlane, 1, 4, 3>(
                   new FunctorPointToPlane(point_world_, point_body_, norm_vector_, weight_)));
          }

          Eigen::Vector3d world_reference_;
          Eigen::Vector3d raw_point_;
          Eigen::Vector3d reference_normal_;
          double weight_ = 1.0;
          Eigen::Matrix<double, 1, 1> sqrt_info;

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     };

     struct FunctorPointToPoint
     {

          static constexpr int NumResiduals() { return 3; }

          // typedef ceres::AutoDiffCostFunction<FunctorPointToPoint, 3, 4, 3> cost_function_t;

          FunctorPointToPoint(const Eigen::Vector3d &reference,
                              const Eigen::Vector3d &target,
                              const Eigen::Vector3d normal,
                              double weight = 1.0) : world_reference_(reference),
                                                     raw_point_(target),
                                                     weight_(weight) {}

          template <typename T>
          bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const
          {
               Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
               Eigen::Matrix<T, 3, 1> transformed = quat * raw_point_.template cast<T>();
               transformed(0, 0) += trans_params[0];
               transformed(1, 0) += trans_params[1];
               transformed(2, 0) += trans_params[2];

               T t_weight = T(weight_);
               residual[0] = t_weight * (transformed(0) - T(world_reference_(0)));
               residual[1] = t_weight * (transformed(1) - T(world_reference_(1)));
               residual[2] = t_weight * (transformed(2) - T(world_reference_(2)));
               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Vector3d &point_world_,
                                             const Eigen::Vector3d &point_body_,
                                             const Eigen::Vector3d &norm_vector_,
                                             double weight_ = 1.0)
          {
               return (new ceres::AutoDiffCostFunction<FunctorPointToPoint, 3, 4, 3>(
                   new FunctorPointToPoint(point_world_, point_body_, norm_vector_, weight_)));
          }

          Eigen::Vector3d world_reference_;
          Eigen::Vector3d raw_point_;
          double weight_ = 1.0;

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     };

     struct FunctorPointToLine
     {

          static constexpr int NumResiduals() { return 1; }

          // typedef ceres::AutoDiffCostFunction<FunctorPointToLine, 1, 4, 3> cost_function_t;

          FunctorPointToLine(const Eigen::Vector3d &reference,
                             const Eigen::Vector3d &target,
                             const Eigen::Vector3d line,
                             double weight = 1.0) : world_reference_(reference),
                                                    raw_point_(target),
                                                    direction_(line), weight_(weight) {}

          template <typename T>
          bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const
          {
               Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
               Eigen::Matrix<T, 3, 1> transformed = quat * raw_point_.template cast<T>();
               transformed(0, 0) += trans_params[0];
               transformed(1, 0) += trans_params[1];
               transformed(2, 0) += trans_params[2];

               Eigen::Matrix<T, 3, 1> cross = direction_.template cast<T>();
               residual[0] = T(weight_) * cross.normalized().template cross((transformed -
                                                                             world_reference_.template cast<T>()))
                                              .norm();
               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Vector3d &point_world_,
                                             const Eigen::Vector3d &point_body_,
                                             const Eigen::Vector3d &norm_vector_,
                                             double weight_ = 1.0)
          {
               return (new ceres::AutoDiffCostFunction<FunctorPointToLine, 1, 4, 3>(
                   new FunctorPointToLine(point_world_, point_body_, norm_vector_, weight_)));
          }

          Eigen::Vector3d world_reference_;
          Eigen::Vector3d raw_point_;
          Eigen::Vector3d direction_;
          double weight_ = 1.0;

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     };

     struct FunctorPointToDistribution
     {

          static constexpr int NumResiduals() { return 1; }

          // typedef ceres::AutoDiffCostFunction<FunctorPointToDistribution, 1, 4, 3> cost_function_t;

          FunctorPointToDistribution(const Eigen::Vector3d &reference,
                                     const Eigen::Vector3d &target,
                                     const Eigen::Matrix3d &covariance,
                                     double weight = 1.0) : world_reference_(reference),
                                                            raw_point_(target),
                                                            weight_(weight)
          {
               Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance);
               //             Rescale the neighborhood covariance by the largest singular value
               //            neighborhood_information_ = (neighborhood.covariance / std::abs(svd.singularValues()[0]) +
               //                                         Eigen::Matrix3d::Identity() * epsilon).inverse();

               neighborhood_information_ = (covariance +
                                            Eigen::Matrix3d::Identity() * epsilon)
                                               .inverse();
          }

          template <typename T>
          bool operator()(const T *const rot_params, const T *const trans_params, T *residual) const
          {
               Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T *>(rot_params));
               Eigen::Matrix<T, 3, 1> transformed = quat.normalized() * raw_point_.template cast<T>();
               transformed(0, 0) += trans_params[0];
               transformed(1, 0) += trans_params[1];
               transformed(2, 0) += trans_params[2];

               Eigen::Matrix<T, 3, 1> diff = transformed - world_reference_.template cast<T>();

               residual[0] = T(weight_) * (diff.transpose() * neighborhood_information_ * diff)(0, 0);
               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Vector3d &point_world_,
                                             const Eigen::Vector3d &point_body_,
                                             const Eigen::Matrix3d &covariance_,
                                             double weight_ = 1.0)
          {
               return (new ceres::AutoDiffCostFunction<FunctorPointToDistribution, 1, 4, 3>(
                   new FunctorPointToDistribution(point_world_, point_body_, covariance_, weight_)));
          }

          Eigen::Vector3d world_reference_;
          Eigen::Vector3d raw_point_;
          Eigen::Matrix3d neighborhood_information_;
          double weight_ = 1.0;
          double epsilon = 0.05;

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     };

     template <typename FunctorT>
     struct CTFunctor
     {

          static constexpr int NumResiduals() { return FunctorT::NumResiduals(); }

          typedef ceres::AutoDiffCostFunction<CTFunctor<FunctorT>, FunctorT::NumResiduals(), 4, 3, 4, 3> cost_function_t;

          CTFunctor(double timestamp,
                    const Eigen::Vector3d &reference,
                    const Eigen::Vector3d &raw_point,
                    const Eigen::Vector3d &desc,
                    double weight = 1.0)
              : functor(reference, raw_point, desc, weight), alpha_timestamp_(timestamp) {}

          template <typename T>
          inline bool operator()(const T *const begin_rot_params, const T *begin_trans_params,
                                 const T *const end_rot_params, const T *end_trans_params, T *residual) const
          {
               T alpha_m = T(1.0 - alpha_timestamp_);
               T alpha = T(alpha_timestamp_);

               Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot_params));
               Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot_params));
               Eigen::Quaternion<T> quat_inter = quat_begin.normalized().slerp(T(alpha),
                                                                               quat_end.normalized());
               quat_inter.normalize();

               Eigen::Matrix<T, 3, 1> tr;
               tr(0, 0) = alpha_m * begin_trans_params[0] + alpha * end_trans_params[0];
               tr(1, 0) = alpha_m * begin_trans_params[1] + alpha * end_trans_params[1];
               tr(2, 0) = alpha_m * begin_trans_params[2] + alpha * end_trans_params[2];

               return functor(quat_inter.coeffs().data(), tr.data(), residual);
          }

          FunctorT functor;
          double alpha_timestamp_ = 1.0;
     };

     ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     /// GEOMETRIC COST FUNCTORS
     /// FIXME:   ct-icp 解析求导
     ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

     class LidarPlaneNormFactor : public ceres::SizedCostFunction<1, 3, 4>
     {
     public:
          LidarPlaneNormFactor(const Eigen::Vector3d &point_body_, const Eigen::Vector3d &norm_vector_, const double norm_offset_, double weight_ = 1.0);

          virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

          void check(double **parameters);

          Eigen::Vector3d point_body;
          Eigen::Vector3d norm_vector;

          double norm_offset;
          double weight;

          static Eigen::Vector3d t_il;
          static Eigen::Quaterniond q_il;
          static double sqrt_info;
     };

     // -------------------------------------------------------------------------------------------------------------------------------------------------------------
     //  TODO:   ct-icp 自动求导

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

}