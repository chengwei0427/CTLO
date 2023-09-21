#pragma once
// c++
#include <iostream>

// eigen
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "lio_utils.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// GEOMETRIC COST FUNCTORS
/// FIXME: ct-icp 原始的cost_function
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace CT_ICP
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

     class CTLidarPlaneNormFactor : public ceres::SizedCostFunction<1, 3, 4, 3, 4>
     {
     public:
          CTLidarPlaneNormFactor(const Eigen::Vector3d &raw_keypoint_, const Eigen::Vector3d &norm_vector_, const double norm_offset_, double alpha_time_, double weight_ = 1.0);

          virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

          void check(double **parameters);

          Eigen::Vector3d raw_keypoint;
          Eigen::Vector3d norm_vector;

          double norm_offset;
          double alpha_time;
          double weight;

          static Eigen::Vector3d t_il;
          static Eigen::Quaterniond q_il;
          static double sqrt_info;
     };

     class LocationConsistencyFactor : public ceres::SizedCostFunction<3, 3>
     {

     public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          LocationConsistencyFactor(const Eigen::Vector3d &previous_location_, double beta_);

          virtual ~LocationConsistencyFactor() {}

          virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

          Eigen::Vector3d previous_location;
          double beta = 1.0;
     };

     class RotationConsistencyFactor : public ceres::SizedCostFunction<3, 4>
     {

     public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          RotationConsistencyFactor(const Eigen::Quaterniond &previous_rotation_, double beta_);

          virtual ~RotationConsistencyFactor() {}

          virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

          Eigen::Quaterniond previous_rotation;
          double beta = 1.0;
     };

     class SmallVelocityFactor : public ceres::SizedCostFunction<3, 3, 3>
     {

     public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          SmallVelocityFactor(double beta_);

          virtual ~SmallVelocityFactor() {}

          virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

          double beta;
     };

     class VelocityConsistencyFactor : public ceres::SizedCostFunction<3, 3, 3>
     {

     public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          VelocityConsistencyFactor(const Eigen::Vector3d &previous_vel_, double beta_);

          virtual ~VelocityConsistencyFactor() {}

          virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

          Eigen::Vector3d previous_velocity;

          double beta = 1.0;
     };

     class VelocityConsistencyFactor2 : public ceres::SizedCostFunction<9, 9>
     {

     public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          VelocityConsistencyFactor2(zjloc::state *previous_state_, double beta_);

          virtual ~VelocityConsistencyFactor2() {}

          virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

          Eigen::Vector3d previous_velocity;
          Eigen::Vector3d previous_ba;
          Eigen::Vector3d previous_bg;
          double beta = 1.0;
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

     struct CTPointToPlaneFunctor
     {

          static constexpr int NumResiduals() { return 1; }

          CTPointToPlaneFunctor(const Eigen::Vector3d &reference_point, const Eigen::Vector3d &raw_target,
                                const Eigen::Vector3d &reference_normal, double alpha_timestamp, double weight = 1.0)
              : raw_keypoint_(raw_target),
                reference_point_(reference_point),
                reference_normal_(reference_normal),
                alpha_timestamps_(alpha_timestamp),
                weight_(weight) {}

          template <typename T>
          bool operator()(const T *begin_trans_params, const T *const begin_rot_params,
                          const T *end_trans_params, const T *const end_rot_params, T *residual) const
          {
               Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T *>(begin_rot_params));
               Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T *>(end_rot_params));
               Eigen::Quaternion<T> quat_inter = quat_begin.slerp(T(alpha_timestamps_), quat_end);
               quat_inter.normalize();

               Eigen::Matrix<T, 3, 1> raw_keypoint_temp(T(raw_keypoint_(0, 0)), T(raw_keypoint_(1, 0)), T(raw_keypoint_(2, 0)));

               Eigen::Matrix<T, 3, 1> transformed = quat_inter * raw_keypoint_temp;

               T alpha_m = T(1.0 - alpha_timestamps_);
               transformed(0, 0) += alpha_m * begin_trans_params[0] + alpha_timestamps_ * end_trans_params[0];
               transformed(1, 0) += alpha_m * begin_trans_params[1] + alpha_timestamps_ * end_trans_params[1];
               transformed(2, 0) += alpha_m * begin_trans_params[2] + alpha_timestamps_ * end_trans_params[2];

               Eigen::Matrix<T, 3, 1> reference_point_temp(T(reference_point_(0, 0)), T(reference_point_(1, 0)), T(reference_point_(2, 0)));

               Eigen::Matrix<T, 3, 1> reference_normal_temp(T(reference_normal_(0, 0)), T(reference_normal_(1, 0)), T(reference_normal_(2, 0)));

               residual[0] = T(weight_) * (reference_point_temp - transformed).transpose() * reference_normal_temp;

               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Vector3d &point_world_,
                                             const Eigen::Vector3d &point_body_,
                                             const Eigen::Vector3d &norm_vector_,
                                             const double &alpha_time_,
                                             double weight_ = 1.0)
          {
               return (new ceres::AutoDiffCostFunction<CTPointToPlaneFunctor, 1, 3, 4, 3, 4>(
                   new CTPointToPlaneFunctor(point_world_, point_body_, norm_vector_, alpha_time_, weight_)));
          }

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

          Eigen::Vector3d raw_keypoint_;
          Eigen::Vector3d reference_point_;
          Eigen::Vector3d reference_normal_;
          double alpha_timestamps_;
          double weight_ = 1.0;
     };

     struct LocationConsistencyFunctor
     {

          static constexpr int NumResiduals() { return 3; }

          LocationConsistencyFunctor(const Eigen::Vector3d &previous_location,
                                     double beta) : beta_(beta), previous_location_(previous_location) {}

          template <typename T>
          bool operator()(const T *const location_params, T *residual) const
          {

               Eigen::Matrix<T, 3, 1> previous_location_temp(T(previous_location_(0, 0)), T(previous_location_(1, 0)), T(previous_location_(2, 0)));

               residual[0] = beta_ * (location_params[0] - previous_location_temp(0, 0));
               residual[1] = beta_ * (location_params[1] - previous_location_temp(1, 0));
               residual[2] = beta_ * (location_params[2] - previous_location_temp(2, 0));
               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Vector3d &previous_location_,
                                             const double &beta_)
          {
               return (new ceres::AutoDiffCostFunction<LocationConsistencyFunctor, 3, 3>(
                   new LocationConsistencyFunctor(previous_location_, beta_)));
          }

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     private:
          Eigen::Vector3d previous_location_;
          double beta_ = 1.0;
     };

     // A Functor which enforces frame orientation consistency between two poses
     struct OrientationConsistencyFunctor
     {

          static constexpr int NumResiduals() { return 1; }

          OrientationConsistencyFunctor(const Eigen::Quaterniond &previous_orientation, double beta) : beta_(beta), previous_orientation_(previous_orientation) {}

          template <typename T>
          bool operator()(const T *const orientation_params, T *residual) const
          {

               Eigen::Quaternion<T> quat(orientation_params);

               Eigen::Quaternion<T> previous_orientation_temp(T(previous_orientation_.w()), T(previous_orientation_.x()), T(previous_orientation_.y()), T(previous_orientation_.z()));

               T scalar_quat = quat.dot(previous_orientation_temp);

               residual[0] = T(beta_) * (T(1.0) - scalar_quat * scalar_quat);
               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Quaterniond &previous_orientation_,
                                             const double &beta_)
          {
               return (new ceres::AutoDiffCostFunction<OrientationConsistencyFunctor, 1, 4>(
                   new OrientationConsistencyFunctor(previous_orientation_, beta_)));
          }

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     private:
          Eigen::Quaterniond previous_orientation_;
          double beta_;
     };

     // A Const Functor which enforces a Constant Velocity constraint on translation
     struct ConstantVelocityFunctor
     {

          static constexpr int NumResiduals() { return 3; }

          ConstantVelocityFunctor(const Eigen::Vector3d &previous_velocity, double beta) : previous_velocity_(previous_velocity), beta_(beta) {}

          template <typename T>
          bool operator()(const T *const begin_t, const T *const end_t, T *residual) const
          {

               Eigen::Matrix<T, 3, 1> previous_velocity_temp(T(previous_velocity_(0, 0)), T(previous_velocity_(1, 0)), T(previous_velocity_(2, 0)));

               residual[0] = T(beta_) * (end_t[0] - begin_t[0] - previous_velocity_temp(0, 0));
               residual[1] = T(beta_) * (end_t[1] - begin_t[1] - previous_velocity_temp(1, 0));
               residual[2] = T(beta_) * (end_t[2] - begin_t[2] - previous_velocity_temp(2, 0));
               return true;
          }

          static ceres::CostFunction *Create(const Eigen::Vector3d &previous_velocity_,
                                             const double &beta_)
          {
               return (new ceres::AutoDiffCostFunction<ConstantVelocityFunctor, 3, 3, 3>(
                   new ConstantVelocityFunctor(previous_velocity_, beta_)));
          }

          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
     private:
          Eigen::Vector3d previous_velocity_;
          double beta_ = 1.0;
     };

     // A Const Functor which enforces a Small Velocity constraint
     struct SmallVelocityFunctor
     {

          static constexpr int NumResiduals() { return 3; }

          SmallVelocityFunctor(double beta) : beta_(beta){};

          template <typename T>
          bool operator()(const T *const begin_t, const T *const end_t, T *residual) const
          {
               residual[0] = beta_ * (begin_t[0] - end_t[0]);
               residual[1] = beta_ * (begin_t[1] - end_t[1]);
               residual[2] = beta_ * (begin_t[2] - end_t[2]);
               return true;
          }

          static ceres::CostFunction *Create(const double &beta_)
          {
               return (new ceres::AutoDiffCostFunction<SmallVelocityFunctor, 3, 3, 3>(
                   new SmallVelocityFunctor(beta_)));
          }

          double beta_;
     };

     class TruncatedLoss : public ceres::LossFunction
     {
     public:
          explicit TruncatedLoss(double sigma) : sigma2_(sigma * sigma) {}

          void Evaluate(double, double *) const override;

     private:
          const double sigma2_;
     };
}