/*
 * @Description: LOAM scan registration, implementation
 * @Author: Ge Yao
 * @Date: 2021-05-04 14:53:21
 */

#include <chrono>

#include "glog/logging.h"

#include "lidar_localization/models/loam/floam_factor.hpp"

#include "lidar_localization/models/loam/floam_registration.hpp"

namespace lidar_localization {

CeresFLOAMRegistration::CeresFLOAMRegistration(
    const CeresFLOAMRegistration::Config& config, 
    const Eigen::Quaterniond &dq, const Eigen::Vector3d &dt
) {   
    //
    // config optimizer:
    // 
    // 1. parameterization:
    config_.q_parameterization_ptr = new PoseSE3Parameterization();
    // 2. loss function:
    // TODO: move param to config
    config_.loss_function_ptr = new ceres::HuberLoss(0.10);

    // 3. solver:
    config_.options.linear_solver_type = ceres::DENSE_QR;
    // config_.options.use_explicit_schur_complement = true;
    // config_.options.trust_region_strategy_type = ceres::DOGLEG;
    // config_.options.use_nonmonotonic_steps = true;
    config_.options.num_threads = config.num_threads;
    config_.options.max_num_iterations = config.max_num_iterations;
    config_.options.minimizer_progress_to_stdout = false;
    config_.options.max_solver_time_in_seconds = config.max_solver_time_in_seconds;

    //
    // config target variables:
    //
    param_.q[0] = dq.x(); param_.q[1] = dq.y(); param_.q[2] = dq.z(); param_.q[3] = dq.w();
    param_.t[0] = dt.x(); param_.t[1] = dt.y(); param_.t[2] = dt.z();
    param_.pose[0] = dq.x(); param_.pose[1] = dq.y(); param_.pose[2] = dq.z(); param_.pose[3] = dq.w();
    param_.pose[4] = dt.x(); param_.pose[5] = dt.y(); param_.pose[6] = dt.z();
    problem_.AddParameterBlock(param_.pose, 7, config_.q_parameterization_ptr);
}

CeresFLOAMRegistration::~CeresFLOAMRegistration() {
}

/**
  * @brief  add residual block for edge constraint from lidar frontend
  * @param  source, source point  
  * @param  target_x, target point x
  * @param  target_y, target point y
  * @param  ratio, interpolation ratio 
  * @return void
  */
bool CeresFLOAMRegistration::AddEdgeFactor(
    const Eigen::Vector3d &source,
    const Eigen::Vector3d &target_x, const Eigen::Vector3d &target_y,
    const double &ratio
) {
    // ceres::CostFunction *factor_edge = LidarEdgeFactor::Create(
    //     source, 
    //     target_x, target_y, 
    //     ratio
    // );
    
    // problem_.AddResidualBlock(
    //     factor_edge, 
    //     config_.loss_function_ptr, 
    //     param_.q, param_.t
    // );
    ceres::CostFunction* factor_edge = new LidarEdgeFactor
    (
        source, 
        target_x, target_y, 
        ratio
    );

    problem_.AddResidualBlock(
        factor_edge, 
        config_.loss_function_ptr, 
        param_.pose
    );

    return true;
}

/**
  * @brief  add residual block for plane constraint from lidar frontend
  * @param  source, source point
  * @param  target_x, target point x
  * @param  target_y, target point y
  * @param  target_z, target point z
  * @param  ratio, interpolation ratio
  * @return void
  */
bool CeresFLOAMRegistration::AddPlaneFactor(
    const Eigen::Vector3d &source,
    const Eigen::Vector3d &target_x, const Eigen::Vector3d &target_y, const Eigen::Vector3d &target_z,
    const double &ratio
) {
    // ceres::CostFunction *factor_plane = LidarPlaneFactor::Create(
    //     source, 
    //     target_x, target_y, target_z, 
    //     ratio
    // );

    // problem_.AddResidualBlock(
    //     factor_plane,
    //     config_.loss_function_ptr, 
    //     param_.q, param_.t
    // );

    ceres::CostFunction* factor_plane = new LidarPlaneFactor
    (
        source, 
        target_x, target_y, target_z, 
        ratio
    );

    problem_.AddResidualBlock(
        factor_plane, 
        config_.loss_function_ptr, 
        param_.pose
    );

    return true;
}

/**
  * @brief  add residual block for plane constraint from lidar frontend
  * @param  source, source point
  * @param  norm, normal direction of target plane
  * @param  negative_oa_dot_norm
  * @return void
  */
bool CeresFLOAMRegistration::AddPlaneNormFactor(
    const Eigen::Vector3d &source,
    const Eigen::Vector3d &norm, const double &negative_oa_dot_norm
) {
    ceres::CostFunction *factor_plane = LidarPlaneNormFactor::Create(
        source, 
        norm, negative_oa_dot_norm
    );

    problem_.AddResidualBlock(
        factor_plane,
        config_.loss_function_ptr, 
        param_.q, param_.t
    );

    return true;
}

bool CeresFLOAMRegistration::Optimize() {
    // solve:
    ceres::Solver::Summary summary;

    // time it:
    auto start = std::chrono::steady_clock::now();

    ceres::Solve(config_.options, &problem_, &summary);
    
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = end - start;
    
    return true;
}

/**
  * @brief  get optimized relative pose
  * @return true if success false otherwise
  */
bool CeresFLOAMRegistration::GetOptimizedRelativePose(Eigen::Quaterniond &dq, Eigen::Vector3d &dt) {
    dq.x() = param_.pose[0];
    dq.y() = param_.pose[1];
    dq.z() = param_.pose[2];
    dq.w() = param_.pose[3];
    dq.normalize();

    dt.x() = param_.pose[4];
    dt.y() = param_.pose[5];
    dt.z() = param_.pose[6];
    
    // dq.x() = param_.q[0];
    // dq.y() = param_.q[1];
    // dq.z() = param_.q[2];
    // dq.w() = param_.q[3];
    // dq.normalize();

    // dq = param.so3_.unit_quaternion();

    // dt.x() = param_.t[0];
    // dt.y() = param_.t[1];
    // dt.z() = param_.t[2];

    return true;
}

} // namespace graph_ptr_optimization