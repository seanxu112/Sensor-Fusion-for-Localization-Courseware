/*
 * @Description: ceres residual block for LIO IMU pre-integration measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace sliding_window {

class FactorPRVAGIMUPreIntegration : public ceres::SizedCostFunction<15, 15, 15> {
public:
	static const int INDEX_P = 0;
	static const int INDEX_R = 3;
	static const int INDEX_V = 6;
	static const int INDEX_A = 9;
	static const int INDEX_G = 12;

  FactorPRVAGIMUPreIntegration(void) {};

	void SetT(const double &T) {
		T_ = T;
	}

	void SetGravitiy(const Eigen::Vector3d &g) {
		g_ = g;
	}

  void SetMeasurement(const Eigen::VectorXd &m) {
		m_ = m;
	}

  void SetInformation(const Eigen::MatrixXd &I) {
    I_ = I;
  }

	void SetJacobian(const Eigen::MatrixXd &J) {
		J_ = J;
	}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    //
    // parse parameters:
    //
    // a. pose i
    Eigen::Map<const Eigen::Vector3d>     pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d                    ori_i = Sophus::SO3d::exp(log_ori_i);
		Eigen::Map<const Eigen::Vector3d>     vel_i(&parameters[0][INDEX_V]);
		Eigen::Map<const Eigen::Vector3d>     b_a_i(&parameters[0][INDEX_A]);
		Eigen::Map<const Eigen::Vector3d>     b_g_i(&parameters[0][INDEX_G]);

    // b. pose j
    Eigen::Map<const Eigen::Vector3d>     pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d                    ori_j = Sophus::SO3d::exp(log_ori_j);
		Eigen::Map<const Eigen::Vector3d>     vel_j(&parameters[1][INDEX_V]);
		Eigen::Map<const Eigen::Vector3d>     b_a_j(&parameters[1][INDEX_A]);
		Eigen::Map<const Eigen::Vector3d>     b_g_j(&parameters[1][INDEX_G]);

    //
    // parse measurement:
    // 
		const Eigen::Vector3d &alpha_ij = m_.block<3, 1>(INDEX_P, 0);
		const Eigen::Vector3d &theta_ij = m_.block<3, 1>(INDEX_R, 0);
		const Eigen::Vector3d  &beta_ij = m_.block<3, 1>(INDEX_V, 0);

    //
    // TODO: get square root of information matrix:
    //
    Eigen::Matrix<double, 6, 6>  sqrt_info =  Eigen::LLT<Eigen::Matrix<double, 6 ,6>>(I_).matrixL().transpose() ;
    //
    // TODO: compute residual:
    //
    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual_vec(residuals);
    residual_vec.block<3, 1>(INDEX_P, 0) = ori_i.matrix() * (pos_j - pos_i - vel_i*T_ + 0.5*g_*T_*T_) - alpha_ij;
    residual_vec.block<3, 1>(INDEX_R, 0) = (Sophus::SO3d::exp(theta_ij).inverse() * ori_i.inverse() * ori_j).log();
    residual_vec.block<3, 1>(INDEX_V, 0) = ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
    residual_vec.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
    residual_vec.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;

    //
    // TODO: compute jacobians:
    //
    if ( jacobians ) {
      // compute shared intermediate results:
      Eigen::MatrixXd R_i(ori_i.matrix());
      Eigen::MatrixXd R_j(ori_i.matrix());
      Eigen::MatrixXd R_i_inv(ori_i.inverse().matrix());
      Eigen::MatrixXd R_j_inv(ori_j.inverse().matrix());
      Eigen::MatrixXd J_pa = J_.block<3, 3>(INDEX_P, INDEX_A);
      Eigen::MatrixXd J_pg = J_.block<3, 3>(INDEX_P, INDEX_G);
      Eigen::MatrixXd J_qg = J_.block<3, 3>(INDEX_R, INDEX_G);
      Eigen::MatrixXd J_va = J_.block<3, 3>(INDEX_V, INDEX_A);
      Eigen::MatrixXd J_vg = J_.block<3, 3>(INDEX_V, INDEX_G);
      Eigen::MatrixXd J_r_inv = JacobianRInv(residual_vec.block<3, 1>(INDEX_R, 0));



      if ( jacobians[0] ) {
        Eigen::Map<Eigen::Matrix<double, 6, 15>> jacobi_mat(jacobians[0]);
        jacobi_mat.setZero();
        // a. residual, position:
        jacobi_mat.block<3,3>(INDEX_P, INDEX_P) = -R_i_inv;
        jacobi_mat.block<3,3>(INDEX_P, INDEX_R) = Sophus::SO3d::hat(R_i_inv*(pos_j - pos_i - vel_i*T_ + 0.5*g_*T_*T_));
        jacobi_mat.block<3,3>(INDEX_P, INDEX_V) = -R_i_inv*T_;
        jacobi_mat.block<3,3>(INDEX_P, INDEX_A) = -J_pa;
        jacobi_mat.block<3,3>(INDEX_P, INDEX_G) = -J_pg;

        // b. residual, orientation:
        // jacobi_mat.block<3,3>(INDEX_R, INDEX_P) = 
        jacobi_mat.block<3,3>(INDEX_R, INDEX_R) = -J_r_inv * R_j_inv * R_i;
        // jacobi_mat.block<3,3>(INDEX_R, INDEX_V) = 
        // jacobi_mat.block<3,3>(INDEX_R, INDEX_A) = 
        jacobi_mat.block<3,3>(INDEX_R, INDEX_G) = -J_r_inv * (Sophus::SO3d::exp(residual_vec.block<3, 1>(INDEX_R, 0))).matrix().inverse() * J_qg;

        // c. residual, velocity:
        // jacobi_mat.block<3,3>(INDEX_V, INDEX_P) = 
        jacobi_mat.block<3,3>(INDEX_V, INDEX_R) = Sophus::SO3d::hat(R_i_inv * (vel_j - vel_i + g_*T_));
        jacobi_mat.block<3,3>(INDEX_V, INDEX_V) = -R_i_inv;
        jacobi_mat.block<3,3>(INDEX_V, INDEX_A) = -J_va;
        jacobi_mat.block<3,3>(INDEX_V, INDEX_G) = -J_vg;
        // d. residual, bias accel:
        // jacobi_mat.block<3,3>(INDEX_A, INDEX_P) = 
        // jacobi_mat.block<3,3>(INDEX_A, INDEX_R) = 
        // jacobi_mat.block<3,3>(INDEX_A, INDEX_V) = 
        jacobi_mat.block<3,3>(INDEX_A, INDEX_A) = - Eigen::Matrix3d::Identity();
        // jacobi_mat.block<3,3>(INDEX_A, INDEX_G) = 

        // d. residual, bias accel:
        // jacobi_mat.block<3,3>(INDEX_G, INDEX_P) = 
        // jacobi_mat.block<3,3>(INDEX_G, INDEX_R) = 
        // jacobi_mat.block<3,3>(INDEX_G, INDEX_V) = 
        // jacobi_mat.block<3,3>(INDEX_G, INDEX_A) = 
        jacobi_mat.block<3,3>(INDEX_G, INDEX_G) = - Eigen::Matrix3d::Identity();

        jacobi_mat = sqrt_info * jacobi_mat;
      }

      if ( jacobians[1] ) {
        Eigen::Map<Eigen::Matrix<double, 6, 15>> jacobi_mat(jacobians[1]);
        jacobi_mat.setZero();
        // a. residual, position:
        jacobi_mat.block<3,3>(INDEX_P, INDEX_P) = R_i_inv;
        // jacobi_mat.block<3,3>(INDEX_P, INDEX_R) = 
        // jacobi_mat.block<3,3>(INDEX_P, INDEX_V) = 
        // jacobi_mat.block<3,3>(INDEX_P, INDEX_A) = 
        // jacobi_mat.block<3,3>(INDEX_P, INDEX_G) = 

        // b. residual, orientation:
        // jacobi_mat.block<3,3>(INDEX_R, INDEX_P) = 
        jacobi_mat.block<3,3>(INDEX_R, INDEX_R) = J_r_inv;
        // jacobi_mat.block<3,3>(INDEX_R, INDEX_V) = 
        // jacobi_mat.block<3,3>(INDEX_R, INDEX_A) = 
        // jacobi_mat.block<3,3>(INDEX_R, INDEX_G) = 

        // c. residual, velocity:
        // jacobi_mat.block<3,3>(INDEX_V, INDEX_P) = 
        // jacobi_mat.block<3,3>(INDEX_V, INDEX_R) =
        jacobi_mat.block<3,3>(INDEX_V, INDEX_V) = R_i_inv;
        // jacobi_mat.block<3,3>(INDEX_V, INDEX_A) =
        // jacobi_mat.block<3,3>(INDEX_V, INDEX_G) =
        // d. residual, bias accel:
        // jacobi_mat.block<3,3>(INDEX_A, INDEX_P) = 
        // jacobi_mat.block<3,3>(INDEX_A, INDEX_R) = 
        // jacobi_mat.block<3,3>(INDEX_A, INDEX_V) = 
        jacobi_mat.block<3,3>(INDEX_A, INDEX_A) = Eigen::Matrix3d::Identity();
        // jacobi_mat.block<3,3>(INDEX_A, INDEX_G) = 

        // d. residual, bias accel:
        // jacobi_mat.block<3,3>(INDEX_G, INDEX_P) = 
        // jacobi_mat.block<3,3>(INDEX_G, INDEX_R) = 
        // jacobi_mat.block<3,3>(INDEX_G, INDEX_V) = 
        // jacobi_mat.block<3,3>(INDEX_G, INDEX_A) = 
        jacobi_mat.block<3,3>(INDEX_G, INDEX_G) = Eigen::Matrix3d::Identity();

        jacobi_mat = sqrt_info * jacobi_mat;
      }
    }

    //
    // TODO: correct residual by square root of information matrix:
    //
    residual_vec = sqrt_info * residual_vec;
		
    return true;
  }

private:
  static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
      Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

      double theta = w.norm();

      if ( theta > 1e-5 ) {
          Eigen::Vector3d k = w.normalized();
          Eigen::Matrix3d K = Sophus::SO3d::hat(k);
          
          J_r_inv = J_r_inv 
                    + 0.5 * K
                    + (1.0 - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K;
      }

      return J_r_inv;
  }
  
	double T_ = 0.0;

	Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;

	Eigen::MatrixXd J_;
};

} // namespace sliding_window

#endif // LIDAR_LOCALIZATION_MODELS_SLIDING_WINDOW_FACTOR_PRVAG_IMU_PRE_INTEGRATION_HPP_
