// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

//
// TODO: implement analytic Jacobians for LOAM residuals in this file
// 

#include <eigen3/Eigen/Dense>

//
// TODO: Sophus is ready to use if you have a good undestanding of Lie algebra.
// 
#include <sophus/so3.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


Eigen::Matrix3d skew(const Eigen::Vector3d vec){                 //  反对称矩阵定义
    Eigen::Matrix3d skew_mat;
    skew_mat.setZero();
    skew_mat <<         0,      -vec(2),       vec(1),
                   vec(2),            0,      -vec(0),
                  -vec(1),       vec(0),            0;
    return skew_mat;
}

void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t){
    Eigen::Vector3d omega(se3.data());
    Eigen::Vector3d upsilon(se3.data()+3);
    Eigen::Matrix3d Omega = skew(omega);

    double theta = omega.norm();
    double half_theta = 0.5*theta;

    double imag_factor;
    double real_factor = cos(half_theta);
    if(theta<1e-10)
    {
        double theta_sq = theta*theta;
        double theta_po4 = theta_sq*theta_sq;
        imag_factor = 0.5-0.0208333*theta_sq+0.000260417*theta_po4;
    }
    else
    {
        double sin_half_theta = sin(half_theta);
        imag_factor = sin_half_theta/theta;
    }

    q = Eigen::Quaterniond(real_factor, imag_factor*omega.x(), imag_factor*omega.y(), imag_factor*omega.z());


    Eigen::Matrix3d J;
    if (theta<1e-10)
    {
        J = q.matrix();
    }
    else
    {
        Eigen::Matrix3d Omega2 = Omega*Omega;
        J = (Eigen::Matrix3d::Identity() + (1-cos(theta))/(theta*theta)*Omega + (theta-sin(theta))/(pow(theta,3))*Omega2);
    }

    t = J*upsilon;
}

class LidarEdgeFactor:  public   ceres::SizedCostFunction<1, 7> // Size 1 residue, Input block 4 (quaternion), and 3 (translation) 
{
	public:
		Eigen::Vector3d cp, lpa, lpb;
		double s;
		LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
						Eigen::Vector3d last_point_b_, double s_)
			: cp(curr_point_), lpa(last_point_a_), lpb(last_point_b_), s(s_) {}

	virtual bool Evaluate(double const* const* parameters,
                          double* residual,
                          double** jacobians) const
	{
		Eigen::Map<const  Eigen::Quaterniond>   q_last_curr(parameters[0]);               //   存放 w  x y z 
        Eigen::Map<const  Eigen::Vector3d>      t_last_curr(parameters[0] + 4);
		
		Eigen::Vector3d lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Vector3d nu = (lp - lpa).cross(lp - lpb);
		Eigen::Vector3d de = lpa - lpb;

		residual[0] = nu.norm() / de.norm();

		if (jacobians != nullptr && jacobians[0] != nullptr) 
		{
			Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            dp_by_se3.block<3,3>(0,0) = -skew_lp;
            (dp_by_se3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            Eigen::Matrix3d skew_de = skew(de);
            J_se3.block<1,6>(0,0) = - nu.transpose() / nu.norm() * skew_de * dp_by_se3/de.norm();
		}
		return true;
	}
};

class LidarPlaneFactor : public   ceres::SizedCostFunction<1, 7> 
{
	public:
		LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
						Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
			: cp(curr_point_), lpj(last_point_j_), lpl(last_point_l_),
			lpm(last_point_m_), s(s_)
		{
			ljm_norm = (lpj - lpl).cross(lpj - lpm);
			ljm_norm.normalize();
		}

	virtual bool Evaluate(double const* const* parameters,
                          double* residual,
                          double** jacobians) const
	{
		Eigen::Map<const  Eigen::Quaterniond>   q_last_curr(parameters[0]);             
		Eigen::Map<const  Eigen::Vector3d>      t_last_curr(parameters[0] + 4);

		Eigen::Vector3d lp;
		lp = q_last_curr * cp + t_last_curr;

		residual[0] = (lp - lpj).dot(ljm_norm);

        if (jacobians != nullptr && jacobians[0] != nullptr) 
		{
			Eigen::Matrix3d skew_lp = skew(lp);
            Eigen::Matrix<double, 3, 6> dp_by_se3;
            dp_by_se3.block<3,3>(0,0) = -skew_lp;
            (dp_by_se3.block<3,3>(0, 3)).setIdentity();
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > J_se3(jacobians[0]);
            J_se3.setZero();
            J_se3.block<1,6>(0,0) = ljm_norm.transpose() * dp_by_se3;
		}
		return true;
	}

	Eigen::Vector3d cp, lpj, lpl, lpm;
	Eigen::Vector3d ljm_norm;
	double s;
};

struct LidarPlaneNormFactor
{

	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T *pose, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{pose[3], pose[0], pose[1], pose[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{pose[4], pose[5], pose[6]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;

		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 7>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d plane_unit_norm;
	double negative_OA_dot_norm;
};


struct LidarDistanceFactor
{

	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_) 
						: curr_point(curr_point_), closed_point(closed_point_){}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;


		residual[0] = point_w.x() - T(closed_point.x());
		residual[1] = point_w.y() - T(closed_point.y());
		residual[2] = point_w.z() - T(closed_point.z());
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarDistanceFactor, 3, 4, 3>(
			new LidarDistanceFactor(curr_point_, closed_point_)));
	}

	Eigen::Vector3d curr_point;
	Eigen::Vector3d closed_point;
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
	// Quaternion and then translation
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const
	{
		Eigen::Map<const Eigen::Vector3d> trans(x + 4);

		Eigen::Quaterniond delta_q;
		Eigen::Vector3d delta_t;
		getTransformFromSe3(Eigen::Map<const Eigen::Matrix<double,6,1>>(delta), delta_q, delta_t);
		Eigen::Map<const Eigen::Quaterniond> quater(x);
		Eigen::Map<Eigen::Quaterniond> quater_plus(x_plus_delta);
		Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

		quater_plus = delta_q * quater;
		trans_plus = delta_q * trans + delta_t;

    	return true;
	}
    virtual bool ComputeJacobian(const double* x, double* jacobian) const
	{
		Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
		(j.topRows(6)).setIdentity();
		(j.bottomRows(1)).setZero();

		return true;
	}
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};