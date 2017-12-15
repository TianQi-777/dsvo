#include "imu_math.hpp"
#include <math.h>
#include <iostream>
Eigen::Quaterniond exp_q(const Eigen::Vector3d& v) {
	double theta = v.norm();
	Eigen::Vector3d u = v / theta;

	Eigen::Quaterniond q;
	q.w() = cos(theta);
	q.vec() = sin(theta)*u;

	return q;
}

Eigen::Matrix4d L_prod(const Eigen::Quaterniond& q) {
	Eigen::Matrix4d M;
	double w = q.w();
	double x = q.x();
	double y = q.y();
	double z = q.z();
	M << w, -x, -y, -z,
	     x,  w, -z,  y,
	     y,  z,  w, -x,
	     z, -y,  x,  w;

	return M;
}

Eigen::Matrix4d R_prod(const Eigen::Quaterniond& q) {
	Eigen::Matrix4d M;
	double w = q.w();
	double x = q.x();
	double y = q.y();
	double z = q.z();
	M << w, -x, -y, -z,
	     x,  w,  z, -y,
	     y, -z,  w,  x,
	     z,  y, -x,  w;

	return M;
}

Eigen::MatrixXd d_c_R(const Eigen::Vector3d& c) {
	Eigen::Matrix<double,3,9> M;
	double c1 = c[0];
	double c2 = c[1];
	double c3 = c[2];
	M << c1, c2, c3,  0,  0,  0,  0,  0,  0,
	      0,  0,  0, c1, c2, c3,  0,  0,  0,
	      0,  0,  0,  0,  0,  0, c1, c2, c3;

	return M;
}

Eigen::MatrixXd d_R_q(const Eigen::Quaterniond& q) {
	Eigen::Matrix<double,9,4> M;
	double w = q.w();
	double x = q.x();
	double y = q.y();
	double z = q.z();
	M <<    0,    0, -4*y, -4*z,
		 -2*z,  2*y,  2*x, -2*w,
		  2*y,  2*z,  2*w,  2*x,
		  2*z,  2*y,  2*x,  2*w,
		    0, -4*x,    0, -4*z,
		 -2*x, -2*w,  2*z,  2*y,
		 -2*y,  2*z, -2*w,  2*x,
		  2*x,  2*w,  2*z,  2*y,
		 	0, -4*x, -4*y,	  0; 
	// M <<    0,    0, -4*y, -4*z,
	// 	  2*z,  2*y,  2*x,  2*w,
	// 	 -2*y,  2*z, -2*w,  2*x,
	// 	 -2*z,  2*y,  2*x, -2*w,
	// 	    0, -4*x,    0, -4*z,
	// 	  2*x,  2*w,  2*z,  2*y,
	// 	  2*y,  2*z,  2*w,  2*x,
	// 	 -2*x, -2*w,  2*z,  2*y,
	// 	 	0, -4*x, -4*y,	  0; 

	return M;
}

Eigen::MatrixXd d_exp_q(const Eigen::Vector3d& x) {
	// angle is too small
	Eigen::Matrix<double, 4, 3> M = Eigen::Matrix<double, 4, 3>::Zero();
	M(1,0) = 1.0;
	M(2,1) = 1.0;
	M(3,2) = 1.0;
	return M;
}