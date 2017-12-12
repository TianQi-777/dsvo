#ifndef IMU_MATH_HPP
#define IMU_MATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Quaterniond exp_q(const Eigen::Vector3d& v);
Eigen::Matrix4d L_prod(const Eigen::Quaterniond& q);
Eigen::Matrix4d R_prod(const Eigen::Quaterniond& q);
Eigen::MatrixXd d_c_R(const Eigen::Vector3d& c);
Eigen::MatrixXd d_R_q(const Eigen::Quaterniond& q);
Eigen::MatrixXd d_exp_q(const Eigen::Vector3d& x);

#endif