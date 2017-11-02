#include "state.hpp"
#include <iostream>

IMU::IMU(std::vector<double> T_BS,
		 double gyroscope_noise_density,
		 double gyroscope_random_walk,
		 double accelerometer_noise_density,
		 double accelerometer_random_walk) {
	//set imu model
	model.T_BS = Eigen::Matrix4d::Map(&T_BS[0]);
	// std::cout<<model.T_BS<<std::endl;
	model.gyroscope_noise_density = gyroscope_noise_density;
	model.gyroscope_random_walk = gyroscope_random_walk;
	model.accelerometer_noise_density = accelerometer_noise_density;
	model.accelerometer_random_walk = accelerometer_random_walk;

	t_recent = -1;
}

void IMU::imu_propagate(State& state, const Eigen::Vector3d& rot_vel, const Eigen::Vector3d& lin_acc, double t_cur) {
	// initialization
	// std::cout<<rot_vel<<std::endl;
	if(t_recent<0) {
		g = -lin_acc;
		t_recent = t_cur;
		return;
	}

	// get duration
	double d_t = t_cur - t_recent;
	t_recent = t_cur;

	// overall acceleration
	Eigen::Matrix3d R = state.pose.orientation.toRotationMatrix();
	Eigen::Vector3d acc = g + R*(lin_acc-state.imu_bias.acceleration);

	// update covariance
	Eigen::Vector3d c1 = 0.5*(lin_acc-state.imu_bias.acceleration)*d_t*d_t;
	Eigen::Vector3d c2 = (lin_acc-state.imu_bias.acceleration)*d_t;
	Eigen::MatrixXd d_p_q = d_c_R(c1)*d_R_q(state.pose.orientation);
	Eigen::MatrixXd d_v_q = d_c_R(c2)*d_R_q(state.pose.orientation);
	Eigen::Matrix<double, 16, 16> G = Eigen::Matrix<double, 16, 16>::Identity();
	G.block<3,4>(0,3) = d_p_q;
	G.block<3,3>(0,7) = d_t*Eigen::Matrix3d::Identity();
	G.block<3,3>(0,10) = -0.5*d_t*d_t*R;
	G.block<4,4>(3,3) = R_prod(exp_q((rot_vel-state.imu_bias.rotation)*d_t/2));
	G.block<4,3>(3,13) = -0.5*d_t*L_prod(state.pose.orientation)*d_exp_q(state.imu_bias.rotation);

	// propagation 
	state.pose.position = state.pose.position + state.velocity*d_t + 0.5*acc*d_t*d_t;
	state.pose.orientation = state.pose.orientation*exp_q((rot_vel-state.imu_bias.rotation)*d_t/2);
	state.velocity = state.velocity + acc*d_t;
	
	// state.variance = 

	// show pose
	state.showPose();
}
