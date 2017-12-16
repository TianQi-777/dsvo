#include "imu.hpp"
#include <iostream>

IMU::IMU(std::vector<double> T_BS,
		 double gyroscope_noise_density,
		 double gyroscope_random_walk,
		 double accelerometer_noise_density,
		 double accelerometer_random_walk) {
	//set imu model
	model.T_BS = Eigen::Matrix4d::Map(&T_BS[0]);
	// std::cout<<model.T_BS<<std::endl;
	model.M = Eigen::Matrix<double,12,12>::Zero();
	model.M.block<3,3>(0,0) = accelerometer_noise_density*Eigen::Matrix3d::Identity();
	model.M.block<3,3>(3,3) = gyroscope_noise_density*Eigen::Matrix3d::Identity();
	model.M.block<3,3>(6,6) = accelerometer_random_walk*Eigen::Matrix3d::Identity();
	model.M.block<3,3>(9,9) = gyroscope_random_walk*Eigen::Matrix3d::Identity();

	t_recent = -1;
}

void IMU::imu_propagate(State& state, const Eigen::Vector3d& rot_vel, const Eigen::Vector3d& lin_acc, double t_cur) {
    std::cout<<"Propagating... "<<std::endl<<std::endl;
	// initialization
	// std::cout<<rot_vel<<std::endl;
	if(t_recent<0) {
		model.g = -lin_acc;
		t_recent = t_cur;
		return;
	}

	// get duration
	double d_t = t_cur - t_recent;
	t_recent = t_cur;

	// overall acceleration
	Eigen::Matrix3d R = state.pose.orientation.toRotationMatrix();
	Eigen::Vector3d acc = model.g + R*(lin_acc-state.imu_bias.acceleration);

	// update covariance
	Eigen::Vector3d c1 = 0.5*(lin_acc-state.imu_bias.acceleration)*d_t*d_t;
	Eigen::Vector3d c2 = (lin_acc-state.imu_bias.acceleration)*d_t;
	Eigen::MatrixXd d_p_q = d_c_R(c1)*d_R_q(state.pose.orientation);
	Eigen::MatrixXd d_v_q = d_c_R(c2)*d_R_q(state.pose.orientation);
	
	Eigen::MatrixXd G = Eigen::Matrix<double, 16, 16>::Identity();
	G.block<3,4>(0,3) = d_p_q;
	G.block<3,3>(0,7) = d_t*Eigen::Matrix3d::Identity();
	G.block<3,3>(0,10) = -0.5*d_t*d_t*R;
	G.block<4,4>(3,3) = R_prod(exp_q((rot_vel-state.imu_bias.rotation)*d_t/2));
	G.block<4,3>(3,13) = -0.5*d_t*L_prod(state.pose.orientation)*d_exp_q(state.imu_bias.rotation);
	G.block<3,4>(7,3) = d_v_q;
	G.block<3,3>(7,10) = -R*d_t;
// std::cout<<G<<std::endl;

	Eigen::MatrixXd V = Eigen::Matrix<double, 16,12>::Zero();
	V.block<3,3>(0,0) = -G.block<3,3>(0,10);
	V.block<3,3>(0,6) = G.block<3,3>(0,10);
	V.block<4,3>(3,3) = -G.block<4,3>(3,13);
	V.block<4,3>(3,9) = G.block<4,3>(3,13);
	V.block<3,3>(7,0) = -G.block<3,3>(7,10);
	V.block<3,3>(7,6) = G.block<3,3>(7,10);
	V.block<3,3>(10,6) = Eigen::Matrix3d::Identity();
	V.block<3,3>(13,9) = Eigen::Matrix3d::Identity();

	Eigen::MatrixXd trunked_cov = state.covariance.block<16,16>(0,0);
	trunked_cov = G*trunked_cov*G.transpose() + V*model.M*V.transpose();
	state.covariance.block<16,16>(0,0) = trunked_cov;


	// propagation 
	// state.pose.position = state.pose.position + state.velocity*d_t + 0.5*acc*d_t*d_t;
	state.pose.orientation = state.pose.orientation*exp_q((rot_vel-state.imu_bias.rotation)*d_t/2);
	state.velocity = state.velocity + acc*d_t;

	// show pose
	state.showPose();
    std::cout<<"Propagating end "<<std::endl<<std::endl;
}
