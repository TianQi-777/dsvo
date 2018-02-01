#include "imu.hpp"
#include <iostream>

IMU::IMU(std::vector<double> T_BS) {
	//set imu model
	model.T_BS = Eigen::Matrix4d::Map(&T_BS[0]);

	t_recent = -1;
}

void IMU::imu_propagate(State& state, const Eigen::Vector3d& rot_vel, const Eigen::Vector3d& lin_acc, double t_cur) {
    // std::cout<<"Propagating... "<<std::endl<<std::endl;

	// initialization
	if(t_recent<0) {
		model.g = -lin_acc;
		state.imu_bias.rotation = -rot_vel;
		t_recent = t_cur;
		return;
	}

	// get duration
	double d_t = t_cur - t_recent;
	t_recent = t_cur;

	// overall acceleration
	Eigen::Matrix3d R = state.pose.orientation.toRotationMatrix();
	Eigen::Vector3d acc = model.g + R*(lin_acc-state.imu_bias.acceleration);

	// propagation 
	state.pose.position = state.pose.position + state.velocity*d_t + 0.5*acc*d_t*d_t;
	state.pose.orientation = state.pose.orientation*exp_q((rot_vel-state.imu_bias.rotation)*d_t/2);
	state.velocity = state.velocity + acc*d_t;

	// show pose
	state.showPose();
    // std::cout<<"Propagating end "<<std::endl<<std::endl;
}
