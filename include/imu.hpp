#include <Eigen/Core>
#include <Eigen/Geometry>
#include "imu_math.hpp"
#include "state.hpp"

struct IMUModel {
	Eigen::Matrix4d T_BS;
	Eigen::Vector3d g;
	Eigen::Matrix<double,12,12> M;
};

class State;

class IMU {
private:
	IMUModel model;
	double t_recent;
public:
	IMU(std::vector<double> T_BS,
		double gyroscope_noise_density,
		double gyroscope_random_walk,
		double accelerometer_noise_density,
		double accelerometer_random_walk);

	void imu_propagate(State& state, const Eigen::Vector3d& rot_vel, const Eigen::Vector3d& lin_acc, double t_cur);
};