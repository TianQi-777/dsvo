#include <Eigen/Core>
#include <Eigen/Geometry>
#include "imu_math.hpp"
#include "state.hpp"

struct IMUModel {
	Eigen::Matrix4d T_BS;
};

struct State;

class IMU {
private:
	IMUModel model;
	double t_recent;
public:
	IMU(std::vector<double> T_BS);

	void imu_propagate(State& state, const Eigen::Vector3d& rot_vel, const Eigen::Vector3d& lin_acc, double t_cur);
};