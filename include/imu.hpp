struct IMUModel {
	Eigen::Matrix4d T_BS;
	double gyroscope_noise_density;
	double gyroscope_random_walk;
	double accelerometer_noise_density;
	double accelerometer_random_walk;
};

class State;

class IMU {
private:
	IMUModel model;
	Eigen::Vector3d g;
	double t_recent;
public:
	IMU(std::vector<double> T_BS,
		double gyroscope_noise_density,
		double gyroscope_random_walk,
		double accelerometer_noise_density,
		double accelerometer_random_walk);

	void imu_propagate(State& state, const Eigen::Vector3d& rot_vel, const Eigen::Vector3d& lin_acc, double t_cur);
};