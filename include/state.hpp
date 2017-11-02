#include "helper.hpp"
#include "imu.hpp"
#include "stereo_camera.hpp"
#include <geometry_msgs/PoseStamped.h>

struct Pose
{
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
};

struct IMU_bias
{
	Eigen::Vector3d acceleration;
	Eigen::Vector3d rotation;
};

class State {
private:
	// mean
	Pose pose;
	Eigen::Vector3d velocity;
	IMU_bias imu_bias;

	// variance
	Eigen::Matrix<double, 15, 15> variance;

public:
	State();
	friend void IMU::imu_propagate(State& state, const Eigen::Vector3d& rot_vel, const Eigen::Vector3d& lin_acc, double d_t);

	ros::NodeHandle nh;
	ros::Publisher pose_pub;
	void showPose();
};