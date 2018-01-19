#ifndef STATE_H
#define STATE_H

#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

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

struct State {
	// mean
	Pose pose;
	Eigen::Vector3d velocity;
	IMU_bias imu_bias;

	State();
	ros::NodeHandle nh;
	ros::Publisher pose_pub;
	void showPose();
};

#endif 

