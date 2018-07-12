#ifndef STATE_H
#define STATE_H

#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct Pose
{
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;

	Pose() {
		position = Eigen::Vector3d::Zero();
		orientation = Eigen::Quaterniond::Identity();
	}
};

struct State {
	// mean
	ros::Time time;
	Pose pose;
	Eigen::Vector3d velocity;

	State();
	ros::NodeHandle nh;
	ros::Publisher pose_pub;
	void reset();
	void showPose(bool stereo_match_flag);
};

#endif
