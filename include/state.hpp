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

struct State {
	// mean
	ros::Time time;
	Pose pose;
	Eigen::Vector3d velocity;

	State();
	ros::NodeHandle nh;
	ros::Publisher pose_pub;
	void reset();
	void showPose();
};

#endif 

