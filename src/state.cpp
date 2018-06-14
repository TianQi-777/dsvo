#include "state.hpp"
#include <iostream>
#include <stdlib.h>

State::State() {
	reset();
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 1000);
}

void State::reset() {
	pose.position = Eigen::Vector3d::Zero();
	pose.orientation = Eigen::Quaterniond::Identity();
}

void State::showPose() {
	// publish pose
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "/map";
	pose.header.stamp = time;

	pose.pose.position.x = this->pose.position[0];
	pose.pose.position.y = this->pose.position[1];
	pose.pose.position.z = this->pose.position[2];

	pose.pose.orientation.w = this->pose.orientation.w();
	pose.pose.orientation.x = this->pose.orientation.x();
	pose.pose.orientation.y = this->pose.orientation.y();
	pose.pose.orientation.z = this->pose.orientation.z();

	pose_pub.publish(pose);
}
