#ifndef SPTAM_RECORDER_H
#define SPTAM_RECORDER_H

#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fstream>
#include <ctime>

class SPTAMRecorder{
protected:
	ros::NodeHandle nh;
	ros::Subscriber sptam_sub;
	std::ofstream sptam_ofs;

	double init_time;

public:
	SPTAMRecorder();
	void sptam_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
};
