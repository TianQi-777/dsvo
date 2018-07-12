#ifndef COMPARER_H
#define COMPARER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fstream>
#include <ctime>
#include "state.hpp"

class Comparer{
protected:
	ros::NodeHandle nh;
	ros::Subscriber gt_sub;
	std::ofstream gt_ofs;
	ros::Subscriber vo_sub;
	std::ofstream vo_ofs;
	ros::Subscriber sptam_sub;
	std::ofstream sptam_ofs;
	double init_time;

public:
	Comparer();

	void vo_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void sptam_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
};


#endif
