#ifndef COMPARER_H
#define COMPARER_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <ctime>
#include "state.hpp"

class Comparer{
protected:
	ros::NodeHandle nh;
	// ros::Subscriber vo_sub;
	std::ofstream vo_ofs;
	std::ofstream gt_ofs;

	double init_time;
	double cur_time;
	
public:
	Comparer();
	void reset();

	virtual void write_vo(const Pose& pose, double p_time){};
};


#endif 