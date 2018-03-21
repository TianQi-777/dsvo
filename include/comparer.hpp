#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PointStamped.h"
#include <fstream>
#include <ctime>

class Comparer{
private:
	ros::NodeHandle nh;
	ros::Subscriber gt_sub;
	ros::Subscriber vo_sub;
	std::ofstream gt_ofs;
	std::ofstream vo_ofs;

	geometry_msgs::PointStamped gt_msg;
	geometry_msgs::PoseStamped vo_msg;

	double init_time;

	bool gt_new_msg;
	bool vo_new_msg;

	void gt_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);
	void vo_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void tryRecord();

public:
	Comparer();
	void reset();
};