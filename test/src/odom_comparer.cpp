#include "odom_comparer.hpp"

OdomComparer::OdomComparer() {
	gt_sub = nh.subscribe("/odom", 1000, &OdomComparer::gt_Callback, this);
}

void OdomComparer::gt_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	gt_ofs << msg->header.stamp.toSec() - init_time << " "
	   	   << msg->pose.pose.position.x << " "
	  	   << msg->pose.pose.position.y << " "
	  	   << msg->pose.pose.position.z << std::endl;
}
