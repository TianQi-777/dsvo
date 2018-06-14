#include "odom_comparer.hpp"

OdomComparer::OdomComparer() {
	gt_sub = nh.subscribe("/odom", 1000, &OdomComparer::gt_Callback, this);
	reset();
}

void OdomComparer::gt_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	if(cur_time > msg->header.stamp.toSec()) {
		reset();
		return;
	}

	gt_ofs << msg->header.stamp.toSec() - init_time << " "
	   	   << msg->pose.pose.position.x << " "
	  	   << msg->pose.pose.position.y << " "
	  	   << msg->pose.pose.position.z << std::endl;

	cur_time = msg->header.stamp.toSec();
}


void OdomComparer::write_vo(const Pose& pose, double p_time, bool stereo_match_flag) {
	if(init_time < 0) {
		init_time = p_time;
	}

	vo_ofs << p_time - init_time << " "
			 << stereo_match_flag << " "
		   << pose.position(0) << " "
		   << pose.position(1) << " "
		   << pose.position(2) << std::endl;
}
