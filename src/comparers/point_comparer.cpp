#include "comparers/point_comparer.hpp"

PointComparer::PointComparer() {
	gt_sub = nh.subscribe("/leica/position", 1000, &PointComparer::gt_Callback, this);
	reset();
}

void PointComparer::gt_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	if(cur_time > msg->header.stamp.toSec()) {
		reset();
		return;
	}

	gt_ofs << msg->header.stamp.toSec() - init_time << " "
	   	   << msg->point.x << " "
	  	   << msg->point.y << " "
	  	   << msg->point.z << std::endl;

	cur_time = msg->header.stamp.toSec();
}

void PointComparer::write_vo(const Pose& pose, double p_time) {
	if(init_time < 0) {
		init_time = p_time;
	}

	vo_ofs << p_time - init_time << " " 
		   << pose.position(0) << " " 
		   << pose.position(1) << " " 
		   << pose.position(2) << std::endl;
}
