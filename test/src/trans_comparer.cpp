#include "trans_comparer.hpp"

TransComparer::TransComparer() {
	gt_sub = nh.subscribe("/vicon/firefly_sbx/firefly_sbx", 1000, &TransComparer::gt_Callback, this);
	reset();
}

void TransComparer::gt_Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	if(cur_time > msg->header.stamp.toSec()) {
		reset();
		return;
	}

	gt_ofs << msg->header.stamp.toSec() - init_time << " "
	   	   << msg->transform.translation.x << " "
	  	   << msg->transform.translation.y << " "
	  	   << msg->transform.translation.z << std::endl;

	cur_time = msg->header.stamp.toSec();
}

void TransComparer::write_vo(const Pose& pose, double p_time, bool stereo_match_flag) {
	if(init_time < 0) {
		init_time = p_time;
	}

	vo_ofs << p_time - init_time << " "
			 << stereo_match_flag << " "
		   << pose.position(0) << " "
		   << pose.position(1) << " "
		   << pose.position(2) << std::endl;
}
