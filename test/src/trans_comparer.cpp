#include "trans_comparer.hpp"

TransComparer::TransComparer() {
	gt_sub = nh.subscribe("/vicon/firefly_sbx/firefly_sbx", 1000, &TransComparer::gt_Callback, this);
}

void TransComparer::gt_Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	gt_ofs << msg->header.stamp.toSec() - init_time << " "
	   	   << msg->transform.translation.x << " "
	  	   << msg->transform.translation.y << " "
	  	   << msg->transform.translation.z << std::endl;
}
