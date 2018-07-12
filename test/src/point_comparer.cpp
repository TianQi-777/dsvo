#include "point_comparer.hpp"

PointComparer::PointComparer() {
	gt_sub = nh.subscribe("/leica/position", 1000, &PointComparer::gt_Callback, this);
}

void PointComparer::gt_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	gt_ofs << msg->header.stamp.toSec() - init_time << " "
	   	   << msg->point.x << " "
	  	   << msg->point.y << " "
	  	   << msg->point.z << std::endl;
}
