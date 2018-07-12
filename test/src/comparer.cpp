#include "comparer.hpp"

Comparer::Comparer() {
	vo_sub = nh.subscribe("cam_pose", 1000, &Comparer::vo_Callback, this);
	vo_ofs.open("vo.txt", std::ofstream::out | std::ofstream::trunc);
	sptam_sub = nh.subscribe("/sptam/robot/pose", 1000, &Comparer::sptam_Callback, this);
	sptam_ofs.open("sptam.txt", std::ofstream::out | std::ofstream::trunc);

	gt_ofs.open("truth.txt", std::ofstream::out | std::ofstream::trunc);

	init_time = -1;
}

void Comparer::vo_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	vo_ofs << msg->header.stamp.toSec() - init_time << " "
			 << msg->pose.covariance[0] << " "
   	   << msg->pose.pose.position.x << " "
  	   << msg->pose.pose.position.y << " "
  	   << msg->pose.pose.position.z << std::endl;
}

void Comparer::sptam_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	sptam_ofs << msg->header.stamp.toSec() - init_time << " "
	   	   << msg->pose.pose.position.x << " "
	  	   << msg->pose.pose.position.y << " "
	  	   << msg->pose.pose.position.z << std::endl;
}
