#include "sptam_recorder.hpp"

SPTAMRecorder::SPTAMRecorder() {
	sptam_sub = nh.subscribe("/sptam/robot/pose", 1000, &SPTAMRecorder::sptam_Callback, this);
	sptam_ofs.open("sptam.txt", std::ofstream::out | std::ofstream::trunc);
	init_time = -1.0;
	cur_time = -1.0;
}

void SPTAMRecorder::sptam_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	sptam_ofs << msg->header.stamp.toSec() - init_time << " "
	   	   << msg->pose.pose.position.x << " "
	  	   << msg->pose.pose.position.y << " "
	  	   << msg->pose.pose.position.z << std::endl;

}
