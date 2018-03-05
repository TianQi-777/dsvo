#include "comparer.hpp"

Comparer::Comparer() {
	gt_sub = nh.subscribe("/leica/position", 1000, &Comparer::gt_Callback, this);
	vo_sub = nh.subscribe("/cam_pose", 1000, &Comparer::vo_Callback, this);
	reset();
}

void Comparer::gt_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	if(gt_msg.header.stamp.toSec() > msg->header.stamp.toSec()) {
		gt_msg = geometry_msgs::PointStamped();
		reset();
		return;
	}

	gt_msg.header = msg->header;
	gt_msg.point = msg->point;
	gt_new_msg = true;

	tryRecord();
}

void Comparer::vo_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if(init_time < 0) {
		init_time = msg->header.stamp.toSec();
	}

	if(vo_msg.header.stamp.toSec() > msg->header.stamp.toSec()) {
		vo_msg = geometry_msgs::PoseStamped();
		reset();
		return;
	}

	vo_msg.header = msg->header;
	vo_msg.pose = msg->pose;
	vo_new_msg = true;

	tryRecord();
}

void Comparer::tryRecord() {
	// std::cout<<gt_new_msg<<" : "<<vo_new_msg<<std::endl;
	if(gt_new_msg && vo_new_msg) {

		gt_ofs << gt_msg.header.stamp.toSec() - init_time << " "
		   	   << gt_msg.point.x << " "
		  	   << gt_msg.point.y << " "
		  	   << gt_msg.point.z << std::endl;

		vo_ofs << vo_msg.header.stamp.toSec() - init_time << " " 
			   << vo_msg.pose.position.x << " " 
			   << vo_msg.pose.position.y << " " 
			   << vo_msg.pose.position.z << std::endl;

		gt_new_msg = false;
		vo_new_msg = false;
   }
}

void Comparer::reset() {
	gt_new_msg = false;
	vo_new_msg = false;

	init_time = -1.0;

	vo_ofs.close();
	gt_ofs.close();

	vo_ofs.open("vo.txt", std::ofstream::out | std::ofstream::trunc);
	gt_ofs.open("truth.txt", std::ofstream::out | std::ofstream::trunc);

}