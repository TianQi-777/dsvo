#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <iostream>
#include <fstream>

bool new_msg = false;
geometry_msgs::PointStamped new_pose_msg;

void pose_repuberCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  new_pose_msg.header = msg->header;
  new_pose_msg.header.frame_id = "/map";
  new_pose_msg.point = msg->point;
  new_msg = true;
}

int main(int argc, char **argv)
{
  std::ofstream ofs("truth.txt", std::ofstream::trunc);

  ros::init(argc, argv, "pose_repuber");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/leica/position", 1000, pose_repuberCallback);
  ros::Publisher pose_repub = n.advertise<geometry_msgs::PointStamped>("true_pose", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    if(new_msg){
      pose_repub.publish(new_pose_msg);
      ofs << new_pose_msg.point.x<<" "<< new_pose_msg.point.y<<" "<< new_pose_msg.point.z<<std::endl;
      new_msg = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ofs.close();
  return 0;
}