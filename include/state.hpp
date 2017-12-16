#ifndef STATE_H
#define STATE_H

#include <pcl_ros/point_cloud.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Pose
{
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
};

struct IMU_bias
{
	Eigen::Vector3d acceleration;
	Eigen::Vector3d rotation;
};

struct State {
	// mean
	Pose pose;
	Eigen::Vector3d velocity;
	IMU_bias imu_bias;
	std::vector<cv::Point3d> landmarks;
	cv::Mat landmarks_dscrt;

	// covariance
	Eigen::MatrixXd covariance;

	State();
	ros::NodeHandle nh;
	ros::Publisher pose_pub;
	ros::Publisher pcl_pub;
	void showPose();
};

#endif 