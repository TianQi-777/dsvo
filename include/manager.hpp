#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "stereo_camera.hpp"
#include "imu.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> StereoSyncPolicy;

class Manager {
private:
	State state;

	boost::shared_ptr<IMU> imu;
	boost::shared_ptr<StereoCamera> stereo_cam;

	ros::NodeHandle nh;
	ros::Subscriber imu_sub;
	message_filters::Subscriber<sensor_msgs::Image> *cam0_sub;
	message_filters::Subscriber<sensor_msgs::Image> *cam1_sub;
	message_filters::Synchronizer<StereoSyncPolicy> *sync;

	void imuMessageCallback(const sensor_msgs::ImuConstPtr& imu_cptr);
	void imageMessageCallback(const sensor_msgs::ImageConstPtr& img0_cptr, const sensor_msgs::ImageConstPtr& img1_cptr);

public:
	Manager();
};