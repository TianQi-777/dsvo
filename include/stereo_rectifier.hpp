#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <string>
#include "data.hpp"

class StereoRectifier {
private:

	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image> *cam0_sub;
	message_filters::Subscriber<sensor_msgs::Image> *cam1_sub;
	message_filters::Synchronizer<StereoSyncPolicy> *sync;
	image_transport::Publisher pub0;
	image_transport::Publisher pub1;

	cv::Mat rect_map0_x;
	cv::Mat rect_map0_y;
	cv::Mat rect_map1_x;
	cv::Mat rect_map1_y;

	void imageMessageCallback(const sensor_msgs::ImageConstPtr& img0_cptr, const sensor_msgs::ImageConstPtr& img1_cptr);

public:
	StereoRectifier();
};
