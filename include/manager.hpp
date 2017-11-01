#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2//core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>

struct CameraModel {
	cv::Mat E;
	cv::Mat K;
	cv::Size frame_size;
	cv::Mat dist_coeff;
};

struct IMUModel {
	cv::Mat E;
	double gyroscope_noise_density;
	double gyroscope_random_walk;
	double accelerometer_noise_density;
	double accelerometer_random_walk;
};

struct Frame {
	cv::Mat img0;
	cv::Mat img1;
};

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Imu> StereoIMUSyncPolicy;

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

class Manager {
private:
	CameraModel cam0;
	CameraModel cam1;
	IMUModel imu;
	Frame last_frame;
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image> *cam0_sub;
	message_filters::Subscriber<sensor_msgs::Image> *cam1_sub;
	message_filters::Subscriber<sensor_msgs::Imu> *imu_sub;
	message_filters::Synchronizer<StereoIMUSyncPolicy> *sync;
	ros::Publisher pcl_pub;
	ros::Publisher pose_pub;
	void imageMessageCallback(const sensor_msgs::ImageConstPtr& img0_cptr, const sensor_msgs::ImageConstPtr& img1_cptr, const sensor_msgs::ImuConstPtr& imu);
	void reconstruct3DPts(const std::vector<cv::Point2f>& features0, const std::vector<cv::Point2f>& features1, const cv::Mat& R, const cv::Mat& t, std::vector<cv::Point3d>& pts);

public:
	Manager();
};