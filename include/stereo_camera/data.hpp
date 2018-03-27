#ifndef DATA_HPP
#define DATA_HPP

#include "state.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/plot.hpp>

#define BATCH_SIZE 5

typedef Eigen::Matrix<double, 1, BATCH_SIZE*BATCH_SIZE> ScaleBatch;

struct Pose;

struct Gray3DPoint;

struct StereoModel
{
	cv::Mat R;
	cv::Mat t;
};

struct FeaturePoints
{
	std::vector<cv::Point2f> features;
	std::vector<cv::Point3f> points;
};

struct CameraModel {
	Eigen::Matrix3d R_C2B;
	Eigen::Vector3d t_C2B;
	Eigen::Matrix3d R_B2C;
	Eigen::Vector3d t_B2C;
	cv::Mat K;
	StereoModel stereo;
	cv::Mat rect_map_x;
	cv::Mat rect_map_y;
};

struct KeyFrame {
	Pose pose;
	double time;
	cv::Mat img0;
    std::vector<cv::Point2f> features0;
    FeaturePoints feature_points;
    int init_feature0_count;
	cv::Mat img1;
};

struct Frame {
	cv::Mat img;
    std::vector<cv::Point2f> features;
    std::vector<cv::Point2f> feature_pnp;
};

struct  FeatureTrackingResult
{
	double track_inliers;
    std::vector<cv::Point2f> lastKF_features;
    std::vector<cv::Point2f> cur_features;
    std::vector<cv::Point2f> cur_feature_pnp;
	cv::Mat inlier_mask;
	cv::Mat R_lastKF2Cur;
	cv::Mat t_lastKF2Cur;

	FeatureTrackingResult() {
	   R_lastKF2Cur = cv::Mat::eye(3,3,CV_64F);
	   t_lastKF2Cur = cv::Mat::zeros(3,1,CV_64F);
	}
};


#endif