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

struct PointWithUncertainty
{
	cv::Point3d point;
	double uncertainty;

	PointWithUncertainty(const cv::Point3d& p, double u) {
		point = p;
		uncertainty = u;
	}
};

struct FeaturePoints
{
	std::vector<cv::Point2f> features;
	std::vector<cv::Point3f> points;
	std::vector<double> uncertainties;
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
	cv::Mat img1;
};

struct Frame {
	Pose pose;
	double time;
	cv::Mat img;
    std::vector<cv::Point2f> features;
    FeaturePoints feature_points;
};

struct  FeatureTrackingResult
{
  std::vector<cv::Point2f> lastKF_features;
  std::vector<cv::Point2f> cur_features;
	cv::Mat R_lastKF2Cur;
	cv::Mat t_lastKF2Cur;

	FeatureTrackingResult(const std::vector<cv::Point2f>& _lastKF_features, const std::vector<cv::Point2f>& _cur_features, const cv::Mat& _R_lastKF2Cur, const cv::Mat& _t_lastKF2Cur) {
		assert(_last_features.size() == _cur_features.size());
  	lastKF_features=_lastKF_features;
  	cur_features=_cur_features;
		R_lastKF2Cur = _R_lastKF2Cur;
		t_lastKF2Cur = _t_lastKF2Cur;
	}
};


#endif
