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

struct PointsWithUncertainties
{
	std::vector<cv::Point3f> points;
	std::vector<double> uncertainties;

	void push_back(const cv::Point3f& point, double uncertainty) {
		points.push_back(point);
		uncertainties.push_back(uncertainty);
	}

	void clear() {
		points.clear();
		uncertainties.clear();
	}

	int size() const {
		return points.size();
	}
};

struct FeaturePoints
{
	std::vector<cv::Point2f> features;
	std::vector<cv::Point3f> points;
	std::vector<double> uncertainties;

	void push_back(const cv::Point2f& feature, const cv::Point3f& point, double uncertainty) {
		features.push_back(feature);
		points.push_back(point);
		uncertainties.push_back(uncertainty);
	}

	void clear() {
		features.clear();
		points.clear();
		uncertainties.clear();
	}

	int size() {
		return points.size();
	}
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
	cv::Mat img0;															// left frame
	cv::Mat img1;															// right frame
  std::vector<cv::Point2f> new_features;		// feature points for new keyframe
  FeaturePoints feature_points;							// points in local frame, could be transforred to world frame by pose
	int feature_points_init_count;
};

struct Frame {
	Pose pose_from_lastKF;
	cv::Mat img;
  std::vector<cv::Point2f> features;
  FeaturePoints feature_points;
};

struct  KFData
{
	double MAX_UNCERTAINTY = 999999.9;

  std::vector<cv::Point2f> lastKF_features;
	std::vector<cv::Point2f> cur_features;
  PointsWithUncertainties points;
	cv::Mat R_lastKF2Cur;
	cv::Mat t_lastKF2Cur;

	KFData(const cv::Mat& _R_lastKF2Cur, const cv::Mat& _t_lastKF2Cur, const FeaturePoints& fts_pts, const std::vector<cv::Point2f>& fts, const std::vector<cv::Point2f>& fts0, const std::vector<cv::Point2f>& fts1) {
		R_lastKF2Cur = _R_lastKF2Cur;
		t_lastKF2Cur = _t_lastKF2Cur;

		lastKF_features.insert(lastKF_features.end(), fts_pts.features.begin(), fts_pts.features.end());
		cur_features.insert(cur_features.end(), fts.begin(), fts.end());
		points.points.insert(points.points.end(), fts_pts.points.begin(), fts_pts.points.end());
		points.uncertainties.insert(points.uncertainties.end(), fts_pts.uncertainties.begin(), fts_pts.uncertainties.end());

		lastKF_features.insert(lastKF_features.end(), fts0.begin(), fts0.end());
		cur_features.insert(cur_features.end(), fts1.begin(), fts1.end());
		for(int i=0; i<fts0.size(); i++) {
			points.points.push_back(cv::Point3f(-1,-1,-1));
			points.uncertainties.push_back(MAX_UNCERTAINTY);
		}
	}
};


#endif
