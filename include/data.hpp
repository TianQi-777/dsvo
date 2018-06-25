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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> StereoSyncPolicy;

#define RECT_IMG0_TOPIC "rect_img0"
#define RECT_IMG1_TOPIC "rect_img1"

struct Pose;

struct Gray3DPoint;

struct StereoModel
{
	cv::Mat R;
	cv::Mat t;
};

struct CameraModel {
	Eigen::Matrix3d R_C2B;
	Eigen::Vector3d t_C2B;
	Eigen::Matrix3d R_B2C;
	Eigen::Vector3d t_B2C;
	cv::Mat K;
	StereoModel stereo;
};

struct PointWithUncertainty
{
	cv::Point3f point;
	double uncertainty;

	PointWithUncertainty(){};

	PointWithUncertainty(const cv::Point3f& p, double u) {
		point = p;
		uncertainty = u;
	}
};

struct PointsWithUncertainties
{
	std::vector<PointWithUncertainty> pts;

	void clear() {pts.clear();}

	int size() const {return pts.size();}

	PointWithUncertainty operator[](int i) const {return pts[i];}

	PointWithUncertainty& operator[](int i) {return pts[i];}

	void push_back(const PointWithUncertainty& point) {
		pts.push_back(point);
	}

	void push_back(const cv::Point3f& point, double uncertainty) {
		pts.push_back(PointWithUncertainty(point, uncertainty));
	}

	std::vector<cv::Point3f> points() const {
		std::vector<cv::Point3f> points;
		for(int i=0; i<pts.size(); i++) {
			points.push_back(pts[i].point);
		}
		return points;
	}

	std::vector<double> uncertainties() const {
		std::vector<double> uncertainties;
		for(int i=0; i<pts.size(); i++) {
			uncertainties.push_back(pts[i].uncertainty);
		}
		return uncertainties;
	}
};

struct FeaturePoint
{
	cv::Point2f feature;
	PointWithUncertainty point;
	bool published;

	FeaturePoint(){};

	FeaturePoint(const cv::Point2f& _feature, const cv::Point3f& _point, double _uncertainty, bool _published) {
		feature = _feature;
		point = PointWithUncertainty(_point, _uncertainty);
		published = _published;
	}
};

struct FeaturePoints
{
	std::vector<FeaturePoint> fts_pts;

	int size() const {return fts_pts.size();}

	void clear() {fts_pts.clear();}

	bool empty() const {return fts_pts.empty();}

	FeaturePoint operator[](int i) const {return fts_pts[i];}

	FeaturePoint& operator[](int i) {return fts_pts[i];}

	void push_back(const cv::Point2f& feature, const cv::Point3f& point, double uncertainty, bool published=false) {
		FeaturePoint fp(feature, point, uncertainty, published);
		fts_pts.push_back(fp);
	}

	std::vector<cv::Point2f> features() const {
		std::vector<cv::Point2f> features;
		for(int i=0; i<fts_pts.size(); i++) {
			features.push_back(fts_pts[i].feature);
		}
		return features;
	}

	std::vector<cv::Point3f> points() const {
		std::vector<cv::Point3f> points;
		for(int i=0; i<fts_pts.size(); i++) {
			points.push_back(fts_pts[i].point.point);
		}
		return points;
	}

	std::vector<double> uncertainties() const {
		std::vector<double> uncertainties;
		for(int i=0; i<fts_pts.size(); i++) {
			uncertainties.push_back(fts_pts[i].point.uncertainty);
		}
		return uncertainties;
	}
};

struct  KFData
{
	double MAX_UNCERTAINTY = 999999.9;

  std::vector<cv::Point2f> lastKF_features;
	std::vector<cv::Point2f> cur_features;
	PointsWithUncertainties points;
  std::vector<bool> new_pts_flags;
	cv::Mat R_lastKF2Cur;
	cv::Mat t_lastKF2Cur;

	int size() const {return cur_features.size();}

	KFData(const cv::Mat& _R_lastKF2Cur, const cv::Mat& _t_lastKF2Cur, const FeaturePoints& fts_pts, const std::vector<cv::Point2f>& fts, const std::vector<cv::Point2f>& fts0, const std::vector<cv::Point2f>& fts1) {
		R_lastKF2Cur = _R_lastKF2Cur;
		t_lastKF2Cur = _t_lastKF2Cur;

		for(int i=0; i<fts_pts.size(); i++){
			lastKF_features.push_back(fts_pts[i].feature);
			cur_features.push_back(fts[i]);
			points.push_back(fts_pts[i].point);
			new_pts_flags.push_back(false);
		}

		for(int i=0; i<fts0.size(); i++) {
			lastKF_features.push_back(fts0[i]);
			cur_features.push_back(fts1[i]);
			points.push_back(PointWithUncertainty(cv::Point3f(-1,-1,-1), MAX_UNCERTAINTY));
			new_pts_flags.push_back(true);
		}
	}
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


#endif
