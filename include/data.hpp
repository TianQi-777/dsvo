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

#define BATCH_SIZE 7

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


static void project3DPtsToImg(const std::vector<cv::Point3d>& pts, double scale, const CameraModel& cam, cv::Mat& proj_img) {
    cv::Mat P01;
    cv::hconcat(cam.stereo.R, cam.stereo.t, P01);
    P01 = cam.K*P01;
    cv::cvtColor(proj_img, proj_img, cv::COLOR_GRAY2BGR);
	for(int i=0; i<pts.size(); i++) {
        cv::Mat X0 = (cv::Mat_<double>(4,1) << pts[i].x, pts[i].y, pts[i].z, 1.0/scale);
        cv::Mat u1 = P01*X0;
        double u = u1.at<double>(0,0) / u1.at<double>(2,0);
        double v = u1.at<double>(1,0) / u1.at<double>(2,0);
        if(0<=u && u<proj_img.cols && 0<=v && v<proj_img.rows) {
	        cv::drawMarker(proj_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
	    }
	}
}

static double getPixelValue(const cv::Mat& img, double x, double y) {
	int xf=int(x), yf=int(y), xc=xf+1, yc=yf+1;

	if(xf<0 || xf>img.cols || yf<0 || yf>img.rows) {
		return -255.0;
	}

	double xx = x - double(xf);
	double yy = y - double(yf);

	double res=0.0f;
	res += (1.0f-xx)*(1.0f-yy)*double(img.at<uchar>(yf,xf));
	res += (1.0f-xx)*yy*double(img.at<uchar>(yc,xf));
	res += xx*(1.0f-yy)*double(img.at<uchar>(yf,xc));
	res += xx*yy*double(img.at<uchar>(yc,xc));

	return res;
}

static void getBatchAround(const cv::Mat& img, double x, double y, ScaleBatch& batch) {
	int half_size = BATCH_SIZE / 2;
	int count = 0;
	for(double i=x-half_size; i<=x+half_size; i++) {
		for(double j=y-half_size; j<=y+half_size; j++) {
			batch(0, count++) = getPixelValue(img, i, j);
		}
	}
	
}

#endif