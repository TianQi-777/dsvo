#ifndef HELPER_HPP
#define HELPER_HPP

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/plot.hpp>
#include "data.hpp"
#include "stereo_camera/pose_edge.hpp"

static void project3DPtsToImg(const std::vector<cv::Point3f>& pts, const cv::Mat& K, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, cv::Mat& proj_img) {
	int marker_size = proj_img.rows / 50;
    cv::Mat P01;
    cv::Mat R_cv, t_cv;
    cv::eigen2cv(R, R_cv);
    cv::eigen2cv(t, t_cv);
    cv::hconcat(R_cv, t_cv, P01);
    P01 = K*P01;
    cv::cvtColor(proj_img, proj_img, cv::COLOR_GRAY2BGR);
	for(int i=0; i<pts.size(); i++) {
        cv::Mat X0 = (cv::Mat_<double>(4,1) << pts[i].x, pts[i].y, pts[i].z, 1.0);
        cv::Mat u1 = P01*X0;
        double u = u1.at<double>(0,0) / u1.at<double>(2,0);
        double v = u1.at<double>(1,0) / u1.at<double>(2,0);
        if(0<=u && u<proj_img.cols && 0<=v && v<proj_img.rows) {
	        cv::drawMarker(proj_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
	    }
	}
}

static void project3DPtsToImg(const std::vector<cv::Point3f>& pts, double scale, const CameraModel& cam, cv::Mat& proj_img) {
	cv::Mat _R;
	cv::Rodrigues(cam.stereo.R, _R);
	std::vector<cv::Point2f> proj_fts;
	cv::projectPoints(pts, _R, cam.stereo.t/scale, cam.K, cv::Mat::zeros(1,4,CV_64F), proj_fts);
	int marker_size = proj_img.rows / 50;
	cv::cvtColor(proj_img, proj_img, cv::COLOR_GRAY2BGR);
	for(int i=0; i<proj_fts.size(); i++) {
        double u = proj_fts[i].x;
        double v = proj_fts[i].y;
        if(0<=u && u<proj_img.cols && 0<=v && v<proj_img.rows) {
	        cv::drawMarker(proj_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
	    }
	}
}

static void project3DPtsToImg(const std::vector<cv::Point3d>& pts, double scale, const CameraModel& cam, cv::Mat& proj_img) {
	int marker_size = proj_img.rows / 50;
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
	        cv::drawMarker(proj_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
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
	double cur_intensity;
	double total_intensity;
	for(double i=x-half_size; i<=x+half_size; i++) {
		for(double j=y-half_size; j<=y+half_size; j++) {
			cur_intensity = getPixelValue(img, i, j);
			total_intensity += cur_intensity;
			batch(0, count++) = cur_intensity;
		}
	}
	double mean = total_intensity / count;
	for(int i=0; i<count; i++) {
		// batch(0,i) = batch(0,i) - mean;
	}
}

#endif
