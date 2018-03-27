#ifndef POSE_ESTIMATER_HPP
#define POSE_ESTIMATER_HPP

#include <Eigen/Core>
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

class PoseEstimater {
public:
	void poseEstimate(const FeaturePoints& fts_pts, const cv::Mat& source_img, const cv::Mat& K, const cv::Mat& dest_img, int pymd, Eigen::Matrix3d& R, Eigen::Vector3d& t);
private:
	void poseEstimatePymd(const std::vector<Eigen::Vector3d>& pts, const std::vector<cv::Point2f>& fts, const cv::Mat& source_img, const cv::Mat& dest_img, const Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t);

};

#endif