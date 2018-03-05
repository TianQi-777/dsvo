#include <Eigen/Core>
#include <Eigen/Geometry>
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
#include "stereo_camera/scale_edge.hpp"

class Optimizer{
public:
	bool optimize(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, double& scale, 
				  const CameraModel& cam0, const CameraModel& cam1, const cv::Mat& img0, const cv::Mat& img1, int pymd, int max_opt_step); 
	
	bool optimize_pymd(const std::vector<cv::Point2f>& fts, const std::vector<Eigen::Vector3d>& pts, double& scale, 
					   const cv::Mat& img0, const cv::Mat& img1, double tx, const Eigen::Matrix3d& K1, int max_opt_step); 

	bool localLoopOptimization(const std::vector<KeyFrame>& keyframes, int KF_count, const CameraModel& cam0, const cv::Mat& cur_img0, const FeaturePoints& cur_fts_pts, Pose& cur_pose);

};