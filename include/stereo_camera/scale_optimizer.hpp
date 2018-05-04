#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/core/robust_kernel_impl.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/plot.hpp>

#include "data.hpp"
#include "helper.hpp"
#include "stereo_camera/scale_edge.hpp"

class ScaleOptimizer{
public:
	double optimize(const std::vector<cv::Point2f>& fts, std::vector<PointWithUncertainty>& pts, double& scale,
				  const CameraModel& cam1, const cv::Mat& img0, const cv::Mat& img1, int pymd, int max_opt_step);

	double optimize_pymd(const std::vector<cv::Point2f>& fts, const std::vector<Eigen::Vector3d>& pts, std::vector<double>& uncertaintys, 
					   double& scale, const cv::Mat& img0, const cv::Mat& img1, double tx, const Eigen::Matrix3d& K1, int max_opt_step);


};
