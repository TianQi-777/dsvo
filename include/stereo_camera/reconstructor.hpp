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

#include "stereo_camera/pixel_edge.hpp"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "data.hpp"

class Reconstructor {
public:
	void reconstructAndBundleAdjust(std::vector<cv::Point2f>& features0, std::vector<cv::Point2f>& features1,
						  const cv::Mat& K, cv::Mat& R, cv::Mat& t, int max_opt_step, int max_reproj_dist,
						  std::vector<PointWithUncertainty>& pts);

	void refinePixel(const cv::Mat& src_img, const cv::Mat& dest_img, const std::vector<cv::Point2f>& src_fts, std::vector<cv::Point2f>& dest_fts);

};
