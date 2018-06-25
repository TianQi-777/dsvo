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
#include "stereo_processor/g2o_edges/klt_edge.hpp"

class KLT{
public:
	void compute(const cv::Mat& prev_img, const cv::Mat& next_img, const std::vector<cv::Point3f>& prev_fts_d, std::vector<cv::Point2f>& next_fts,
								const cv::Mat& K, const cv::Mat& R, const cv::Mat& t,
								std::vector<uchar>& status, std::vector<float>& err, int maxLevel=1);

private:
	void compute_pymd(const cv::Mat& prev_img, const cv::Mat& next_img, const std::vector<cv::Point3f>& prev_fts_d, std::vector<cv::Point2f>& next_fts,
									const cv::Mat& K, const cv::Mat& R, const cv::Mat& t,
									std::vector<uchar>& status, std::vector<float>& err);
};
