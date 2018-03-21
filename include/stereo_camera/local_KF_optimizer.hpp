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
#include "helper.hpp"
#include "local_KF_edge.hpp"

class LocalKFOptimizer{
public:
	bool optimize(const std::vector<KeyFrame>& keyframes, int KF_count, const CameraModel& cam0, const cv::Mat& cur_img0, const FeaturePoints& cur_fts_pts, Pose& cur_pose);

};