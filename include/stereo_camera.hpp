#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "state.hpp"

#define MAX_FEATURE 200

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

struct CameraModel {
	cv::Mat E;
	cv::Mat K;
	cv::Size frame_size;
	cv::Mat dist_coeff;
};

struct KeyFrame {
	cv::Mat img0;
    std::vector<cv::Point2f> features0;
    std::vector<unsigned int> active_features0;
	cv::Mat img1;
    State state;
};

struct Frame {
	cv::Mat img0;
    std::vector<cv::Point2f> features0;
    State state;
};

class StereoCamera
{
private:
	CameraModel cam0;
	CameraModel cam1;
	Frame last_frame;
	KeyFrame last_keyframe;
	bool last_keyframe_required;
	void reconstruct3DPts(const std::vector<cv::Point2f>& features0, 
						  const std::vector<cv::Point2f>& features1, 
						  const cv::Mat& R, 
						  const cv::Mat& t, 
						  std::vector<cv::Point3d>& pts);
public:
	StereoCamera(const std::vector<double>& E0, 
			     const std::vector<double>& K0, 
			     const std::vector<double>& frame_size0, 
			     const std::vector<double>& dist_coeff0,
			     const std::vector<double>& E1, 
			     const std::vector<double>& K1, 
			     const std::vector<double>& frame_size1, 
			     const std::vector<double>& dist_coeff1);
	void track(const cv::Mat& cur_img0, const cv::Mat& cur_img1, State& state);
};
