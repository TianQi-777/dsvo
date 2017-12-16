#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <queue>
#include "state.hpp"

#define MAX_FEATURE 200
#define MAX_FRAME_QUEUE_SIZE 5

struct CameraModel {
	cv::Mat T_BS;
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
    std::vector<cv::Point3d> pts;
    std::vector<double> var;
    State state;
};

class StereoCamera
{
private:
	CameraModel cam0;
	CameraModel cam1;
	cv::Mat cR;
	cv::Mat ct;
	std::queue<Frame> last_frames;
	KeyFrame last_keyframe;
	ros::NodeHandle nh;
	int frame_count;
	void stereoMatch(const cv::Mat& img0, const cv::Mat& img1, std::vector<cv::Point3d>& pts, cv::Mat& kp0_dscrpt);
	void reconstruct3DPts(const std::vector<cv::Point2f>& features0, 
						  const std::vector<cv::Point2f>& features1, 
						  std::vector<cv::Point3d>& pts);
public:
	StereoCamera(const std::vector<double>& T_BS0, 
			     const std::vector<double>& K0, 
			     const std::vector<double>& frame_size0, 
			     const std::vector<double>& dist_coeff0,
			     const std::vector<double>& T_BS1, 
			     const std::vector<double>& K1, 
			     const std::vector<double>& frame_size1, 
			     const std::vector<double>& dist_coeff1);
	void track(const cv::Mat& cur_img0, const cv::Mat& cur_img1, State& cur_state);
	void stereoTrack(const cv::Mat& cur_img0, const cv::Mat& cur_img1, State& cur_state);
};
