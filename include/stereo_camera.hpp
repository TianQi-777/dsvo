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
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include "state.hpp"
#include "directSolver.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Pose;

struct Gray3DPoint;

struct StereoModel
{
	cv::Mat R;
	cv::Mat t;
};

struct CameraModel {
	cv::Mat E;
	cv::Mat K;
	cv::Size frame_size;
	cv::Mat dist_coeff;
};

struct KeyFrame {
	Pose pose;
	double time;
	cv::Mat img0;
    std::vector<cv::Point2f> features0;
    int init_feature0_count;
	cv::Mat img1;
};

struct Frame {
	cv::Mat img0;
    std::vector<cv::Point2f> features0;
};

class StereoCamera
{
private:
	ros::NodeHandle nh;
	ros::Publisher pcl_pub;
	ros::Publisher pose_pub;
	CameraModel cam0;
	CameraModel cam1;
	StereoModel stereo;
	Frame last_frame;
	std::vector<KeyFrame> keyframes;
	shared_ptr<DirectSolver> directSolver_ptr;
	KeyFrame createKeyFrame(const Pose& cur_pose, const cv::Mat& cur_img0, const cv::Mat& cur_img1, double cur_time);
	void reconstruct3DPts(const std::vector<cv::Point2f>& features0, 
						  const std::vector<cv::Point2f>& features1, 
						  const cv::Mat& R, 
						  const cv::Mat& t, 
						  std::vector<cv::Point3d>& pts);
	float monoTrack(const cv::Mat& last_frame_img, std::vector<cv::Point2f>& last_frame_features, 
	                         const cv::Mat& last_keyframe_img, std::vector<cv::Point2f>& last_keyframe_features, 
	                         const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features, 
	                         const cv::Mat& K, cv::Mat& line_img, cv::Mat& R, cv::Mat& t);
	void optimize(const std::vector<cv::Point2f>& last_keyframe_features0, const std::vector<cv::Point2f>& cur_features0,
								cv::Mat R, cv::Mat t, const cv::Mat& last_keyframe_img0, const cv::Mat& last_keyframe_img1); 
	void projectToRight(const std::vector<cv::Point2f>& last_keyframe_features0, const std::vector<cv::Point3d>& last_keyframe_pts, const cv::Mat& last_keyframe_img0,
                        const cv::Mat& stereo_R, const cv::Mat& stereo_t, const cv::Mat& last_keyframe_img1, cv::Mat& proj_img);
public:
	StereoCamera(const std::vector<double>& E0, 
			     const std::vector<double>& K0, 
			     const std::vector<double>& frame_size0, 
			     const std::vector<double>& dist_coeff0,
			     const std::vector<double>& E1, 
			     const std::vector<double>& K1, 
			     const std::vector<double>& frame_size1, 
			     const std::vector<double>& dist_coeff1);
	void track(State& cur_state, const cv::Mat& cur_img0, const cv::Mat& cur_img1, double cur_time);
};
