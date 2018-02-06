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
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include "state.hpp"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Pose;

struct Gray3DPoint;

struct StereoModel
{
	cv::Mat R;
	cv::Mat t;
};

struct FeaturePoints
{
	cv::Point2f feature;
	cv::Point3d points;
	FeaturePoints(const cv::Point2f& f, const cv::Point3d& p) {
		feature = f;
		points = p;
	}
};

struct CameraModel {
	Eigen::Matrix3d R_C2B;
	Eigen::Vector3d t_C2B;
	Eigen::Matrix3d R_B2C;
	Eigen::Vector3d t_B2C;
	cv::Mat K;
	cv::Mat rect_map_x;
	cv::Mat rect_map_y;
};

struct KeyFrame {
	Pose pose;
	double time;
	cv::Mat img0;
    std::vector<cv::Point2f> features0;
    int init_feature0_count;
	cv::Mat img1;
    std::vector<cv::Point2f> features1;
    int init_feature1_count;
};

struct Frame {
	cv::Mat img0;
    std::vector<cv::Point2f> features0;
	cv::Mat img1;
    std::vector<cv::Point2f> features1;
};

struct  MonoResult
{
    std::vector<cv::Point2f> lastKF_features;
    std::vector<cv::Point2f> cur_features;
	cv::Mat inlier_mask;
	cv::Mat R_lastKF2Cur;
	cv::Mat t_lastKF2Cur;

    MonoResult(const std::vector<cv::Point2f>& f0, const std::vector<cv::Point2f>& f1, const cv::Mat& inl, const cv::Mat& _R, const cv::Mat& _t) {
	   lastKF_features = f0;
	   cur_features = f1;
	   inlier_mask = inl;
	   R_lastKF2Cur = _R;
	   t_lastKF2Cur = _t;
    }
};

class StereoCamera
{
private:
	ros::NodeHandle nh;
	ros::Publisher pcl_pub;
	ros::Publisher pose_pub;
	CameraModel cam0;
	CameraModel cam1;
	Frame last_frame;
	StereoModel stereo;
	StereoModel stereo_inv;
	std::vector<KeyFrame> keyframes;
	double last_time;
	// shared_ptr<DirectSolver> directSolver_ptr;
	KeyFrame createKeyFrame(const Pose& cur_pose, const cv::Mat& cur_img0, const cv::Mat& cur_img1, double cur_time);
	void reconstruct3DPts(std::vector<cv::Point2f>& KF_features, 
						  const std::vector<cv::Point2f>& features1, 
						  const cv::Mat& R, 
						  const cv::Mat& t, 
						  std::vector<cv::Point3d>& pts,
						  cv::Mat& reproj_img);
	float monoTrack(const cv::Mat& last_frame_img, std::vector<cv::Point2f>& last_frame_features, 
	                         const cv::Mat& last_keyframe_img, std::vector<cv::Point2f>& last_keyframe_features, 
	                         const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features, 
	                         const cv::Mat& K, cv::Mat& line_img, cv::Mat& R, cv::Mat& t, cv::Mat& inlier_mask);
	bool reconstructAndOptimize(const MonoResult mono, const StereoModel& stereo, const Pose& lastKF_pose, 
								const cv::Mat& lastKF_pts_img, const cv::Mat& lastKF_proj_img, 
								const CameraModel& cam_pts, const CameraModel& cam_proj,
							    Pose& cur_pose, std::vector<FeaturePoints>& lastKF_fts_pts, cv::Mat& proj_img);
	bool optimize(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, double& scale, const StereoModel& stereo, const cv::Mat& orig_K, const cv::Mat& img0, const cv::Mat& img1); 
	bool optimize_pymd(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, const cv::Mat& X_mat, double& scale, const cv::Mat& img0, const cv::Mat& img1, const StereoModel& stereo, const cv::Mat& K, std::vector<double>& errs); 
	void projectToImg(const std::vector<cv::Point3d>& pts, double scale, const cv::Mat& K, const cv::Mat& R, const cv::Mat& t, cv::Mat& proj_img);
public:
	StereoCamera(const std::vector<double>& E0, 
			     const std::vector<double>& K0, 
			     const std::vector<double>& frame_size0, 
			     const std::vector<double>& dist_coeff0,
			     const std::vector<double>& E1, 
			     const std::vector<double>& K1, 
			     const std::vector<double>& frame_size1, 
			     const std::vector<double>& dist_coeff1);
	void track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, double cur_time);
};
