#include <dynamic_reconfigure/server.h>
#include "direct_stereo/DirectStereoConfig.h"
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
#include "geometry_msgs/PointStamped.h"
#include "state.hpp"
#include <string>
#include <fstream>

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
	std::vector<cv::Point2f> features;
	std::vector<cv::Point3f> points;
};

struct CameraModel {
	Eigen::Matrix3d R_C2B;
	Eigen::Vector3d t_C2B;
	Eigen::Matrix3d R_B2C;
	Eigen::Vector3d t_B2C;
	cv::Mat K;
	StereoModel stereo;
	cv::Mat rect_map_x;
	cv::Mat rect_map_y;
};

struct KeyFrame {
	Pose pose;
	double time;
	cv::Mat img0;
    std::vector<cv::Point2f> features0;
    FeaturePoints feature_points;
    int init_feature0_count;
	cv::Mat img1;
};

struct Frame {
	cv::Mat img;
    std::vector<cv::Point2f> features;
    std::vector<cv::Point2f> feature_pnp;
};

struct  FeatureTrackingResult
{
	double track_inliers;
    std::vector<cv::Point2f> lastKF_features;
    std::vector<cv::Point2f> cur_features;
    std::vector<cv::Point2f> cur_feature_pnp;
	cv::Mat inlier_mask;
	cv::Mat R_lastKF2Cur;
	cv::Mat t_lastKF2Cur;

	FeatureTrackingResult() {
	   R_lastKF2Cur = cv::Mat::eye(3,3,CV_64F);
	   t_lastKF2Cur = cv::Mat::zeros(3,1,CV_64F);
	}
};

class StereoCamera
{
private:
	ros::NodeHandle nh;
	ros::Publisher pcl_pub;
	ros::Publisher pose_pub;
	ros::Subscriber truth_sub;
	dynamic_reconfigure::Server<direct_stereo::DirectStereoConfig> server;
	dynamic_reconfigure::Server<direct_stereo::DirectStereoConfig>::CallbackType f;

	CameraModel camera0;
	CameraModel camera1;
	Frame last_frame0;
	Frame last_frame1;
	std::vector<KeyFrame> keyframes0;
	std::vector<KeyFrame> keyframes1;
	double last_time;
	bool param_changed;
	double init_time;

	std::ofstream vo_ofs;
	std::ofstream gt_ofs;

	int frame_dropped_count;
	// shared_ptr<DirectSolver> directSolver_ptr;

	void truth_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);

	void reconstruct3DPts(std::vector<cv::Point2f>& features0, std::vector<cv::Point2f>& features1, 
						  const cv::Mat& K, const cv::Mat& R, const cv::Mat& t,
						  std::vector<cv::Point3d>& pts, cv::Mat& reproj_img);
	
	void projectToImg(const std::vector<cv::Point3d>& pts, double scale, const CameraModel& cam, cv::Mat& proj_img);
	
	void monoTrack(State& cur_state, double cur_time, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1,
				   Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name);

	KeyFrame createKeyFrame(const Pose& cur_pose, double cur_time, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const FeaturePoints& feature_points);

	void featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, const cv::Mat& K, FeatureTrackingResult& feature_tracking_result);

	void propagateState(State& cur_state, double cur_time, const KeyFrame& lastKF, const std::vector<cv::Point2f>& cur_features, const CameraModel& cam);

	bool reconstructAndOptimize(const FeatureTrackingResult feature_result, const KeyFrame& lastKF, 
								const CameraModel& cam0, const CameraModel& cam1,
								Pose& cur_pose, FeaturePoints& lastKF_fts_pts, cv::Mat& proj_img, std::vector<double>& errs);

	bool optimize(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, double& scale, const CameraModel& cam, const cv::Mat& img0, const cv::Mat& img1, std::vector<double>& errs); 
	
	bool optimize_pymd(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, const cv::Mat& X_mat, double& scale, 
					   const cv::Mat& img0, const cv::Mat& img1, const StereoModel& stereo, const cv::Mat& K, std::vector<double>& errs); 
public:
	StereoCamera(const std::vector<double>& E0, 
			     const std::vector<double>& K0, 
			     const std::vector<double>& frame_size0, 
			     const std::vector<double>& dist_coeff0,
			     const std::vector<double>& E1, 
			     const std::vector<double>& K1, 
			     const std::vector<double>& frame_size1, 
			     const std::vector<double>& dist_coeff1);
	
	void updateConfig(direct_stereo::DirectStereoConfig &config, uint32_t level);

	void track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, double cur_time);
};
