#include "direct_stereo/DirectStereoConfig.h"
#include "data.hpp"
#include "helper.hpp"
#include "reconstructor.hpp"
#include "scale_optimizer.hpp"
#include "local_KF_optimizer.hpp"
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
#include <dynamic_reconfigure/server.h>
// #include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PointStamped.h"
#include "state.hpp"
#include <string>
#include <ctime>

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class StereoCamera
{
private:
	ros::NodeHandle nh;
	// ros::Publisher pcl_pub;
	ros::Publisher pose_pub;
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

	Reconstructor reconstructor;
	ScaleOptimizer scale_optimizer;
	LocalKFOptimizer local_KF_optimizer;

	int frame_dropped_count;
	// shared_ptr<DirectSolver> directSolver_ptr;

	void truth_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);
		
	void monoTrack(State& cur_state, ros::Time cur_time_ros, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1,
				   Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name);

	KeyFrame createKeyFrame(const Pose& cur_pose, double cur_time, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const FeaturePoints& feature_points);

	void featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, const cv::Mat& K, FeatureTrackingResult& feature_tracking_result);

	void propagateState(State& cur_state, double cur_time, const KeyFrame& lastKF, const std::vector<cv::Point2f>& cur_features, const CameraModel& cam);

	bool reconstructAndOptimize(FeatureTrackingResult feature_result, const KeyFrame& lastKF, 
								const CameraModel& cam0, const CameraModel& cam1,
								Pose& cur_pose, FeaturePoints& curKF_fts_pts, cv::Mat& proj_img, std::vector<double>& errs, const cv::Mat& cur_img0, const cv::Mat& cur_img1);
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

	void track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, ros::Time cur_time_ros);
};
