#ifndef STEREO_CAMERA_HPP
#define STEREO_CAMERA_HPP

#include <dynamic_reconfigure/server.h>
#include <direct_stereo/DirectStereoConfig.h>
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
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PointStamped.h"
#include "state.hpp"
#include <string>
#include <ctime>
#include <thread>

#include "comparers/comparer.hpp"
#include "comparers/odom_comparer.hpp"
#include "comparers/point_comparer.hpp"
#include "comparers/trans_comparer.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct KeyPoint12_LessThan_y
{
    bool operator()(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2) const
    {
        return kp1.pt.y < kp2.pt.y;
    }
};

class StereoCamera
{
private:
	// parameters
  int OF_size = 11;		// optical flow size
	float MAX_VEL = 3;
	double MAX_SCALE = 10;
	int MAX_REPROJ_DIST, KP_BLOCKS, INIT_MAX_KP, BA_MAX_STEP, SCALE_MAX_STEP, SCALE_PYMD, MIN_FEATURE_DIST, PROP_POSE_PYMD, PROP_POSE_ITER, BLUR_SZ, FAST_THRES, MAX_FEATURES, MIN_TRACK_POINTS;
	int FEATURE_OF_PYMD;
	double MONO_INLIER_THRES, QUAL_LEVEL, BLUR_VAR, MIN_TRACK_DIST, INIT_SCALE;
	bool LOOP_CLOSURE, REFINE_PIXEL, DEBUG_FEATURE, TEST_STEREO;

	ros::NodeHandle nh;
	ros::Publisher direct_pcl_pub;
	ros::Publisher stereo_pcl_pub;
	ros::Publisher pose_pub;
	dynamic_reconfigure::Server<direct_stereo::DirectStereoConfig> server;
	dynamic_reconfigure::Server<direct_stereo::DirectStereoConfig>::CallbackType f;

	CameraModel camera0;
	CameraModel camera1;
	cv::Size frame_size;
	Frame last_frame0;
	Frame last_frame1;
	std::vector<KeyFrame> keyframes0;
	std::vector<KeyFrame> keyframes1;
	bool param_changed;
	cv::Mat cam0_Q;	//comparison with stereo match

	Reconstructor reconstructor;
	ScaleOptimizer scale_optimizer;
	LocalKFOptimizer local_KF_optimizer;
	PoseEstimater pose_estimater;

	Comparer* comparer;
	std::ofstream time_ofs;

	int frame_dropped_count;
  double init_time=-1.0;
  double cur_time;
  bool stereo_match_flag;
	// shared_ptr<DirectSolver> directSolver_ptr;

	void truth_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);

	void monoTrack(State& cur_state, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1,
				   Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name);

	void triangulateByStereoMatch(KeyFrame& keyframe, const CameraModel& cam);

	void detectFeatures(const cv::Mat& img, std::vector<cv::KeyPoint>& kps);

	KeyFrame createKeyFrame(const Pose& cur_pose, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const FeaturePoints& feature_points, const CameraModel& cam0=CameraModel());

	void featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features);

	void propagateState(State& cur_state, const cv::Mat& cur_img, Frame& last_frame, const CameraModel& cam, FeaturePoints& cur_feature_points, cv::Mat& prop_img);

	bool reconstructAndOptimize(FeatureTrackingResult feature_result, const KeyFrame& lastKF,
								const CameraModel& cam0, const CameraModel& cam1,
								Pose& cur_pose, FeaturePoints& curKF_fts_pts, cv::Mat& proj_img,
								const cv::Mat& cur_img0, const cv::Mat& cur_img1);
public:
	StereoCamera(const std::vector<double>& E0,
			     const std::vector<double>& K0,
			     const std::vector<double>& frame_size0,
			     const std::vector<double>& dist_coeff0,
			     const std::vector<double>& E1,
			     const std::vector<double>& K1,
			     const std::vector<double>& frame_size1,
			     const std::vector<double>& dist_coeff1,
			     bool cvt2VGA, const string& gt_type);

	void updateConfig(direct_stereo::DirectStereoConfig &config, uint32_t level);

	void track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, double _cur_time);
};

#endif
