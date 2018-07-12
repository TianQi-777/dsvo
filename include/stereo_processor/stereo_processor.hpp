#ifndef STEREO_PROCESSOR_HPP
#define STEREO_PROCESSOR_HPP

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
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <ctime>
#include <cmath>

#include "dsvo/dsvoConfig.h"
#include "data.hpp"
#include "state.hpp"
#include "helper.hpp"
#include "pose_estimater.hpp"
#include "reconstructor.hpp"
// #include "klt.hpp"
#include "scale_optimizer.hpp"
// #include "local_KF_optimizer.hpp"

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class StereoProcessor
{
private:
	State state;

	// parameters
  int OF_size = 11;       // optical flow size
	float MAX_VEL = 30;      // max pose propagation velocity

  int BLUR_SZ;            // blur size
  double BLUR_VAR;        // blur value
	int FEATURE_BLOCKS;     // divide img to blocks for feature detection
  int FEATURE_THRES;      // FAST threshold
  int FEATURES_PER_CELL;  // # of features in each block cell
	int FEATURE_OF_PYMD;    // feature tracking pyramid
  int PROP_PYMD;          // pose propagation pyramid
  int PROP_MAX_STEP;      // max step for pose propagation
  double PROP_PROJ_DIST;  // max projection distance for pose propagation
  double PROP_PTS_RATIO;  // minimal inlier ratio for successful pose propagation
  int PROP_MIN_POINTS;    // min # of features for pose propagation
  double KF_DIST;         // minimal Keyframe distance
  double BA_INLIER_THRES; // minimal inlier ratio for successful bundle adjustment
	double BA_REPROJ_DIST;  // bundle adjustment max reprojection distance
  int BA_MAX_STEP;        // max step for bundle adjustment
  int SCALE_PYMD;         // scale optimization pyramid
  int SCALE_MAX_STEP;     // max step for scale optimization
	bool LOOP_CLOSURE;      // flag for local loop detection and optimization
  bool REFINE_PIXEL;      // refine stereo match after scale optimization
  bool DEBUG;             // debug mode, inspecting frame by frame
  bool TEST_STEREO;       // test pure stereo match mode

  // ros modules
	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image> *cam0_sub;
	message_filters::Subscriber<sensor_msgs::Image> *cam1_sub;
	message_filters::Synchronizer<StereoSyncPolicy> *sync;
	ros::Publisher pcl_pub;
	dynamic_reconfigure::Server<dsvo::dsvoConfig> server;
	dynamic_reconfigure::Server<dsvo::dsvoConfig>::CallbackType f;
	void imageMessageCallback(const sensor_msgs::ImageConstPtr& img0_cptr, const sensor_msgs::ImageConstPtr& img1_cptr);

  // stereo components
	CameraModel camera0;
	CameraModel camera1;
	Frame last_frame0;
	Frame last_frame1;
	std::vector<KeyFrame> keyframes0;
	std::vector<KeyFrame> keyframes1;
	bool param_changed;

	// camera pose between consecutive frames
	Eigen::Matrix3d R_last_frame;
	Eigen::Vector3d t_last_frame;

  // components for testing
	std::ofstream time_ofs;
  PointCloud::Ptr point_cloud;
  bool stereo_match_flag;     // stereo match is called if dsvo failed to generate points

  // record times
  double init_time=-1.0;
  double last_time;
  double cur_time;

  // core dsvo modules
	PoseEstimater pose_estimater;
	Reconstructor reconstructor;
	ScaleOptimizer scale_optimizer;
	// KLT klt;
	// LocalKFOptimizer local_KF_optimizer;
	void track(const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, double _cur_time);
	void monoTrack(const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1, Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name);
 	bool propagateState(const cv::Mat& cur_img, KeyFrame& lastKF, Frame& last_frame, const CameraModel& cam, FeaturePoints& cur_feature_points);
 	void featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features);
	bool reconstructAndOptimize(KFData& kf_data, const KeyFrame& lastKF,
								const CameraModel& cam0, const CameraModel& cam1,
								Pose& cur_pose, FeaturePoints& curKF_fts_pts, std::vector<bool>& curKF_new_pts_flags,
								const cv::Mat& cur_img0, const cv::Mat& cur_img1);

  // helper functions
	KeyFrame createKeyFrame(const Pose& cur_pose, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0,
													const FeaturePoints& feature_points=FeaturePoints(), const std::vector<bool>& new_pts_flags=std::vector<bool>());
	void detectFeatures(const cv::Mat& img, std::vector<cv::KeyPoint>& kps, int fts_per_cell);
	void triangulateByStereoMatch(KeyFrame& keyframe, const CameraModel& cam);

public:
	StereoProcessor();

	void updateConfig(dsvo::dsvoConfig &config, uint32_t level);

};

#endif
