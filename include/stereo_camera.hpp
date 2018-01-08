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


typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

struct CameraModel {
	cv::Mat E;
	cv::Mat K;
	cv::Size frame_size;
	cv::Mat dist_coeff;
};

struct KeyFrame {
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
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

class StereoCamera
{
private:
	ros::NodeHandle nh;
	ros::Publisher pose_pub;
	CameraModel cam0;
	CameraModel cam1;
	cv::Mat R01;
	cv::Mat t01;
	Frame last_frame;
	std::vector<KeyFrame> keyframes;
	bool new_keyframe_required;
	KeyFrame createKeyFrame(const cv::Mat& cur_img0, const cv::Mat& cur_img1, const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
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
	void track(const cv::Mat& cur_img0, const cv::Mat& cur_img1);
};
