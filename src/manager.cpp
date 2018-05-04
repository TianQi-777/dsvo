#include "manager.hpp"

Manager::Manager() {
	ros::NodeHandle nhPriv("~");
	std::vector<double> T_BS0, K0, frame_size0, dist_coeff0;
	std::vector<double> T_BS1, K1, frame_size1, dist_coeff1;
	// cameras model
	if(!nhPriv.getParam("/cam0/T_BS/data", T_BS0)
	|| !nhPriv.getParam("/cam0/intrinsics", K0)
	|| !nhPriv.getParam("/cam0/resolution", frame_size0)
	|| !nhPriv.getParam("/cam0/distortion_coefficients", dist_coeff0)
	|| !nhPriv.getParam("/cam1/T_BS/data", T_BS1)
	|| !nhPriv.getParam("/cam1/intrinsics", K1)
	|| !nhPriv.getParam("/cam1/resolution", frame_size1)
	|| !nhPriv.getParam("/cam1/distortion_coefficients", dist_coeff1)
    )
	{
		ROS_INFO("Fail to get cameras parameters, exit.");
        return;
	}

	bool cvt2VGA = false;
	nhPriv.getParam("cvt2VGA", cvt2VGA);
	std::cout<<"cvt2VGA "<<cvt2VGA<<std::endl;

	// ground truth type
	string gt_type = "";
	nhPriv.getParam("gt_type", gt_type);
	stereo_cam = boost::shared_ptr<StereoCamera>(new StereoCamera(T_BS0, K0, frame_size0, dist_coeff0,
																  T_BS1, K1, frame_size1, dist_coeff1,
																  cvt2VGA, gt_type));

	// sensor topics
	std::string cam0_topic, cam1_topic;
	if(!nhPriv.getParam("cam0_topic", cam0_topic)
	|| !nhPriv.getParam("cam1_topic", cam1_topic) )
	{
		ROS_INFO("Fail to get sensor topics, exit.");
        return;
	}

	cam0_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, cam0_topic, 1000);
	cam1_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, cam1_topic, 1000);

	sync = new message_filters::Synchronizer<StereoSyncPolicy>(StereoSyncPolicy(10), *cam0_sub, *cam1_sub);
	sync->registerCallback(boost::bind(&Manager::imageMessageCallback, this, _1, _2));
}


void Manager::imageMessageCallback(const sensor_msgs::ImageConstPtr& img0_cptr, const sensor_msgs::ImageConstPtr& img1_cptr){
	cv_bridge::CvImageConstPtr img0_ptr, img1_ptr;
	try
	{
		img0_ptr = cv_bridge::toCvShare(img0_cptr, sensor_msgs::image_encodings::BGR8);
		img1_ptr = cv_bridge::toCvShare(img1_cptr, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat cur_img0, cur_img1;

	img0_ptr->image.copyTo(cur_img0);
	img1_ptr->image.copyTo(cur_img1);
	cv::cvtColor(cur_img0, cur_img0, CV_BGR2GRAY);
	cv::cvtColor(cur_img1, cur_img1, CV_BGR2GRAY);

	stereo_cam->track(state, cur_img0, cur_img1, img0_cptr->header.stamp.toSec());

	return;
}
