#include "stereo_rectifier/stereo_rectifier.hpp"

StereoRectifier::StereoRectifier() {
	ros::NodeHandle nhPriv("~");
	std::vector<double> E0, _K0, frame_size0, dist_coeff0;
	std::vector<double> E1, _K1, frame_size1, dist_coeff1;
	// cameras model
	if(!nhPriv.getParam("/cam0/T_BS/data", E0)
	|| !nhPriv.getParam("/cam0/intrinsics", _K0)
	|| !nhPriv.getParam("/cam0/resolution", frame_size0)
	|| !nhPriv.getParam("/cam0/distortion_coefficients", dist_coeff0)
	|| !nhPriv.getParam("/cam1/T_BS/data", E1)
	|| !nhPriv.getParam("/cam1/intrinsics", _K1)
	|| !nhPriv.getParam("/cam1/resolution", frame_size1)
	|| !nhPriv.getParam("/cam1/distortion_coefficients", dist_coeff1)
    )
	{
		ROS_INFO("Fail to get cameras parameters, exit.");
        return;
	}

	// cam0
	cv::Mat cam0_E = cv::Mat(E0);
	cam0_E = cam0_E.reshape(0,4);
	cv::Mat R0(cam0_E, cv::Rect(0,0,3,3));
	cv::Mat t0 = cam0_E(cv::Rect(3,0,1,3));
	cv::Mat K0 = cv::Mat::zeros(3,3,CV_64F);
	K0.at<double>(0,0) = _K0[0];
	K0.at<double>(1,1) = _K0[1];
	K0.at<double>(0,2) = _K0[2];
	K0.at<double>(1,2) = _K0[3];
	K0.at<double>(2,2) = 1.0;
	cv::Size cam0_frame_size = cv::Size(frame_size0[0], frame_size0[1]);

	// cam1
	cv::Mat cam1_E = cv::Mat(E1);
	cam1_E = cam1_E.reshape(0,4);
	cv::Mat R1 = cam1_E(cv::Rect(0,0,3,3));
	cv::Mat t1 = cam1_E(cv::Rect(3,0,1,3));
	cv::Mat K1 = cv::Mat::zeros(3,3,CV_64F);
	K1.at<double>(0,0) = _K1[0];
	K1.at<double>(1,1) = _K1[1];
	K1.at<double>(0,2) = _K1[2];
	K1.at<double>(1,2) = _K1[3];
	K1.at<double>(2,2) = 1.0;
	cv::Size cam1_frame_size = cv::Size(frame_size1[0], frame_size1[1]);

	/***********************stereo rectify begin***********************/
	cv::Mat R1T;
	cv::transpose(R1, R1T);
	cv::Mat R = R1T * R0;
	cv::Mat t = R1T * (t0 - t1);
	cv::Size frame_size = cam0_frame_size;
	bool cvt2VGA = false;
	nhPriv.getParam("cvt2VGA", cvt2VGA);
	if(cvt2VGA) frame_size = cv::Size(640,480);
	cv::Mat rect_R0, rect_R1, rect_P0, rect_P1, Q;
	cv::stereoRectify(K0, dist_coeff0, K1, dist_coeff1,cam0_frame_size, R, t, rect_R0, rect_R1, rect_P0, rect_P1, Q, cv::CALIB_ZERO_DISPARITY, 0, frame_size);
	cv::initUndistortRectifyMap(K0, dist_coeff0, rect_R0, rect_P0, frame_size, CV_32F, rect_map0_x, rect_map0_y);
	cv::initUndistortRectifyMap(K1, dist_coeff1, rect_R1, rect_P1, frame_size, CV_32F, rect_map1_x, rect_map1_y);

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
	sync->registerCallback(boost::bind(&StereoRectifier::imageMessageCallback, this, _1, _2));

	image_transport::ImageTransport it(nh);
	pub0 = it.advertise(RECT_IMG0_TOPIC, 1000);
	pub1 = it.advertise(RECT_IMG1_TOPIC, 1000);
}


void StereoRectifier::imageMessageCallback(const sensor_msgs::ImageConstPtr& img0_cptr, const sensor_msgs::ImageConstPtr& img1_cptr){
	cv_bridge::CvImageConstPtr img0_ptr, img1_ptr;
	try
	{
		img0_ptr = cv_bridge::toCvShare(img0_cptr, sensor_msgs::image_encodings::MONO8);
		img1_ptr = cv_bridge::toCvShare(img1_cptr, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat img0, img1;
	img0_ptr->image.copyTo(img0);
	img1_ptr->image.copyTo(img1);

	cv::remap(img0, img0, rect_map0_x, rect_map0_y, cv::INTER_LINEAR);
	cv::remap(img1, img1, rect_map1_x, rect_map1_y, cv::INTER_LINEAR);

	sensor_msgs::ImagePtr rect_img0 = cv_bridge::CvImage(img0_cptr->header, "mono8", img0).toImageMsg();
	sensor_msgs::ImagePtr rect_img1 = cv_bridge::CvImage(img1_cptr->header, "mono8", img1).toImageMsg();

	pub0.publish(rect_img0);
	pub1.publish(rect_img1);

	return;
}
