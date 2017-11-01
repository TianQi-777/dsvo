#include "manager.hpp"

std::vector<double> rot2quat(const cv::Mat & rotationMatrix)
{
    double m00 = rotationMatrix.at<double>(0,0);
    double m01 = rotationMatrix.at<double>(0,1);
    double m02 = rotationMatrix.at<double>(0,2);
    double m10 = rotationMatrix.at<double>(1,0);
    double m11 = rotationMatrix.at<double>(1,1);
    double m12 = rotationMatrix.at<double>(1,2);
    double m20 = rotationMatrix.at<double>(2,0);
    double m21 = rotationMatrix.at<double>(2,1);
    double m22 = rotationMatrix.at<double>(2,2);

    double qw = sqrt(1+m00+m11+m22)/2;
    double qx = (m21-m12)/(4*qw);
    double qy = (m02-m20)/(4*qw);
    double qz = (m10-m01)/(4*qw);

    return {qx, qy, qz, qw};
}

Manager::Manager() {
	ros::NodeHandle nhPriv("~");
	std::vector<double> E, K, frame_size, dist_coeff;
	// cam0 model
	if(!nhPriv.getParam("/cam0/T_BS/data", E) 
	|| !nhPriv.getParam("/cam0/intrinsics", K) 
	|| !nhPriv.getParam("/cam0/resolution", frame_size) 
	|| !nhPriv.getParam("/cam0/distortion_coefficients", dist_coeff) 
    )
	{
		ROS_INFO("Fail to get cam0 parameters, exit.");
        return;
	}

	cam0.E = cv::Mat(E);
	cam0.E = cam0.E.reshape(4,4);

	cam0.K = cv::Mat::zeros(3,3,CV_64F);
	cam0.K.at<double>(0,0) = K[0];
	cam0.K.at<double>(1,1) = K[1];
	cam0.K.at<double>(0,2) = K[2];
	cam0.K.at<double>(1,2) = K[3];

	cam0.frame_size = cv::Size(frame_size[0], frame_size[1]);

	cam0.dist_coeff= cv::Mat(dist_coeff);

	// cam1 model
	if(!nhPriv.getParam("/cam1/T_BS/data", E) 
	|| !nhPriv.getParam("/cam1/intrinsics", K) 
	|| !nhPriv.getParam("/cam1/resolution", frame_size) 
	|| !nhPriv.getParam("/cam1/distortion_coefficients", dist_coeff) 
    )
	{
		ROS_INFO("Fail to get cam1 parameters, exit.");
        return;
	}

	cam1.E = cv::Mat(E);
	cam1.E = cam1.E.reshape(4,4);

	cam1.K = cv::Mat::zeros(3,3,CV_64F);
	cam1.K.at<double>(0,0) = K[0];
	cam1.K.at<double>(1,1) = K[1];
	cam1.K.at<double>(0,2) = K[2];
	cam1.K.at<double>(1,2) = K[3];

	cam1.frame_size = cv::Size(frame_size[0], frame_size[1]);

	cam1.dist_coeff= cv::Mat(dist_coeff);

	// imu model
	if(!nhPriv.getParam("/imu/T_BS/data", E) 
	|| !nhPriv.getParam("/imu/gyroscope_noise_density", imu.gyroscope_noise_density) 
	|| !nhPriv.getParam("/imu/gyroscope_random_walk", imu.gyroscope_random_walk) 
	|| !nhPriv.getParam("/imu/accelerometer_noise_density", imu.accelerometer_noise_density) 
	|| !nhPriv.getParam("/imu/accelerometer_random_walk", imu.accelerometer_random_walk) 
    )
	{
		ROS_INFO("Fail to get cam1 parameters, exit.");
        return;
	}

	imu.E = cv::Mat(E);
	imu.E = cam1.E.reshape(4,4);

	// sensor topics
	std::string cam0_topic, cam1_topic, imu_topic;
	if(!nhPriv.getParam("cam0_topic", cam0_topic) 
	|| !nhPriv.getParam("cam1_topic", cam1_topic) 
	|| !nhPriv.getParam("imu_topic", imu_topic) 
    )
	{
		ROS_INFO("Fail to get sensor topics, exit.");
        return;
	}

	cam0_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, cam0_topic, 1);
	cam1_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, cam1_topic, 1);
	imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(nh, imu_topic, 1);	

	sync = new message_filters::Synchronizer<StereoIMUSyncPolicy>(StereoIMUSyncPolicy(10), *cam0_sub, *cam1_sub, *imu_sub);
	sync->registerCallback(boost::bind(&Manager::imageMessageCallback, this, _1, _2, _3));

	pcl_pub = nh.advertise<PCLCloud> ("pcl_points", 100);
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 1000);

}


void Manager::imageMessageCallback(const sensor_msgs::ImageConstPtr& img0_cptr, const sensor_msgs::ImageConstPtr& img1_cptr, const sensor_msgs::ImuConstPtr& imu_cptr){
	cv_bridge::CvImageConstPtr img0_ptr, img1_ptr;
	try
	{
		img0_ptr = cv_bridge::toCvShare(img0_cptr, img0_cptr->encoding);
		img1_ptr = cv_bridge::toCvShare(img1_cptr, img1_cptr->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//image preparation
	cv::Mat img0, img1;
	img0_ptr->image.copyTo(img0);
	img1_ptr->image.copyTo(img1);

    if(last_frame.img0.empty()) {
    	last_frame = Frame();
    	last_frame.img0 = img0;
    	last_frame.img1 = img1;

    	return;
    }

    // detect feature points
    std::vector<cv::Point2f> features0;
    cv::goodFeaturesToTrack(last_frame.img0, features0, 500, 0.01, 5);

	std::vector<cv::Point2f> features1;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(last_frame.img0, img0, features0, features1, status, err);

    // draw matches
    cv::Mat f_img = img0.clone();
    std::vector<cv::Point2f> features0_inlier, features1_inlier;
    for(int i=0; i<status.size(); i++){
    	if(!status[i] || cv::norm(features0[i]-features1[i]) > 25) continue;

    	features0_inlier.push_back(features0[i]);
    	features1_inlier.push_back(features1[i]);
    	cv::line(f_img, features0[i], features1[i], cv::Scalar(255,0,0));
    }
	cv::imshow("temp match", f_img);
    cv::waitKey(1);  

    cv::Mat inliner_mask, Est, R, t;
    Est  = cv::findEssentialMat(features0_inlier, features1_inlier, cam0.K, cv::RANSAC, 0.999, 0.7, inliner_mask);
    recoverPose(Est, features0_inlier, features1_inlier, cam0.K, R, t, inliner_mask);
    features0.clear();
    features1.clear();
    for(unsigned int i=0; i<features0_inlier.size(); i++) {
        if(inliner_mask.at<uchar>(i,0)>0) {
            features0.push_back(features0_inlier[i]);
            features1.push_back(features1_inlier[i]);
        }
    }

    cv::Mat pose_R, pose_t;
    cv::Mat Sigma = cv::Mat::zeros(cv::Size(6,6), CV_64F);
    pose_R = R.inv();
    pose_t = -pose_R*t;


    std::vector<double> quat = rot2quat(pose_R);

    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "/map";

    pose.pose.position.x = pose_t.data[0]/1000.0;
    pose.pose.position.y = pose_t.data[1]/1000.0;
    pose.pose.position.z = pose_t.data[2]/1000.0;
      
    pose.pose.orientation.x = quat[0];
    pose.pose.orientation.y = quat[1];
    pose.pose.orientation.z = quat[2];
    pose.pose.orientation.w = quat[3]; 


    pose_pub.publish(pose);

    std::vector<cv::Point3d> pts;
    reconstruct3DPts(features0, features1, R, t, pts);

    // update last_frame
	last_frame.img0 = img0;
	last_frame.img1 = img1;

	return;
}

void Manager::reconstruct3DPts(const std::vector<cv::Point2f>& features0, const std::vector<cv::Point2f>& features1, const cv::Mat& R, const cv::Mat& t, std::vector<cv::Point3d>& pts){
	//get perspective projection matrix
    cv::Mat Pl, Pr;
    Pl = cam0.K*(cv::Mat_<double>(3,4) <<1,0,0,0,0,1,0,0,0,0,1,0);
    cv::hconcat(R, t, Pr);
    Pr = cam0.K*Pr;

    PCLCloud::Ptr point_cloud (new PCLCloud);

    //reconstruct 3d feature points
    for(unsigned int i=0; i<features0.size(); i++) {
        float ul = features0[i].x;
        float vl = features0[i].y;
        float ur = features1[i].x;
        float vr = features1[i].y;
        cv::Mat ul_skew = (cv::Mat_<double>(3,3) << 0, -1, vl, 1, 0, -ul, -vl, ul, 0);
        cv::Mat ur_skew = (cv::Mat_<double>(3,3) << 0, -1, vr, 1, 0, -ur, -vr, ur, 0);
        cv::Mat uPl = ul_skew*Pl;
        cv::Mat uPr = ur_skew*Pr;
        cv::Mat A, W, U, V;
        cv::vconcat(uPl, uPr, A);
        cv::SVDecomp(A, W, U, V, cv::SVD::FULL_UV);
        cv::transpose(V,V);

        double x = V.at<double>(0,3) / V.at<double>(3,3);
        double y = V.at<double>(1,3) / V.at<double>(3,3);
        double z = V.at<double>(2,3) / V.at<double>(3,3);
  
        pts.push_back(cv::Point3d(x,y,z));

        // if(fabs(x) < 100.0 && fabs(y) < 100.0 && fabs(z) < 100.0)
        {
            pcl::PointXYZ p;
            p.x = x;
            p.y = y;
            p.z = z;
            point_cloud->push_back(p);
        }
    }

    point_cloud->header.frame_id = "/map";
    pcl_pub.publish(point_cloud);
}