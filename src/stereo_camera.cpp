#include "stereo_camera.hpp"
#define PI 3.1415926
#define MAX_REPROJ_DIST 3

StereoCamera::StereoCamera(const std::vector<double>& E0, 
						   const std::vector<double>& K0, 
						   const std::vector<double>& frame_size0, 
						   const std::vector<double>& dist_coeff0,
						   const std::vector<double>& E1, 
						   const std::vector<double>& K1, 
						   const std::vector<double>& frame_size1, 
						   const std::vector<double>& dist_coeff1) {
	cam0.E = cv::Mat(E0);
	cam0.E = cam0.E.reshape(0,4);
	cv::Mat R0(cam0.E, cv::Rect(0,0,3,3));
	cv::Mat t0 = cam0.E(cv::Rect(3,0,1,3));

	for(int i=0; i<3; i++) {
		for(int j=0; j<4; j++) {
			T_cam0_BS(i,j) = cam0.E.at<double>(i,j);
		}
	}
	T_cam0_BS_inv = T_cam0_BS.inverse();

	cam0.K = cv::Mat::zeros(3,3,CV_64F);
	cam0.K.at<double>(0,0) = K0[0];
	cam0.K.at<double>(1,1) = K0[1];
	cam0.K.at<double>(0,2) = K0[2];
	cam0.K.at<double>(1,2) = K0[3];
	cam0.K.at<double>(2,2) = 1.0;

	cam0.frame_size = cv::Size(frame_size0[0], frame_size0[1]);

	cam0.dist_coeff= dist_coeff0;
	for(auto it:cam0.dist_coeff)

	cam1.E = cv::Mat(E1);
	cam1.E = cam1.E.reshape(0,4);
	cv::Mat R1 = cam1.E(cv::Rect(0,0,3,3));
	cv::Mat t1 = cam1.E(cv::Rect(3,0,1,3));

	cam1.K = cv::Mat::zeros(3,3,CV_64F);
	cam1.K.at<double>(0,0) = K1[0];
	cam1.K.at<double>(1,1) = K1[1];
	cam1.K.at<double>(0,2) = K1[2];
	cam1.K.at<double>(1,2) = K1[3];
	cam1.K.at<double>(2,2) = 1.0;

	cam1.frame_size = cv::Size(frame_size1[0], frame_size1[1]);

	cam1.dist_coeff= dist_coeff1;

	cv::Mat R1T;
	cv::transpose(R1, R1T);
	stereo.R = R1T * R0;
	stereo.t = R1T * (t0 - t1);

	// std::cout<<stereo.t<<std::endl;
	pcl_pub = nh.advertise<PointCloud> ("points2", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 1000);

    directSolver_ptr = shared_ptr<DirectSolver>(new DirectSolver());
}

void StereoCamera::reconstruct3DPts(const std::vector<cv::Point2f>& features0, 
									   const std::vector<cv::Point2f>& features1, 
									   const cv::Mat& R, 
									   const cv::Mat& t, 
									   std::vector<cv::Point3d>& pts){
	//get perspective projection matrix
    cv::Mat Pl, Pr;
    Pl = cam0.K*(cv::Mat_<double>(3,4) <<1,0,0,0,0,1,0,0,0,0,1,0);
    cv::hconcat(R, t, Pr);
    Pr = cam0.K*Pr;

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
  
	    // cv::waitKey(0);
        pts.push_back(cv::Point3d(x,y,z));
    }
}

float StereoCamera::monoTrack(const cv::Mat& last_frame_img, std::vector<cv::Point2f>& last_frame_features, 
	                         const cv::Mat& last_keyframe_img, std::vector<cv::Point2f>& last_keyframe_features, 
	                         const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features, 
	                         const cv::Mat& K, cv::Mat& line_img, cv::Mat& R, cv::Mat& t, cv::Mat& inlier_mask) {
	R = cv::Mat::eye(3,3,CV_64F);
	t = cv::Mat::zeros(3,1,CV_64F);
    cv::hconcat(last_keyframe_img, cur_img, line_img);
    // filter matches
    std::vector<cv::Point2f> last_keyframe_features_inlier, last_frame_features_inlier, cur_features_inlier;

    std::vector<uchar> status;
    std::vector<float> err;
	if(last_keyframe_features.size() < 10) {
		last_keyframe_features.clear();
		return 0.0;
	}
    cv::calcOpticalFlowPyrLK(last_frame_img, cur_img, last_frame_features, cur_features, status, err);
    for(int i=0; i<status.size(); i++){
    	if(!status[i]) continue;
    	if(cv::norm(last_frame_features[i]-cur_features[i]) > 25) continue;

    	last_keyframe_features_inlier.push_back(last_keyframe_features[i]);
    	last_frame_features_inlier.push_back(last_frame_features[i]);
    	cur_features_inlier.push_back(cur_features[i]);
    	cv::line(line_img, last_keyframe_features[i], cv::Point2f(cur_features[i].x+last_keyframe_img.cols, cur_features[i].y), cv::Scalar(255,0,0));
    } 

    last_keyframe_features = last_keyframe_features_inlier; last_keyframe_features_inlier.clear();
	last_frame_features = last_frame_features_inlier; last_frame_features_inlier.clear();
	cur_features = cur_features_inlier;

	if(last_keyframe_features.size() < 10) {
		last_keyframe_features.clear();
		return 0.0;
	}

    //recover pose
    cv::Mat Est;
    Est  = cv::findEssentialMat(last_keyframe_features, cur_features, K, cv::RANSAC, 0.999, 1.0, inlier_mask);
    int inlier_count = cv::recoverPose(Est, last_keyframe_features, cur_features, K, R, t, inlier_mask);

    // std::cout<<"length: "<<cv::norm(t)<<std::endl;
    // std::cout<<"ratio: "<<inlier_count / float(last_keyframe_features.size())<<std::endl;
    return inlier_count / float(last_keyframe_features.size()); 
}

KeyFrame StereoCamera::createKeyFrame(const Pose& cur_pose, const cv::Mat& cur_img0, const cv::Mat& cur_img1, double cur_time){
	std::cout<<"New KeyFrame"<<std::endl;
	KeyFrame keyframe = KeyFrame();

	keyframe.pose = cur_pose;
	keyframe.time = cur_time;
	keyframe.img0 = cur_img0.clone();
	keyframe.img1 = cur_img1.clone();

    // detect feature points
    cv::goodFeaturesToTrack(keyframe.img0, keyframe.features0, 500, 0.01, 5);
    keyframe.init_feature0_count = keyframe.features0.size();

    return keyframe;
}

void StereoCamera::track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, double cur_time) {
	cv::Mat cur_img0, cur_img1;
	cv::undistort(_cur_img0, cur_img0, cam0.K, cam0.dist_coeff);
	cv::undistort(_cur_img0, cur_img1, cam1.K, cam1.dist_coeff);
    cv::GaussianBlur(cur_img0, cur_img0, cv::Size(3,3), 1);
    cv::GaussianBlur(cur_img1, cur_img1, cv::Size(3,3), 1);

	//initialize key_frame
    if(keyframes.empty() || keyframes.back().features0.size() < 10) {
    	KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_img0, cur_img1, cur_time);

	    last_frame.img0 = keyframe.img0.clone(); 
	    last_frame.features0 = keyframe.features0;

	    keyframes.push_back(keyframe);

    	return;
    }

    KeyFrame& last_keyframe = keyframes.back();

	// scale from IMU
	cv::Mat line_img, last_KF_to_cur_R, last_KF_to_cur_t, inlier_mask;
    cv::Vec3d r_vec0, r_vec1;
	std::vector<cv::Point2f> cur_features0, cur_features1;
	float track_inliers = monoTrack(last_frame.img0, last_frame.features0, last_keyframe.img0, last_keyframe.features0, 
									cur_img0, cur_features0, cam0.K, line_img, last_KF_to_cur_R, last_KF_to_cur_t, inlier_mask);

    // update last_frame
	last_frame.img0 = cur_img0.clone();
	last_frame.features0 = cur_features0;

	//TODO: MOVE DOWN
	// scale from IMU
	Eigen::Vector3d translation_to_last_keyframe = T_cam0_BS_inv * (cur_state.pose.position - last_keyframe.pose.position);
	double scale = translation_to_last_keyframe.norm();
	std::cout<<"scale "<<scale<<std::endl;

	if (track_inliers>0.3) 
	{
    	std::cout<<"track frame"<<std::endl;

    	// remove outlier by pose estimation
		std::vector<cv::Point2f> last_keyframe_features0_inlier, cur_features0_inlier;

		assert(last_keyframe.features0.size() == cur_features0.size());
	    for(unsigned int i=0; i<cur_features0.size(); i++) {
	        if(inlier_mask.at<uchar>(i,0)>0) {
		    	last_keyframe_features0_inlier.push_back(last_keyframe.features0[i]);
		    	cur_features0_inlier.push_back(cur_features0[i]);
	        }
	    }

    	// last_KF_to_cur_t = scale*last_KF_to_cur_t;
    	// last_KF_to_cur_t = 0.1*last_KF_to_cur_t;

    	// optimization
        optimize(last_keyframe_features0_inlier, cur_features0_inlier, last_KF_to_cur_R, last_KF_to_cur_t, last_keyframe.img0, last_keyframe.img1);

        // update current state
        Eigen::Isometry3d T_last_KF_to_cur = Eigen::Isometry3d::Identity();
        for(int i=0; i<3; i++) {
        	for(int j=0; j<3; j++) {
        		T_last_KF_to_cur(i,j) = last_KF_to_cur_R.at<double>(i,j);
        	}
        	T_last_KF_to_cur(i,3) = last_KF_to_cur_t.at<double>(i,0);
        }
        // transform to body frame
        T_last_KF_to_cur = T_cam0_BS * T_last_KF_to_cur * T_cam0_BS_inv;

        Eigen::Isometry3d T_last_KF = Eigen::Isometry3d::Identity();
        T_last_KF.translate(cur_state.pose.position);
        T_last_KF.rotate(cur_state.pose.orientation);

        Eigen::Isometry3d T_cur_pose = T_last_KF * T_last_KF_to_cur.inverse();

	    cur_state.pose.position = T_cur_pose.translation();
	    cur_state.pose.orientation = Eigen::Quaterniond(T_cur_pose.rotation());
	    cur_state.velocity = (cur_state.pose.position - last_keyframe.pose.position) / (cur_time - last_keyframe.time);
	    cur_state.showPose();

        // construct new keyframe
		KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_img0, cur_img1, cur_time);
	    keyframes.push_back(keyframe);

	    last_frame.img0 = keyframe.img0.clone(); 
	    last_frame.features0 = keyframe.features0;
		
 	}
}

void StereoCamera::projectToImg(const std::vector<cv::Point3d>& pts, const cv::Mat& K, const cv::Mat& R, const cv::Mat& t, cv::Mat& proj_img) {
    cv::Mat P01;
    cv::hconcat(R, t, P01);
    P01 = K*P01;
    cv::cvtColor(proj_img, proj_img, cv::COLOR_GRAY2BGR);
	for(int i=0; i<pts.size(); i++) {
        cv::Mat X0 = (cv::Mat_<double>(4,1) << pts[i].x, pts[i].y, pts[i].z, 1.0);
        cv::Mat u1 = P01*X0;
        double u = u1.at<double>(0,0) / u1.at<double>(2,0);
        double v = u1.at<double>(1,0) / u1.at<double>(2,0);
        if(0<=u && u<proj_img.cols && 0<=v && v<proj_img.rows) {
	        cv::drawMarker(proj_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
	    }
	}

}

void StereoCamera::optimize(const std::vector<cv::Point2f>& _last_keyframe_features0, const std::vector<cv::Point2f>& _cur_features0,
							cv::Mat last_KF_to_cur_R, cv::Mat last_KF_to_cur_t, const cv::Mat& last_keyframe_img0, const cv::Mat& last_keyframe_img1) 
{
	std::vector<cv::Point3d> _last_keyframe_pts;
	reconstruct3DPts(_last_keyframe_features0, _cur_features0, last_KF_to_cur_R, last_KF_to_cur_t, _last_keyframe_pts);

    std::vector<cv::Point2f> last_keyframe_features0;
	std::vector<cv::Point3d> last_keyframe_pts;

	cv::Mat reproj_left = last_keyframe_img0.clone();
    cv::cvtColor(reproj_left, reproj_left, cv::COLOR_GRAY2BGR);
	for(int i=0; i<_last_keyframe_pts.size(); i++) {
        cv::Mat X0 = (cv::Mat_<double>(3,1) << _last_keyframe_pts[i].x, _last_keyframe_pts[i].y, _last_keyframe_pts[i].z);
        cv::Mat u1 = cam0.K*X0;
        double u = u1.at<double>(0,0) / u1.at<double>(2,0);
        double v = u1.at<double>(1,0) / u1.at<double>(2,0);
		//outlier by reprojection
		if(cv::norm(_last_keyframe_features0[i]-cv::Point2f(u, v)) > MAX_REPROJ_DIST) continue;

        if(0<=u && u<reproj_left.cols && 0<=v && v<reproj_left.rows) {
	        cv::drawMarker(reproj_left, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
	    }

		cv::circle(reproj_left, _last_keyframe_features0[i], 5, cv::Scalar(0,255,0));

		last_keyframe_features0.push_back(_last_keyframe_features0[i]);
		last_keyframe_pts.push_back(_last_keyframe_pts[i]);
	}
	cv::imshow("Reproj", reproj_left);
	cv::waitKey(1);

	cv::Mat proj_before_opt = last_keyframe_img1.clone();
	projectToImg(last_keyframe_pts, cam1.K, stereo.R, stereo.t, proj_before_opt);

    PointCloud::Ptr point_cloud (new PointCloud);
    for(int i=0; i<last_keyframe_pts.size(); i++) {
    	// std::cout<<last_keyframe_pts[i]<<std::endl;
        pcl::PointXYZ p;
        p.x = last_keyframe_pts[i].x;
        p.y = last_keyframe_pts[i].z;
        p.z = -last_keyframe_pts[i].y;
        point_cloud->push_back(p);
    }

    point_cloud->header.frame_id = "/map";
    pcl_pub.publish(point_cloud);


    // optimization
	std::vector<Gray3DPoint> gray3dpoints;
	for(int i=0; i<last_keyframe_pts.size(); i++)
	{
		Eigen::Vector3d pts_eigen;
		pts_eigen << last_keyframe_pts[i].x, last_keyframe_pts[i].y, last_keyframe_pts[i].z;
		Gray3DPoint pt(pts_eigen, float(last_keyframe_img1.at<uchar>(last_keyframe_features0[i].y,last_keyframe_features0[i].x)));
		gray3dpoints.push_back(pt);
	}

    Eigen::Matrix3d r;
    cv::cv2eigen(stereo.R, r);
  
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = stereo.t.at<double>(0,0); 
    T(1,3) = stereo.t.at<double>(1,0); 
    T(2,3) = stereo.t.at<double>(2,0);

    Eigen::Matrix3d K_eigen;
    cv::cv2eigen(cam1.K, K_eigen);

    cout<<stereo.R<<endl<<stereo.t<<endl;
	directSolver_ptr->poseEstimate(gray3dpoints, last_keyframe_img1, K_eigen, T, 500);

	cv::Mat stereo_R = cv::Mat::zeros(3,3,CV_64F);
	cv::Mat stereo_t = cv::Mat::zeros(3,1,CV_64F);

	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			stereo_R.at<double>(i,j) = T(i,j);
		}
		stereo_t.at<double>(i,0) = T(i, 3);
	}
    cout<<stereo_R<<endl<<stereo_t<<endl;

	cv::Mat proj_after_opt = last_keyframe_img1.clone();
	projectToImg(last_keyframe_pts, cam1.K, stereo_R, stereo_t, proj_after_opt);

	cv::Mat proj_img;
	cv::hconcat(proj_before_opt, proj_after_opt, proj_img);
    cv::imshow("proj img", proj_img);
    cv::waitKey(1);

}
