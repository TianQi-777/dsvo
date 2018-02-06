#include "stereo_camera.hpp"
#define MAX_REPROJ_DIST 1
#define KP_BLOCKS 2
#define INIT_MAX_KP 100
#define NEW_KF_KP_RATIO 0.3
#define MAX_OPT_STEP 100
#define MIN_OPT_STEP 10
#define WINDOW_SIZE 7
#define LM_DAMP 500.0
#define PYMD 5
#define MONO_INLIER_THRES 0.5

double getPixelValue(const cv::Mat img, double x, double y) {
	int xf=int(x), yf=int(y), xc=xf+1, yc=yf+1;

	if(xf<0 || xf>img.cols || yf<0 || yf>img.rows) {
		return -255.0;
	}

	double xx = x - double(xf);
	double yy = y - double(yf);

	double res=0.0f;
	res += (1.0f-xx)*(1.0f-yy)*double(img.at<uchar>(yf,xf));
	res += (1.0f-xx)*yy*double(img.at<uchar>(yc,xf));
	res += xx*(1.0f-yy)*double(img.at<uchar>(yf,xc));
	res += xx*yy*double(img.at<uchar>(yc,xc));

	return res;
}

void getBatchAround(const cv::Mat img, double x, double y, Eigen::MatrixXd& batch) {
	assert((WINDOW_SIZE/2)*2 != WINDOW_SIZE);

	batch.resize(1,WINDOW_SIZE*WINDOW_SIZE);
	int half_size = WINDOW_SIZE / 2;
	int count = 0;
	for(double i=x-half_size; i<=x+half_size; i++) {
		for(double j=y-half_size; j<=y+half_size; j++) {
			batch(0, count++) = getPixelValue(img, i, j);
		}
	}
	
}

StereoCamera::StereoCamera(const std::vector<double>& E0, 
						   const std::vector<double>& K0, 
						   const std::vector<double>& frame_size0, 
						   const std::vector<double>& dist_coeff0,
						   const std::vector<double>& E1, 
						   const std::vector<double>& K1, 
						   const std::vector<double>& frame_size1, 
						   const std::vector<double>& dist_coeff1) {
	cv::Mat cam0_E = cv::Mat(E0);
	cam0_E = cam0_E.reshape(0,4);
	cv::Mat R0(cam0_E, cv::Rect(0,0,3,3));
	cv::Mat t0 = cam0_E(cv::Rect(3,0,1,3));

	cam0.K = cv::Mat::zeros(3,3,CV_64F);
	cam0.K.at<double>(0,0) = K0[0];
	cam0.K.at<double>(1,1) = K0[1];
	cam0.K.at<double>(0,2) = K0[2];
	cam0.K.at<double>(1,2) = K0[3];
	cam0.K.at<double>(2,2) = 1.0;

	cv::Size cam0_frame_size = cv::Size(frame_size0[0], frame_size0[1]);

	cv::Mat cam1_E = cv::Mat(E1);
	cam1_E = cam1_E.reshape(0,4);
	cv::Mat R1 = cam1_E(cv::Rect(0,0,3,3));
	cv::Mat t1 = cam1_E(cv::Rect(3,0,1,3));

	cam1.K = cv::Mat::zeros(3,3,CV_64F);
	cam1.K.at<double>(0,0) = K1[0];
	cam1.K.at<double>(1,1) = K1[1];
	cam1.K.at<double>(0,2) = K1[2];
	cam1.K.at<double>(1,2) = K1[3];
	cam1.K.at<double>(2,2) = 1.0;

	cv::Size cam1_frame_size = cv::Size(frame_size1[0], frame_size1[1]);


	cv::Mat R1T;
	cv::transpose(R1, R1T);
	stereo.R = R1T * R0;
	stereo.t = R1T * (t0 - t1);

    cv::Mat rect_R0, rect_R1, rect_P0, rect_P1, _Q;
    cv::stereoRectify(cam0.K, dist_coeff0, cam1.K, dist_coeff1,cam0_frame_size, stereo.R, stereo.t, rect_R0, rect_R1, rect_P0, rect_P1, _Q, cv::CALIB_ZERO_DISPARITY, 0);

    cv::initUndistortRectifyMap(cam0.K, dist_coeff0, rect_R0, rect_P0, cam0_frame_size,CV_32F, cam0.rect_map_x, cam0.rect_map_y);
    cv::initUndistortRectifyMap(cam1.K, dist_coeff1, rect_R1, rect_P1, cam1_frame_size, CV_32F, cam1.rect_map_x, cam1.rect_map_y);

    // modify external matrix by rect rotation
	cv::Mat rR0T, rR1T;
	cv::transpose(rect_R0, rR0T);
	cv::transpose(rect_R1, rR1T);
    R0 = R0 * rR0T;
    R1 = R1 * rR1T;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			cam0.R_C2B(i,j) = R0.at<double>(i,j);
			cam1.R_C2B(i,j) = R1.at<double>(i,j);
		}
		cam0.t_C2B(i) = t0.at<double>(i,0);
		cam1.t_C2B(i) = t1.at<double>(i,0);
	}
	cam0.R_B2C = cam0.R_C2B.transpose();
	cam0.t_B2C = - cam0.R_B2C * cam0.t_C2B;
	cam1.R_B2C = cam1.R_C2B.transpose();
	cam1.t_B2C = - cam1.R_B2C * cam1.t_C2B;

    cv::Mat tmp = rect_P0(cv::Rect(0,0,3,3));
    tmp.copyTo(cam0.K);
    tmp = rect_P1(cv::Rect(0,0,3,3));
    tmp.copyTo(cam1.K);

	stereo.R = cv::Mat::eye(3,3,CV_64F);
	tmp = rect_P1(cv::Rect(3,0,1,3));
	tmp.copyTo(stereo.t);
	stereo.t = stereo.t / cam0.K.at<double>(0,0);

	stereo_inv.R = stereo.R;
	stereo_inv.t = -stereo.t;

	// std::cout<<stereo.t<<std::endl;
	pcl_pub = nh.advertise<PointCloud> ("points2", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 1000);

    last_time = -1;
    // directSolver_ptr = shared_ptr<DirectSolver>(new DirectSolver());
}

void StereoCamera::reconstruct3DPts(std::vector<cv::Point2f>& KF_features, 
									const std::vector<cv::Point2f>& features1, 
									const cv::Mat& R, 
									const cv::Mat& t, 
									std::vector<cv::Point3d>& pts,
									cv::Mat& reproj_img){
	//get perspective projection matrix
    cv::Mat Pl, Pr;
    Pl = cam0.K*(cv::Mat_<double>(3,4) <<1,0,0,0,0,1,0,0,0,0,1,0);
    cv::hconcat(R, t, Pr);
    Pr = cam0.K*Pr;

    //reconstruct 3d feature points
	std::vector<cv::Point3d> _pts;
	std::vector<cv::Point2f> _KF_features = KF_features;
    for(unsigned int i=0; i<KF_features.size(); i++) {
        float ul = _KF_features[i].x;
        float vl = _KF_features[i].y;
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
        _pts.push_back(cv::Point3d(x,y,z));
    }

	// remove outlier by reproject to left img
    cv::cvtColor(reproj_img, reproj_img, cv::COLOR_GRAY2BGR);
	for(int i=0; i<_pts.size(); i++) {
        cv::Mat X0 = (cv::Mat_<double>(3,1) << _pts[i].x, _pts[i].y, _pts[i].z);
        cv::Mat u1 = cam0.K*X0;
        double u = u1.at<double>(0,0) / u1.at<double>(2,0);
        double v = u1.at<double>(1,0) / u1.at<double>(2,0);

		// reject outlier by reprojection
		if(cv::norm(_KF_features[i]-cv::Point2f(u, v)) > MAX_REPROJ_DIST) continue;

        if(0<=u && u<reproj_img.cols && 0<=v && v<reproj_img.rows) {
	        cv::drawMarker(reproj_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
	    }

		cv::circle(reproj_img, _KF_features[i], 5, cv::Scalar(0,255,0));

		KF_features.push_back(_KF_features[i]);
		pts.push_back(_pts[i]);
	}
		
}

float StereoCamera::monoTrack(const cv::Mat& last_frame_img, std::vector<cv::Point2f>& last_frame_features, 
	                         const cv::Mat& lastKF_img, std::vector<cv::Point2f>& lastKF_features, 
	                         const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features, 
	                         const cv::Mat& K, cv::Mat& line_img, cv::Mat& R, cv::Mat& t, cv::Mat& inlier_mask) {
	R = cv::Mat::eye(3,3,CV_64F);
	t = cv::Mat::zeros(3,1,CV_64F);
    cv::hconcat(lastKF_img, cur_img, line_img);
    // filter matches
    std::vector<cv::Point2f> lastKF_features_inlier, last_frame_features_inlier, cur_features_inlier;

    std::vector<uchar> status;
    std::vector<float> err;
	if(lastKF_features.size() < 10) {
		lastKF_features.clear();
		return 0.0;
	}
    cv::calcOpticalFlowPyrLK(last_frame_img, cur_img, last_frame_features, cur_features, status, err);
    for(int i=0; i<status.size(); i++){
    	if(!status[i]) continue;
    	if(cv::norm(last_frame_features[i]-cur_features[i]) > 9) continue;

    	lastKF_features_inlier.push_back(lastKF_features[i]);
    	last_frame_features_inlier.push_back(last_frame_features[i]);
    	cur_features_inlier.push_back(cur_features[i]);
    	cv::line(line_img, lastKF_features[i], cv::Point2f(cur_features[i].x+lastKF_img.cols, cur_features[i].y), cv::Scalar(255,0,0));
    } 

    lastKF_features = lastKF_features_inlier; lastKF_features_inlier.clear();
	last_frame_features = last_frame_features_inlier; last_frame_features_inlier.clear();
	cur_features = cur_features_inlier;

	if(lastKF_features.size() < 10) {
		lastKF_features.clear();
		return 0.0;
	}

    //recover pose
    cv::Mat Est;
    Est  = cv::findEssentialMat(lastKF_features, cur_features, K, cv::RANSAC, 0.99, 1.0, inlier_mask);
    int inlier_count = cv::recoverPose(Est, lastKF_features, cur_features, K, R, t, inlier_mask);

    // std::cout<<"length: "<<cv::norm(t)<<std::endl;
    // std::cout<<"ratio: "<<inlier_count / float(lastKF_features.size())<<std::endl;
    return inlier_count / float(lastKF_features.size()); 
}

KeyFrame StereoCamera::createKeyFrame(const Pose& cur_pose, const cv::Mat& cur_img0, const cv::Mat& cur_img1, double cur_time){
	KeyFrame keyframe = KeyFrame();

	keyframe.pose = cur_pose;
	keyframe.time = cur_time;
	keyframe.img0 = cur_img0.clone();
	keyframe.img1 = cur_img1.clone();

    // detect feature points
	int subCols = keyframe.img0.cols / KP_BLOCKS;
	int subRows = keyframe.img0.rows / KP_BLOCKS;
    for(int i=0; i<KP_BLOCKS; i++) {
    	for(int j=0; j<KP_BLOCKS; j++) {
    		cv::Rect subRegion = cv::Rect(i*subCols, j*subRows, subCols, subRows);
    		std::vector<cv::Point2f> subFeatures0, subFeatures1;
		    cv::goodFeaturesToTrack(keyframe.img0(subRegion), subFeatures0, INIT_MAX_KP, 0.01, 5);
		    for(int k=0; k<subFeatures0.size(); k++) {
		    	keyframe.features0.push_back(cv::Point2f(subFeatures0[k].x+i*subCols, subFeatures0[k].y+j*subRows));
		    }
		    cv::goodFeaturesToTrack(keyframe.img1(subRegion), subFeatures1, INIT_MAX_KP, 0.01, 5);
		    for(int k=0; k<subFeatures1.size(); k++) {
		    	keyframe.features1.push_back(cv::Point2f(subFeatures1[k].x+i*subCols, subFeatures1[k].y+j*subRows));
		    }
    	}
    }
    keyframe.init_feature0_count = keyframe.features0.size();
    keyframe.init_feature1_count = keyframe.features1.size();

    // cv::Mat feature_points = keyframe.img0.clone();
    // for(int i=0; i<keyframe.features0.size(); i++) {
    // 	cv::circle(feature_points, keyframe.features0[i], 5, cv::Scalar(0,255,0));
    // }
		  //   cv::imshow("sub", feature_points);
		  //   cv::waitKey();

    return keyframe;
}

void StereoCamera::projectToImg(const std::vector<cv::Point3d>& pts, double scale, const cv::Mat& K, const cv::Mat& R, const cv::Mat& t, cv::Mat& proj_img) {
    cv::Mat P01;
    cv::hconcat(R, t, P01);
    P01 = K*P01;
    cv::cvtColor(proj_img, proj_img, cv::COLOR_GRAY2BGR);
	for(int i=0; i<pts.size(); i++) {
        cv::Mat X0 = (cv::Mat_<double>(4,1) << pts[i].x, pts[i].y, pts[i].z, 1.0/scale);
        cv::Mat u1 = P01*X0;
        double u = u1.at<double>(0,0) / u1.at<double>(2,0);
        double v = u1.at<double>(1,0) / u1.at<double>(2,0);
        if(0<=u && u<proj_img.cols && 0<=v && v<proj_img.rows) {
	        cv::drawMarker(proj_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
	    }
	}

}

void StereoCamera::track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, double cur_time) {
	if(last_time < 0 || last_time > cur_time) {
		cur_state.reset();
		keyframes.clear();
		last_time = cur_time;
	}

	//stereo rectify
	cv::Mat cur_img0, cur_img1;
    cv::remap(_cur_img0, _cur_img0, cam0.rect_map_x, cam0.rect_map_y, cv::INTER_LINEAR);
    cv::GaussianBlur(_cur_img0, cur_img0, cv::Size(3,3), 0);
    cv::remap(_cur_img1, _cur_img1, cam1.rect_map_x, cam1.rect_map_y, cv::INTER_LINEAR);
    cv::GaussianBlur(_cur_img1, cur_img1, cv::Size(3,3), 0);

	//initialize key_frame
    if(keyframes.empty() 
    || keyframes.back().features0.size() < 50 
    || keyframes.back().features1.size() < 50) {
    	if(keyframes.empty()) {
    		std::cout<<"Init KeyFrame"<<std::endl;
    	} else {
			std::cout<<"Drop Frame"<<std::endl;
		}
    	KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_img0, cur_img1, cur_time);

	    last_frame.img0 = keyframe.img0.clone(); 
	    last_frame.features0 = keyframe.features0;
	    last_frame.img1 = keyframe.img1.clone(); 
	    last_frame.features1 = keyframe.features1;

	    keyframes.push_back(keyframe);

    	return;
    }

    KeyFrame& lastKF = keyframes.back();

	//mono tracking
	cv::Mat line_img0, R_lastKF2Cur0, t_lastKF2Cur0, inlier_mask0;
	cv::Mat line_img1, R_lastKF2Cur1, t_lastKF2Cur1, inlier_mask1;
	std::vector<cv::Point2f> cur_features0, cur_features1;
	float track_inliers0 = monoTrack(last_frame.img0, last_frame.features0, lastKF.img0, lastKF.features0, 
									cur_img0, cur_features0, cam0.K, line_img0, R_lastKF2Cur0, t_lastKF2Cur0, inlier_mask0);
	float track_inliers1 = monoTrack(last_frame.img1, last_frame.features1, lastKF.img1, lastKF.features1, 
									cur_img1, cur_features1, cam1.K, line_img1, R_lastKF2Cur1, t_lastKF2Cur1, inlier_mask1);


    // update last_frame
	last_frame.img0 = cur_img0.clone();
	last_frame.features0 = cur_features0;
	last_frame.img1 = cur_img1.clone();
	last_frame.features1 = cur_features1;

	if (track_inliers0>MONO_INLIER_THRES || track_inliers1>MONO_INLIER_THRES) 
	// if (track_inliers1>MONO_INLIER_THRES) 
	// if (track_inliers0>MONO_INLIER_THRES)
	{
		// scale from IMU
		Pose new_cur_pose = cur_state.pose;
		bool left_succ = false;
		bool right_succ = false;
		
		if (track_inliers1>track_inliers0) 
		{
			// cv::imshow("right match", line_img1);
			MonoResult mono10(lastKF.features1, cur_features1, inlier_mask1, R_lastKF2Cur1, t_lastKF2Cur1);
			cv::Mat proj_img;
			std::vector<FeaturePoints> lastKF_fts_pts0;
			right_succ = reconstructAndOptimize(mono10, stereo_inv, lastKF.pose, lastKF.img1, lastKF.img0, cam1, cam0, new_cur_pose, lastKF_fts_pts0, proj_img);
			if(right_succ) {
				cv::imshow("Right reproj", proj_img);
			}
		} 
		else {
			// cv::imshow("left match", line_img0);
			MonoResult mono01(lastKF.features0, cur_features0, inlier_mask0, R_lastKF2Cur0, t_lastKF2Cur0);
			cv::Mat proj_img;
			std::vector<FeaturePoints> lastKF_fts_pts1;
			left_succ = reconstructAndOptimize(mono01, stereo, lastKF.pose, lastKF.img0, lastKF.img1, cam0, cam1, new_cur_pose, lastKF_fts_pts1, proj_img);
			if(left_succ) {
				cv::imshow("Left reproj", proj_img);
			}
		}

		if(right_succ || left_succ) {
		    cur_state.pose = new_cur_pose;
		    cur_state.velocity = (cur_state.pose.position - lastKF.pose.position) / (cur_time - lastKF.time);
		    cur_state.showPose();
		}

		cv::waitKey(1);
        // construct new keyframe
		// std::cout<<"New KeyFrame"<<std::endl;
		KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_img0, cur_img1, cur_time);
	    keyframes.push_back(keyframe);
	    last_frame.img0 = keyframe.img0.clone(); 
	    last_frame.features0 = keyframe.features0;
	    last_frame.img1 = keyframe.img1.clone(); 
	    last_frame.features1 = keyframe.features1;
 	}
}

bool StereoCamera::reconstructAndOptimize(const MonoResult mono, const StereoModel& stereo, const Pose& lastKF_pose, 
										  const cv::Mat& lastKF_pts_img, const cv::Mat& lastKF_proj_img, 
										  const CameraModel& cam_pts, const CameraModel& cam_proj,
										  Pose& cur_pose, std::vector<FeaturePoints>& lastKF_fts_pts, cv::Mat& proj_img)
{
	assert(mono.lastKF_features.size() == mono.cur_features.size());
	// remove outlier by pose estimation
	std::vector<cv::Point2f> lastKF_features_inlier, cur_features_inlier;
    for(unsigned int i=0; i<mono.cur_features.size(); i++) {
        if(mono.inlier_mask.at<uchar>(i,0)>0) {
	    	lastKF_features_inlier.push_back(mono.lastKF_features[i]);
	    	cur_features_inlier.push_back(mono.cur_features[i]);
        }
    }

    //reconstruct 3D pts
	cv::Mat reproj = lastKF_pts_img.clone();
	std::vector<cv::Point3d> lastKF_pts;
	reconstruct3DPts(lastKF_features_inlier, cur_features_inlier, mono.R_lastKF2Cur, mono.t_lastKF2Cur, lastKF_pts, reproj);

	// current scale
	Eigen::Matrix3d R_lastKF2Cur_eigen;
	cv::cv2eigen(mono.R_lastKF2Cur, R_lastKF2Cur_eigen);
	Eigen::Vector3d t_lastKF2Cur_imu = lastKF_pose.position - cur_pose.position; // in world coord.
	Eigen::Matrix3d R_w2Cur = R_lastKF2Cur_eigen * cam_pts.R_B2C * lastKF_pose.orientation.toRotationMatrix().transpose();
	Eigen::Vector3d t_Cur = (Eigen::Matrix3d::Identity() - R_lastKF2Cur_eigen) * cam_pts.t_B2C;
	t_lastKF2Cur_imu = R_w2Cur * t_lastKF2Cur_imu + t_Cur;	// in current cam coord.

	// initial scale
	double scale = t_lastKF2Cur_imu.norm();
	scale = 0.1;
	cv::Mat proj_before_opt = lastKF_proj_img.clone();
	projectToImg(lastKF_pts, scale, cam_proj.K, stereo.R, stereo.t, proj_before_opt);
	// std::cout<<"scale before "<<scale<<std::endl;
	if(fabs(scale)>1 ) {
		// std::cout<<"bad scale"<<std::endl;
		scale = 0.1;
	}

	// optimization
	double new_scale = scale;
    if(!optimize(lastKF_features_inlier, lastKF_pts, new_scale, stereo, cam_proj.K, lastKF_pts_img, lastKF_proj_img)) {
    	return false;
    }
    scale = new_scale;

	// final scale
	// std::cout<<"scale after "<<scale<<std::endl;
	if(fabs(scale)>1 ) {
		// std::cout<<"bad scale"<<std::endl;
		scale = 0.1;
	}

    // show projections
	cv::Mat proj_after_opt = lastKF_proj_img.clone();
	projectToImg(lastKF_pts, scale, cam_proj.K, stereo.R, stereo.t, proj_after_opt);
	cv::hconcat(reproj, proj_before_opt, proj_img);
	cv::hconcat(proj_img, proj_after_opt, proj_img);

    PointCloud::Ptr point_cloud (new PointCloud);
    for(int i=0; i<lastKF_pts.size(); i++) {
    	// std::cout<<lastKF_pts[i]<<std::endl;
        pcl::PointXYZ p;
        p.x = scale*lastKF_pts[i].y;
        p.y = scale*lastKF_pts[i].z;
        p.z = scale*lastKF_pts[i].x;
        point_cloud->push_back(p);
    }

    point_cloud->header.frame_id = "/map";
    pcl_pub.publish(point_cloud);

//   	for(double s=0.01; s<0.2; s+=0.01) {
//   		std::cout<<"xxx "<<s<<std::endl;
	// 	cv::Mat proj_try = lastKF.img1.clone();
	// 	projectToImg(lastKF_pts, s, cam_proj.K, stereo.R, stereo.t, proj_try);

	//     cv::imshow("proj try", proj_try);
	//     cv::waitKey();
	// }

	lastKF_fts_pts.clear();
	assert(lastKF_features_inlier.size() == lastKF_pts.size());
	for(int i=0; i<lastKF_features_inlier.size(); i++) {
		lastKF_fts_pts.push_back(FeaturePoints(lastKF_features_inlier[i], scale*lastKF_pts[i]));
	}
	

    // update current state
	Eigen::Vector3d t_lastKF2Cur_eigen;
	cv::cv2eigen(mono.t_lastKF2Cur, t_lastKF2Cur_eigen);
	t_lastKF2Cur_eigen = scale * t_lastKF2Cur_eigen;
	// std::cout<<cur_pose.position<<std::endl;
	cur_pose.position = lastKF_pose.position - R_w2Cur.transpose() * (t_lastKF2Cur_eigen - t_Cur);
	// std::cout<<cur_pose.position<<std::endl;
	// std::cout<<cur_pose.orientation.w()<<" "<<cur_pose.orientation.x()<<" "<<cur_pose.orientation.y()<<" "<<cur_pose.orientation.z()<<std::endl;
	cur_pose.orientation = Eigen::Quaterniond(R_w2Cur.transpose() * cam_pts.R_B2C);
	// std::cout<<cur_pose.orientation.w()<<" "<<cur_pose.orientation.x()<<" "<<cur_pose.orientation.y()<<" "<<cur_pose.orientation.z()<<std::endl;
	return true;
}

bool StereoCamera::optimize(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, double& scale, const StereoModel& stereo, const cv::Mat& orig_K, const cv::Mat& img0, const cv::Mat& img1) 
{
	cv::Mat tmp0 = img0.clone();
	cv::Mat tmp1 = img1.clone();
	std::vector<cv::Mat> img0_pymd;
	std::vector<cv::Mat> img1_pymd;
	std::vector<cv::Mat> K_pymd;
	std::vector<std::vector<cv::Point2f>> fts_pymd;
	double pymd_scale = 1.0;
	for(int i=0; i<PYMD; i++)
	{
		img0_pymd.push_back(tmp0);
		img1_pymd.push_back(tmp1);
		cv::pyrDown(tmp0, tmp0, cv::Size(tmp0.cols/2, tmp0.rows/2));
		cv::pyrDown(tmp1, tmp1, cv::Size(tmp1.cols/2, tmp1.rows/2));
		
		cv::Mat K = orig_K/pymd_scale;
		K.at<double>(2,2) = 1.0;
		K_pymd.push_back(K);

		std::vector<cv::Point2f> fts_tmp;
		for(int i=0; i<fts.size(); i++) {
			fts_tmp.push_back(cv::Point2f(fts[i].x/pymd_scale, fts[i].y/pymd_scale));
		}
		fts_pymd.push_back(fts_tmp);

		pymd_scale *= 2.0;
	}

	cv::Mat X_mat(3, pts.size(), CV_64F);
	for(int i=0; i<pts.size(); i++) {
		X_mat.at<double>(0,i) = pts[i].x;
		X_mat.at<double>(1,i) = pts[i].y;
		X_mat.at<double>(2,i) = pts[i].z;
    }
    X_mat = stereo.R*X_mat;

    std::vector<double> errs;

	for(int i=PYMD-1; i>=0; i--)
	{
		if(!optimize_pymd(fts_pymd[i], pts, X_mat, scale, img0_pymd[i], img1_pymd[i], stereo, K_pymd[i], errs)) return false;
	}

    cv::Mat err_plot;
	cv::Ptr<cv::plot::Plot2d> eplot = cv::plot::Plot2d::create(errs);
	eplot->render(err_plot);
	cv::imshow("err_plot", err_plot);
	cv::waitKey(1);
	return true;

}

bool StereoCamera::optimize_pymd(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, const cv::Mat& X_mat, double& scale, const cv::Mat& img0, const cv::Mat& img1, const StereoModel& stereo, const cv::Mat& K, std::vector<double>& errs) 
{
	// optimize scale inverse for speed
	double scale_inv = 1.0/scale;

	cv::Mat KX = K*X_mat;
	double Ktx = K.at<double>(0,0) * stereo.t.at<double>(0,0);

	std::vector<double> u_base_vec, u_off_vec, v_vec;
	for(int i=0; i<pts.size(); i++) {
		double _u = KX.at<double>(0,i) / KX.at<double>(2,i);
		u_base_vec.push_back(_u);

		double _o = Ktx / KX.at<double>(2,i);
		u_off_vec.push_back(_o);

		double _v = KX.at<double>(1,i) / KX.at<double>(2,i);
		v_vec.push_back(_v);
	}

    Eigen::MatrixXd b_mat, Jac, f_b_vec;
    b_mat.resize(pts.size(), WINDOW_SIZE*WINDOW_SIZE);
    Jac.resize(pts.size(),1);
    f_b_vec.resize(pts.size(),1);

	for(int i=0; i<pts.size(); i++) {
		Eigen::MatrixXd tmp;
		getBatchAround(img0, fts[i].x, fts[i].y, tmp);
		b_mat.block<1, WINDOW_SIZE*WINDOW_SIZE>(i,0) = tmp;
    }

	// int delta = 0;
	// int ddepth = CV_16S;
	// cv::Mat grad_x, grad_y;
	// cv::Sobel( img1, grad_x, ddepth, 1, 0, 3, 1, delta, cv::BORDER_DEFAULT );
	// cv::Sobel( img1, grad_y, ddepth, 0, 1, 3, 1, delta, cv::BORDER_DEFAULT );

	// cv::Mat abs_grad_x;
	// convertScaleAbs( grad_x, abs_grad_x );
	// cv::imshow("x grad", abs_grad_x);

    for(int opt_step=0; opt_step<MAX_OPT_STEP; opt_step++) {
    	for(int i=0; i<pts.size(); i++) {
    		double u = u_base_vec[i] + scale_inv * u_off_vec[i];
    		double v = v_vec[i];

			Eigen::MatrixXd f;
			getBatchAround(img1, u, v, f);

			f_b_vec(i,0) = (b_mat.block<1,WINDOW_SIZE*WINDOW_SIZE>(i,0) - f).sum();

			Eigen::MatrixXd aI_ap(1,2);
			aI_ap << (getPixelValue(img1, u+1.0, v)-getPixelValue(img1, u, v)), 
					 (getPixelValue(img1, u, v+1.0)-getPixelValue(img1, u, v));
		
			Eigen::MatrixXd ap_as(2,1);
			ap_as << u_off_vec[i], 0;

			Eigen::MatrixXd _grad = aI_ap * ap_as;
			Jac(i,0) = _grad(0,0);
	    }
	    Eigen::MatrixXd JTJ = Jac.transpose() * Jac;
	    if(JTJ.determinant() == 0) return false;

	    // std::cout<<Jac<<std::endl;
	    Eigen::MatrixXd _d_scale_inv = (1.0 / ((JTJ + LM_DAMP*Eigen::MatrixXd(JTJ.diagonal().asDiagonal()))(0,0)) ) * (Jac.transpose() * f_b_vec);
	    double d_scale_inv = _d_scale_inv(0,0);

	    if(opt_step > MIN_OPT_STEP && f_b_vec.norm() > errs.back()){
	    	// std::cout<<"steps: "<<opt_step<<std::endl;
	    	return true;
	    } 

	    scale_inv += d_scale_inv;
	    errs.push_back(f_b_vec.norm());
	    
	    if(scale > 1) return false;

	    scale = 1.0 / scale_inv;

	 //    std::cout<<scale<<std::endl;
		// cv::Mat opt = img1.clone();
		// projectToImg(pts, scale, K, stereo.R, stereo.t, opt);
		// cv::namedWindow("opt", cv::WINDOW_NORMAL);
		// cv::imshow("opt", opt);
		// cv::waitKey();
    }
	// std::cout<<"steps: "<<MAX_OPT_STEP<<std::endl;

    return true;
}