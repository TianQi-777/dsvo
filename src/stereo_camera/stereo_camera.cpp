#include "stereo_camera/stereo_camera.hpp"
#include <cmath>

// parameters
int MAX_REPROJ_DIST, KP_BLOCKS, INIT_MAX_KP, MAX_OPT_STEP, PYMD, MIN_FEATURE_DIST;
double NEW_KF_KP_RATIO, MONO_INLIER_THRES, QUAL_LEVEL;
geometry_msgs::PointStamped new_pose_msg, last_pose_msg;

void StereoCamera::updateConfig(direct_stereo::DirectStereoConfig &config, uint32_t level) {
	// std::cout<<"Init param"<<std::endl;
	MAX_REPROJ_DIST = config.MAX_REPROJ_DIST;
	KP_BLOCKS = config.KP_BLOCKS;
	INIT_MAX_KP = config.INIT_MAX_KP;
	QUAL_LEVEL = config.QUAL_LEVEL;
	MIN_FEATURE_DIST = config.MIN_FEATURE_DIST;
	MAX_OPT_STEP = config.MAX_OPT_STEP;
	PYMD = config.PYMD;
	NEW_KF_KP_RATIO = config.NEW_KF_KP_RATIO;
	MONO_INLIER_THRES = config.MONO_INLIER_THRES;
    param_changed = true;
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

	camera0.K = cv::Mat::zeros(3,3,CV_64F);
	camera0.K.at<double>(0,0) = K0[0];
	camera0.K.at<double>(1,1) = K0[1];
	camera0.K.at<double>(0,2) = K0[2];
	camera0.K.at<double>(1,2) = K0[3];
	camera0.K.at<double>(2,2) = 1.0;

	cv::Size cam0_frame_size = cv::Size(frame_size0[0], frame_size0[1]);

	cv::Mat cam1_E = cv::Mat(E1);
	cam1_E = cam1_E.reshape(0,4);
	cv::Mat R1 = cam1_E(cv::Rect(0,0,3,3));
	cv::Mat t1 = cam1_E(cv::Rect(3,0,1,3));

	camera1.K = cv::Mat::zeros(3,3,CV_64F);
	camera1.K.at<double>(0,0) = K1[0];
	camera1.K.at<double>(1,1) = K1[1];
	camera1.K.at<double>(0,2) = K1[2];
	camera1.K.at<double>(1,2) = K1[3];
	camera1.K.at<double>(2,2) = 1.0;

	cv::Size cam1_frame_size = cv::Size(frame_size1[0], frame_size1[1]);


	cv::Mat R1T;
	cv::transpose(R1, R1T);
	StereoModel stereo0;
	stereo0.R = R1T * R0;
	stereo0.t = R1T * (t0 - t1);

    cv::Mat rect_R0, rect_R1, rect_P0, rect_P1, _Q;
    cv::stereoRectify(camera0.K, dist_coeff0, camera1.K, dist_coeff1,cam0_frame_size, stereo0.R, stereo0.t, rect_R0, rect_R1, rect_P0, rect_P1, _Q, cv::CALIB_ZERO_DISPARITY, 0);

    cv::initUndistortRectifyMap(camera0.K, dist_coeff0, rect_R0, rect_P0, cam0_frame_size,CV_32F, camera0.rect_map_x, camera0.rect_map_y);
    cv::initUndistortRectifyMap(camera1.K, dist_coeff1, rect_R1, rect_P1, cam1_frame_size, CV_32F, camera1.rect_map_x, camera1.rect_map_y);

    // modify external matrix by rect rotation
	cv::Mat rR0T, rR1T;
	cv::transpose(rect_R0, rR0T);
	cv::transpose(rect_R1, rR1T);
    R0 = R0 * rR0T;
    R1 = R1 * rR1T;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			camera0.R_C2B(i,j) = R0.at<double>(i,j);
			camera1.R_C2B(i,j) = R1.at<double>(i,j);
		}
		camera0.t_C2B(i) = t0.at<double>(i,0);
		camera1.t_C2B(i) = t1.at<double>(i,0);
	}
	camera0.R_B2C = camera0.R_C2B.transpose();
	camera0.t_B2C = - camera0.R_B2C * camera0.t_C2B;
	camera1.R_B2C = camera1.R_C2B.transpose();
	camera1.t_B2C = - camera1.R_B2C * camera1.t_C2B;

    cv::Mat tmp = rect_P0(cv::Rect(0,0,3,3));
    tmp.copyTo(camera0.K);
    tmp = rect_P1(cv::Rect(0,0,3,3));
    tmp.copyTo(camera1.K);

	stereo0.R = cv::Mat::eye(3,3,CV_64F);
	tmp = rect_P1(cv::Rect(3,0,1,3));
	tmp.copyTo(stereo0.t);
	stereo0.t = stereo0.t / camera0.K.at<double>(0,0);

	StereoModel stereo1;
	stereo1.R = stereo0.R;
	stereo1.t = -stereo0.t;

	camera1.stereo = stereo0;
	camera0.stereo = stereo1;

	// std::cout<<stereo0.t<<std::endl;
	// pcl_pub = nh.advertise<PointCloud> ("points2", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 1000);

	f = boost::bind(&StereoCamera::updateConfig, this, _1, _2);
  	server.setCallback(f);
    
    last_time = -1;
    frame_dropped_count = 0;
    // directSolver_ptr = shared_ptr<DirectSolver>(new DirectSolver());
    param_changed = true;
    init_time = -1;
}

KeyFrame StereoCamera::createKeyFrame(const Pose& cur_pose, double cur_time, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const FeaturePoints& feature_points){
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
		    cv::goodFeaturesToTrack(keyframe.img0(subRegion), subFeatures0, INIT_MAX_KP, QUAL_LEVEL, MIN_FEATURE_DIST);
		    for(int k=0; k<subFeatures0.size(); k++) {
		    	keyframe.features0.push_back(cv::Point2f(subFeatures0[k].x+i*subCols, subFeatures0[k].y+j*subRows));
		    }
    	}
    }

    keyframe.init_feature0_count = keyframe.features0.size();

    keyframe.feature_points = feature_points;

    // cv::Mat feature_img = keyframe.img0.clone();
    // for(int i=0; i<keyframe.features0.size(); i++) {
    // 	cv::circle(feature_img, keyframe.features0[i], 5, cv::Scalar(0,255,0));
    // }
	  //   cv::imshow("sub", feature_img);
	  //   cv::waitKey();

    return keyframe;
}

void StereoCamera::track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, ros::Time cur_time_ros) {
	//stereo rectify
	cv::Mat cur_img0, cur_img1;
    cv::remap(_cur_img0, _cur_img0, camera0.rect_map_x, camera0.rect_map_y, cv::INTER_LINEAR);
    cv::GaussianBlur(_cur_img0, cur_img0, cv::Size(3,3), 0.5);
    cv::remap(_cur_img1, _cur_img1, camera1.rect_map_x, camera1.rect_map_y, cv::INTER_LINEAR);
    cv::GaussianBlur(_cur_img1, cur_img1, cv::Size(3,3), 0.5);

	if(init_time < 0) {
		init_time = cur_time_ros.toSec();
	}
	monoTrack(cur_state, cur_time_ros, cur_img0, cur_img1, camera0, camera1, last_frame0, keyframes0, "Left");
	// monoTrack(cur_state, cur_time, cur_img1, cur_img0, camera1, camera0, last_frame1, keyframes1, "Right");
}

void StereoCamera::monoTrack(State& cur_state, ros::Time cur_time_ros, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1, 
							 Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name) {
	double cur_time = cur_time_ros.toSec();
	cur_state.time = cur_time_ros;
	
	if(last_time < 0 || last_time > cur_time || (cur_time-last_time) > 1.0 || param_changed) {
		cur_state.reset();
		keyframes.clear();
		param_changed = false;
	}

	//initialize key_frame
    if(keyframes.empty() || keyframes.back().features0.size() < 50 ) {
    	if(!keyframes.empty()) frame_dropped_count++;
		
		std::cout<<"KeyFrame("<<keyframes.size()<<") | Dropped("<<frame_dropped_count<<")frames"<<std::endl;
		FeaturePoints empty_fts_pts;
    	KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_time, cur_img0, cur_img1, empty_fts_pts);
	    keyframes.push_back(keyframe);

	    last_frame.img = keyframe.img0.clone(); 
	    last_frame.features = keyframe.features0;
	    last_frame.feature_pnp = keyframe.feature_points.features;

		last_time = cur_time;

    	return;
    }

    KeyFrame& lastKF = keyframes.back();

	std::clock_t start;
	start = std::clock();
	//feature tracking
	FeatureTrackingResult feature_tracking_result;
	featureTrack(lastKF, last_frame, cur_img0, cam0.K, feature_tracking_result);

	// propagate state based on feature(pnp) tracking  
	propagateState(cur_state, cur_time, lastKF, feature_tracking_result.cur_feature_pnp, cam0);
		std::cout << "featureTrack and propagateState: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    // update last_frame
	last_frame.img = cur_img0.clone();
	last_frame.features = feature_tracking_result.cur_features;
    last_frame.feature_pnp = feature_tracking_result.cur_feature_pnp;

	if (feature_tracking_result.track_inliers>MONO_INLIER_THRES)
	{
		// std::cout<<"inlier"<<feature_tracking_result.track_inliers<<std::endl;

		Pose new_cur_pose = cur_state.pose;
		cv::Mat proj_img;
		std::vector<double> errs;
		FeaturePoints curKF_fts_pts;
		std::clock_t start;
		start = std::clock();
		bool re = reconstructAndOptimize(feature_tracking_result, lastKF, cam0, cam1, new_cur_pose, curKF_fts_pts, proj_img, errs, cur_img0, cur_img1);
		std::cout << "reconstructAndOptimize: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
		if(re) {
			std::string proj_window_name = window_name + " projection";
			cv::imshow(proj_window_name, proj_img);
			cv::waitKey(1);

			local_KF_optimizer.optimize(keyframes, 5, cam0, cur_img0, curKF_fts_pts, new_cur_pose);

			// uodate current state
		    cur_state.pose = new_cur_pose;
		    cur_state.velocity = (cur_state.pose.position - lastKF.pose.position) / (cur_time - lastKF.time);
		    cur_state.showPose();
	
	        // construct new keyframe
			KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_time, cur_img0, cur_img1, curKF_fts_pts);
			std::cout<<"KeyFrame("<<keyframes.size()<<") Dropped ("<<frame_dropped_count<<") frames"<<std::endl;
		    keyframes.push_back(keyframe);
		    last_frame.img = keyframe.img0.clone(); 
		    last_frame.features = keyframe.features0;
		    last_frame.feature_pnp = keyframe.feature_points.features;
		}
 	}
	last_time = cur_time;
}

void StereoCamera::featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, const cv::Mat& K, FeatureTrackingResult& feature_tracking_result) {
	cv::Mat line_img, line_img_pnp;
    cv::vconcat(lastKF.img0, cur_img, line_img);
    line_img_pnp = line_img.clone();

	// match points for PNP
    std::vector<cv::Point2f> cur_feature_pnp, last_frame_pnp_features_back;
    if(!last_frame.feature_pnp.empty()) {
	    std::vector<cv::Point2f> lastKF_feature_pnp_inlier, last_frame_feature_pnp_inlier, cur_feature_pnp_inlier;
	    std::vector<cv::Point3f> lastKF_pts_pnp_inlier;
	    std::vector<uchar> statu_pnp, statu_pnp_back;
	    std::vector<float> err_pnp,err_pnp_back;
	    cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.feature_pnp, cur_feature_pnp, statu_pnp, err_pnp);
	    cv::calcOpticalFlowPyrLK(cur_img, last_frame.img, cur_feature_pnp, last_frame_pnp_features_back, statu_pnp_back, err_pnp_back);
	    for(int i=0; i<statu_pnp.size(); i++){
	    	if(!statu_pnp[i] || !statu_pnp_back[i]) continue;

	    	// remove outlier by forward-backward matching
	    	if(cv::norm(last_frame.feature_pnp[i]-last_frame_pnp_features_back[i]) > 1) continue;

	    	lastKF_feature_pnp_inlier.push_back(lastKF.feature_points.features[i]);
	    	lastKF_pts_pnp_inlier.push_back(lastKF.feature_points.points[i]);
	    	last_frame_feature_pnp_inlier.push_back(last_frame.feature_pnp[i]);
	    	cur_feature_pnp_inlier.push_back(cur_feature_pnp[i]);
	    	// cv::line(line_img_pnp, lastKF.feature_points.features[i], cv::Point2f(cur_feature_pnp[i].x+lastKF.img0.cols, cur_feature_pnp[i].y), cv::Scalar(255,0,0));
	    } 
		// cv::imshow("line_img_pnp", line_img_pnp);

	    lastKF.feature_points.features = lastKF_feature_pnp_inlier; 
	    lastKF.feature_points.points = lastKF_pts_pnp_inlier; 
		last_frame.feature_pnp = last_frame_feature_pnp_inlier;
		cur_feature_pnp = cur_feature_pnp_inlier;
	}
    feature_tracking_result.cur_feature_pnp = cur_feature_pnp;



    // filter matches
    std::vector<cv::Point2f> cur_features, last_frame_features_back, lastKF_features_inlier, last_frame_features_inlier, cur_features_inlier;
    std::vector<uchar> status, status_back;
    std::vector<float> err, err_back;
    cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.features, cur_features, status, err);
    cv::calcOpticalFlowPyrLK(cur_img, last_frame.img, cur_features, last_frame_features_back, status_back, err_back);
    for(int i=0; i<status.size(); i++){
    	if(!status[i] || !status_back[i]) continue;

    	// remove outlier by forward-backward matching
    	if(cv::norm(last_frame.features[i]-last_frame_features_back[i]) > 1) continue;

    	lastKF_features_inlier.push_back(lastKF.features0[i]);
    	last_frame_features_inlier.push_back(last_frame.features[i]);
    	cur_features_inlier.push_back(cur_features[i]);
    	// cv::line(line_img, lastKF.features0[i], cv::Point2f(cur_features[i].x, cur_features[i].y+lastKF.img0.rows), cv::Scalar(255,0,0));
    } 
	// cv::imshow("line_img", line_img);

    lastKF.features0 = lastKF_features_inlier; 
	last_frame.features = last_frame_features_inlier;
	cur_features = cur_features_inlier;

	if(lastKF.features0.size() < 10) {
		lastKF.features0.clear();
		return;
	}

    //recover pose
    cv::Mat Est, inlier_mask, R, t;
    Est  = cv::findEssentialMat(lastKF.features0, cur_features, K, cv::RANSAC, 0.999, 1.0, inlier_mask);
    int inlier_count = cv::recoverPose(Est, lastKF.features0, cur_features, K, R, t, inlier_mask);

    // std::cout<<"length: "<<cv::norm(t)<<std::endl;
    // std::cout<<"ratio: "<<inlier_count / float(lastKF.features0.size())<<std::endl;

    //put together
    feature_tracking_result.track_inliers = inlier_count / float(lastKF.features0.size()); 
    feature_tracking_result.lastKF_features = lastKF.features0;
    feature_tracking_result.cur_features = cur_features;
    feature_tracking_result.inlier_mask = inlier_mask;
	feature_tracking_result.R_lastKF2Cur = R;
	feature_tracking_result.t_lastKF2Cur = t;
}

void StereoCamera::propagateState(State& cur_state, double cur_time, const KeyFrame& lastKF, const std::vector<cv::Point2f>& cur_features, const CameraModel& cam) {
	// std::cout<<lastKF.feature_points.points.size()<<" = "<<cur_features.size()<<std::endl;
	assert(lastKF.feature_points.points.size() == cur_features.size());
	bool brutePropagate = true;
	if(lastKF.feature_points.points.size() > 10) {
	    cv::Mat R, rvec, t, inlier_mask;
	    cv::solvePnPRansac( lastKF.feature_points.points, cur_features, cam.K, cv::Mat(), rvec, t, false, 100, 3.0, 0.99, inlier_mask);
	    float inlier_ratio = float(inlier_mask.rows) / float(cur_features.size());
	    Rodrigues(rvec, R);
	    // std::cout<<R<<std::endl<<t<<std::endl;
	    // std::cout<<inlier_ratio<<std::endl;

		Eigen::Matrix3d R_lastKF2Cur;
		Eigen::Vector3d t_lastKF2Cur;
		cv::cv2eigen(R, R_lastKF2Cur);
		cv::cv2eigen(t, t_lastKF2Cur);
		Eigen::Matrix3d R_w2Cur = R_lastKF2Cur * cam.R_B2C * lastKF.pose.orientation.toRotationMatrix().transpose();
		Eigen::Vector3d t_Cur = (Eigen::Matrix3d::Identity() - R_lastKF2Cur) * cam.t_B2C;
		Eigen::Vector3d t_world = R_w2Cur.transpose() * (t_lastKF2Cur - t_Cur);

		if(inlier_ratio > 0.7 && inlier_ratio < 1.0 && t_world.norm() < 0.5) {
			// std::cout<<"PnP Propagate"<<std::endl;
			cur_state.pose.position = lastKF.pose.position - t_world;
			cur_state.pose.orientation = Eigen::Quaterniond(R_w2Cur.transpose() * cam.R_B2C);
		    cur_state.velocity = (cur_state.pose.position - lastKF.pose.position) / (cur_time - lastKF.time);

			brutePropagate = false;
		}

	} 
	if(brutePropagate) {
		cur_state.pose.position += cur_state.velocity * (cur_time - last_time);
	}

	// cur_state.showPose();
	// cv::waitKey();
}

bool StereoCamera::reconstructAndOptimize(FeatureTrackingResult feature_result, const KeyFrame& lastKF, 
										  const CameraModel& cam0, const CameraModel& cam1,
										  Pose& cur_pose, FeaturePoints& curKF_fts_pts, cv::Mat& proj_img, std::vector<double>& errs, 
										  const cv::Mat& cur_img0, const cv::Mat& cur_img1)
{
	std::clock_t start;
	assert(feature_result.lastKF_features.size() == feature_result.cur_features.size());
	// remove outlier by pose estimation
	std::vector<cv::Point2f> lastKF_features_inlier, cur_features_inlier;
    for(unsigned int i=0; i<feature_result.cur_features.size(); i++) {
        if(feature_result.inlier_mask.at<uchar>(i,0)>0) {
	    	lastKF_features_inlier.push_back(feature_result.lastKF_features[i]);
	    	cur_features_inlier.push_back(feature_result.cur_features[i]);
        }
    }

    //reconstruct 3D pts and bundle adjustment
	cv::Mat reproj = lastKF.img0.clone();
	std::vector<cv::Point3d> lastKF_pts;
	start = std::clock();
	reconstructor.reconstructAndBundleAdjust(lastKF_features_inlier, cur_features_inlier, cam0.K, feature_result.R_lastKF2Cur, feature_result.t_lastKF2Cur, MAX_REPROJ_DIST, lastKF_pts, reproj);
	std::cout << "reconstructAndBundleAdjust: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

	// current scale
	Eigen::Matrix3d R_lastKF2Cur_eigen;
	cv::cv2eigen(feature_result.R_lastKF2Cur, R_lastKF2Cur_eigen);
	Eigen::Vector3d t_lastKF2Cur = lastKF.pose.position - cur_pose.position; // in world coord.
	Eigen::Matrix3d R_w2Cur = R_lastKF2Cur_eigen * cam0.R_B2C * lastKF.pose.orientation.toRotationMatrix().transpose();
	Eigen::Vector3d t_Cur = (Eigen::Matrix3d::Identity() - R_lastKF2Cur_eigen) * cam0.t_B2C;
	t_lastKF2Cur = R_w2Cur * t_lastKF2Cur + t_Cur;	// in current cam coord.

	// initial scale
	double scale = t_lastKF2Cur.norm();
	// std::cout<<scale<<std::endl;
	// scale = 0.1;
	// cv::Mat proj_before_opt = lastKF.img1.clone();
	// project3DPtsToImg(lastKF_pts, scale, cam1, proj_before_opt);
    // cv::imshow("proj_before_opt", proj_before_opt);
	// std::cout<<"scale before "<<scale<<std::endl;
	if(fabs(scale)<0.01 || fabs(scale)>1 ) {
		// std::cout<<"bad scale before: "<<scale<<std::endl;
		scale = 0.1;
	}


	// optimization
	double new_scale = scale;
	start = std::clock();
    if(!scale_optimizer.optimize(lastKF_features_inlier, lastKF_pts, new_scale, cam1, lastKF.img0, lastKF.img1, PYMD, MAX_OPT_STEP)) {
    	return false;
    }
	std::cout << "scale optimize: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
    scale = new_scale;

	// final scale
	// std::cout<<"scale after "<<scale<<std::endl;
	// if(fabs(scale)>1 ) {
	// 	std::cout<<"bad scale after: "<<scale<<std::endl;
	// 	scale = 0.1;
	// }

    // PointCloud::Ptr point_cloud (new PointCloud);
    // for(int i=0; i<lastKF_pts.size(); i++) {
    // 	// std::cout<<lastKF_pts[i]<<std::endl;
    //     pcl::PointXYZ p;
    //     p.x = scale*lastKF_pts[i].y;
    //     p.y = scale*lastKF_pts[i].z;
    //     p.z = scale*lastKF_pts[i].x;
    //     point_cloud->push_back(p);
    // }

    // point_cloud->header.frame_id = "/map";
    // pcl_pub.publish(point_cloud);

//   	for(double s=0.01; s<0.2; s+=0.01) {
//   		std::cout<<"xxx "<<s<<std::endl;
	// 	cv::Mat proj_try = lastKF.img1.clone();
	// 	project3DPtsToImg(lastKF_pts, s, cam1.K, cam0.stereo.R, cam0.stereo.t, proj_try);

	//     cv::imshow("proj try", proj_try);
	//     cv::waitKey();
	// }

	curKF_fts_pts.features.clear();
	curKF_fts_pts.points.clear();
	assert(cur_features_inlier.size() == lastKF_pts.size());
	for(int i=0; i<cur_features_inlier.size(); i++) {
		curKF_fts_pts.features.push_back(cur_features_inlier[i]);
		cv::Point3d p = scale*lastKF_pts[i];
		cv::Mat p_cur = (feature_result.R_lastKF2Cur *(cv::Mat_<double>(3,1)<<p.x, p.y, p.z)) + scale*feature_result.t_lastKF2Cur;
		curKF_fts_pts.points.push_back(cv::Point3f(p_cur.at<double>(0,0), p_cur.at<double>(1,0), p_cur.at<double>(2,0)));
	}
	
	std::vector<cv::Point2f> proj_pts0, proj_pts1;
	cv::Mat stereoRVec;
	cv::Rodrigues(cam1.stereo.R, stereoRVec);
    cv::projectPoints(curKF_fts_pts.points, cv::Mat::zeros(3,1,CV_64F), cv::Mat::zeros(3,1,CV_64F), cam0.K, cv::Mat::zeros(1,4,CV_64F), proj_pts0);
    cv::projectPoints(curKF_fts_pts.points, stereoRVec, cam1.stereo.t, cam1.K, cv::Mat::zeros(1,4,CV_64F), proj_pts1);
    cv::Mat proj_img_cur0 = cur_img0.clone();
    cv::Mat proj_img_cur1 = cur_img1.clone();
    cv::cvtColor(proj_img_cur0, proj_img_cur0, cv::COLOR_GRAY2BGR);
    cv::cvtColor(proj_img_cur1, proj_img_cur1, cv::COLOR_GRAY2BGR);
    for(auto& p:proj_pts0) {
    	if(p.x>=0 && p.x<=proj_img_cur0.cols && p.y>=0 && p.y<=proj_img_cur0.rows)
	        cv::drawMarker(proj_img_cur0, p, cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
    }
    for(auto& p:proj_pts1) {
    	if(p.x>=0 && p.x<=proj_img_cur1.cols && p.y>=0 && p.y<=proj_img_cur1.rows)
	        cv::drawMarker(proj_img_cur1, p, cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
    }

    // show projections
    // cv::waitKey();
	cv::Mat proj_after_opt = lastKF.img1.clone();
	project3DPtsToImg(lastKF_pts, scale, cam1, proj_after_opt);
	cv::Mat proj_img_cur, proj_img_KF;
	cv::hconcat(proj_img_cur0, proj_img_cur1, proj_img_cur);
	cv::hconcat(reproj, proj_after_opt, proj_img_KF);
	cv::vconcat(proj_img_cur, proj_img_KF, proj_img);

    // update current state
	Eigen::Vector3d t_lastKF2Cur_eigen;
	cv::cv2eigen(feature_result.t_lastKF2Cur, t_lastKF2Cur_eigen);
	t_lastKF2Cur_eigen = scale * t_lastKF2Cur_eigen;
	// std::cout<<cur_pose.position<<std::endl;
	cur_pose.position = lastKF.pose.position - R_w2Cur.transpose() * (t_lastKF2Cur_eigen - t_Cur);
	// std::cout<<cur_pose.position<<std::endl;
	// std::cout<<cur_pose.orientation.w()<<" "<<cur_pose.orientation.x()<<" "<<cur_pose.orientation.y()<<" "<<cur_pose.orientation.z()<<std::endl;
	cur_pose.orientation = Eigen::Quaterniond(R_w2Cur.transpose() * cam0.R_B2C);
	// std::cout<<cur_pose.orientation.w()<<" "<<cur_pose.orientation.x()<<" "<<cur_pose.orientation.y()<<" "<<cur_pose.orientation.z()<<std::endl;
	return true;
}

