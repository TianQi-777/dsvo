#include "stereo_camera/stereo_camera.hpp"
#include <cmath>

// parameters
int MAX_REPROJ_DIST, KP_BLOCKS, INIT_MAX_KP, BA_MAX_STEP, SCALE_MAX_STEP, SCALE_PYMD, MIN_FEATURE_DIST, PROP_POSE_PYMD, PROP_POSE_ITER, BLUR_SZ;
double MONO_INLIER_THRES, QUAL_LEVEL, BLUR_VAR;
bool LOOP_CLOSURE, TIME_DEBUG, TIME_LOG;

void StereoCamera::updateConfig(direct_stereo::DirectStereoConfig &config, uint32_t level) {
	// std::cout<<"Init param"<<std::endl;
	MAX_REPROJ_DIST = config.MAX_REPROJ_DIST;
	PROP_POSE_PYMD = config.PROP_POSE_PYMD;
	PROP_POSE_ITER = config.PROP_POSE_ITER;
	KP_BLOCKS = config.KP_BLOCKS;
	INIT_MAX_KP = config.INIT_MAX_KP;
	QUAL_LEVEL = config.QUAL_LEVEL;
	BLUR_SZ = 2*config.BLUR_SZ+1;
	BLUR_VAR = config.BLUR_VAR;
	MIN_FEATURE_DIST = config.MIN_FEATURE_DIST;
	BA_MAX_STEP = config.BA_MAX_STEP;
	SCALE_MAX_STEP = config.SCALE_MAX_STEP;
	SCALE_PYMD = config.SCALE_PYMD;
	MONO_INLIER_THRES = config.MONO_INLIER_THRES;
	LOOP_CLOSURE = config.LOOP_CLOSURE;
	TIME_DEBUG = config.TIME_DEBUG;
	TIME_LOG = config.TIME_LOG;
	if(TIME_LOG) {
		if(time_ofs.is_open()) {
			time_ofs.close();
		}
		time_ofs.open("time.txt", std::ofstream::out | std::ofstream::trunc);
	}
    param_changed = true;
}

StereoCamera::StereoCamera(const std::vector<double>& E0, 
						   const std::vector<double>& K0, 
						   const std::vector<double>& frame_size0, 
						   const std::vector<double>& dist_coeff0,
						   const std::vector<double>& E1, 
						   const std::vector<double>& K1, 
						   const std::vector<double>& frame_size1, 
						   const std::vector<double>& dist_coeff1,
						   const string& gt_type) {
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

    cv::Mat rect_R0, rect_R1, rect_P0, rect_P1;
    cv::stereoRectify(camera0.K, dist_coeff0, camera1.K, dist_coeff1,cam0_frame_size, stereo0.R, stereo0.t, rect_R0, rect_R1, rect_P0, rect_P1, cam0_Q, cv::CALIB_ZERO_DISPARITY, 0, cam0_frame_size);

    // cv::initUndistortRectifyMap(camera0.K, dist_coeff0, rect_R0, rect_P0, cv::Size(640,480),CV_32F, camera0.rect_map_x, camera0.rect_map_y);
    // cv::initUndistortRectifyMap(camera1.K, dist_coeff1, rect_R1, rect_P1, cv::Size(640,480), CV_32F, camera1.rect_map_x, camera1.rect_map_y);

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
	direct_pcl_pub = nh.advertise<PointCloud> ("direct_points", 1);
	stereo_pcl_pub = nh.advertise<PointCloud> ("stereo_points", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 1000);

	// comparer type
	if(gt_type == "odom")
	{
		comparer = new OdomComparer();
	} else if(gt_type == "point") {
		comparer = new PointComparer();
	} else {
		comparer = new TransComparer();
	}

	f = boost::bind(&StereoCamera::updateConfig, this, _1, _2);
  	server.setCallback(f);
    
    last_time = -1;
    frame_dropped_count = 0;
    // directSolver_ptr = shared_ptr<DirectSolver>(new DirectSolver());
    param_changed = true;
}

void StereoCamera::testStereoMatch(const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0) {
	cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0,16,3);
	// cv::Ptr<cv::DisparityWLSFilter> wls_filter = cv::createDisparityWLSFilter(left_matcher);
	// cv::Ptr<cv::StereoMatcher> right_matcher = cv::createRightMatcher(left_matcher);

	cv::Mat left_disp, left_disp_8u;
	left_matcher->compute(cur_img0, cur_img1,left_disp);

	double minVal; double maxVal;
	cv::minMaxLoc( left_disp, &minVal, &maxVal );
	left_disp.convertTo( left_disp_8u, CV_8UC1, 255/(maxVal - minVal));

	cv::Mat points;
	cv::reprojectImageTo3D(left_disp, points, cam0_Q);

	PointCloud::Ptr point_cloud (new PointCloud);
	for(int i=0; i<points.rows; i++) {
		for(int j=0; j<points.cols; j++) {
			// std::cout<<lastKF_pts[i]<<std::endl;
		    pcl::PointXYZ p;
		    p.x = points.at<cv::Vec3f>(i,j)[0];
		    p.y = points.at<cv::Vec3f>(i,j)[1];
		    p.z = points.at<cv::Vec3f>(i,j)[2];
		    point_cloud->push_back(p);
		}
	}

    point_cloud->header.frame_id = "/map";
    stereo_pcl_pub.publish(point_cloud);
	// right_matcher->compute(cur_img1,cur_img0, right_disp);

	// cv::Mat raw_disp_vis;
    // cv::getDisparityVis(left_disp,raw_disp_vis,1.0);
    // cv::namedWindow("raw disparity", cv::WINDOW_AUTOSIZE);
    // cv::imshow("raw disparity", left_disp);

}

KeyFrame StereoCamera::createKeyFrame(const Pose& cur_pose, double cur_time, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const FeaturePoints& feature_points){
	KeyFrame keyframe = KeyFrame();

	keyframe.pose = cur_pose;
	keyframe.time = cur_time;
	keyframe.img0 = cur_img0.clone();
	keyframe.img1 = cur_img1.clone();
    keyframe.feature_points = feature_points;

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
    // // put previous features into new KF for consistency
    // for(int i=0; i<feature_points.features.size(); i++) {
    // 	keyframe.features0.push_back(feature_points.features[i]);
    // }

    // cv::Mat feature_img = keyframe.img0.clone();
    // for(int i=0; i<keyframe.features0.size(); i++) {
    // 	cv::circle(feature_img, keyframe.features0[i], 5, cv::Scalar(0,255,0));
    // }
    // cv::imshow("features", feature_img);
    // cv::waitKey(1);

    return keyframe;
}

void StereoCamera::track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, ros::Time cur_time_ros) {
	//stereo rectify
	cv::Mat cur_img0, cur_img1;
    cv::remap(_cur_img0, cur_img0, camera0.rect_map_x, camera0.rect_map_y, cv::INTER_LINEAR);
    cv::GaussianBlur(cur_img0, cur_img0, cv::Size(BLUR_SZ,BLUR_SZ), BLUR_VAR);
    cv::remap(_cur_img1, cur_img1, camera1.rect_map_x, camera1.rect_map_y, cv::INTER_LINEAR);
    cv::GaussianBlur(cur_img1, cur_img1, cv::Size(BLUR_SZ,BLUR_SZ), BLUR_VAR);

	monoTrack(cur_state, cur_time_ros, cur_img0, cur_img1, camera0, camera1, last_frame0, keyframes0, "Left");
	// std::clock_t start;
	// start = std::clock();
	// testStereoMatch(cur_img0, cur_img1, camera0);
	// std::cout << "StereoMatch: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
	// monoTrack(cur_state, cur_time, cur_img1, cur_img0, camera1, camera0, last_frame1, keyframes1, "Right");
}

void StereoCamera::monoTrack(State& cur_state, ros::Time cur_time_ros, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1, 
							 Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name) {
	double cur_time = cur_time_ros.toSec();
	cur_state.time = cur_time_ros;

	std::clock_t f_start;
	f_start = std::clock();

	if(last_time < 0 || last_time > cur_time || (cur_time-last_time) > 1.0 || param_changed) {
		std::cout<<"Reset"<<std::endl;
		cur_state.reset();
		keyframes.clear();
		frame_dropped_count = 0;
		param_changed = false;
	}

	//initialize key_frame
    if(keyframes.empty() || keyframes.back().features0.size() < 50 ) {
    	if(!keyframes.empty()) frame_dropped_count++;
		
		// std::cout<<"KeyFrame("<<keyframes.size()<<") | Dropped("<<frame_dropped_count<<")frames"<<std::endl;
		FeaturePoints empty_fts_pts;
    	KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_time, cur_img0, cur_img1, empty_fts_pts);
	    keyframes.push_back(keyframe);

	    last_frame.img = keyframe.img0.clone(); 
	    last_frame.features = keyframe.features0;
	    last_frame.feature_points = keyframe.feature_points;

		last_time = cur_time;

    	return;
    }

	// std::cout<<keyframes.back().features0.size()<<std::endl;	//TODO
    KeyFrame& lastKF = keyframes.back();

	// std::clock_t start;
	// start = std::clock();
	//feature tracking
	FeatureTrackingResult feature_tracking_result;
	featureTrack(lastKF, last_frame, cur_img0, feature_tracking_result);
	last_frame.features = feature_tracking_result.cur_features;
	// std::cout << "featureTrack: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

	// start = std::clock();
	propagateState(cur_state, cur_img0, cur_time, last_frame, cam0);
	// std::cout << "propagateState: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    // update last_frame
	last_frame.img = cur_img0.clone();

	Eigen::Vector3d disp = cur_state.pose.position - lastKF.pose.position;
	if (last_frame.feature_points.points.size() < 50 || last_frame.feature_points.points.size() < lastKF.feature_points.points.size() * 0.5 || disp.norm() > 0.5)
	{
	    //recover pose
	    cv::Mat Est, inlier_mask, R, t;

		std::clock_t start;
		start = std::clock();
	    Est  = cv::findEssentialMat(lastKF.features0, feature_tracking_result.cur_features, cam0.K, cv::RANSAC, 0.99, 1.0, inlier_mask);
	    int inlier_count = cv::recoverPose(Est, lastKF.features0, feature_tracking_result.cur_features, cam0.K, R, t, inlier_mask);
		if(TIME_DEBUG) {
			std::cout << "Essential & Pose: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
		}

		if (inlier_count > lastKF.features0.size() * MONO_INLIER_THRES) {
		    feature_tracking_result.inlier_mask = inlier_mask;
			feature_tracking_result.R_lastKF2Cur = R;
			feature_tracking_result.t_lastKF2Cur = t;
			// std::cout<<"inlier"<<feature_tracking_result.track_inliers<<std::endl;

			Pose new_cur_pose = cur_state.pose;
			cv::Mat proj_img;
			std::vector<double> errs;
			FeaturePoints curKF_fts_pts;
			// std::clock_t start;
			// start = std::clock();
			bool re = reconstructAndOptimize(feature_tracking_result, lastKF, cam0, cam1, new_cur_pose, curKF_fts_pts, proj_img, errs, cur_img0, cur_img1);
			// std::cout << "reconstructAndOptimize: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
			if(re) {
				std::string proj_window_name = window_name + " projection";
				// cv::namedWindow(proj_window_name, cv::WINDOW_NORMAL);
				cv::imshow(proj_window_name, proj_img);
				cv::waitKey(1);
		        // construct new keyframe
				KeyFrame keyframe = createKeyFrame(new_cur_pose, cur_time, cur_img0, cur_img1, curKF_fts_pts);
			    keyframes.push_back(keyframe);
				std::cout<<"KeyFrame("<<keyframes.size()<<") Dropped ("<<frame_dropped_count<<") frames"<<std::endl;

				PointCloud::Ptr point_cloud (new PointCloud);
				for(int i=0; i<curKF_fts_pts.points.size(); i++) {
					// std::cout<<lastKF_pts[i]<<std::endl;
				    pcl::PointXYZ p;
				    p.x = curKF_fts_pts.points[i].y;
				    p.y = curKF_fts_pts.points[i].z;
				    p.z = curKF_fts_pts.points[i].x;
				    point_cloud->push_back(p);
				}

			    point_cloud->header.frame_id = "/map";
			    direct_pcl_pub.publish(point_cloud);

				if (LOOP_CLOSURE) {
					// std::clock_t start;
					// start = std::clock();
					local_KF_optimizer.optimize(keyframes, 5, cam0);
					// std::cout << "local_KF_optimizer: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
				}

				// update last frame
			    last_frame.img = keyframe.img0.clone(); 
			    last_frame.features = keyframe.features0;
			    last_frame.feature_points = keyframe.feature_points;

				// update current state
			    cur_state.pose = new_cur_pose;
			    cur_state.velocity = (cur_state.pose.position - lastKF.pose.position) / (cur_time - lastKF.time);

				if(TIME_DEBUG) {
					std::cout << "Keyframe: " << (std::clock() - f_start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
				}
				if(TIME_LOG) {
					time_ofs << "1 " << (std::clock() - f_start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
				}
			}
		}
	} else {

		if(TIME_DEBUG) {
			std::cout << "Normal frame: " << (std::clock() - f_start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
		}

		if(TIME_LOG) {
			time_ofs << "0 " << (std::clock() - f_start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
		}
	}
    comparer->write_vo(cur_state.pose, cur_time);
    cur_state.showPose();
	last_time = cur_time;
}

void StereoCamera::featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, FeatureTrackingResult& feature_tracking_result) {
    std::vector<cv::Point2f> cur_features, last_frame_features_back, lastKF_features_inlier, last_frame_features_inlier, cur_features_inlier;
    std::vector<uchar> status, status_back;
    std::vector<float> err, err_back;

	std::clock_t start;
	start = std::clock();
    // cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.features, cur_features, status, err);
    // cv::calcOpticalFlowPyrLK(cur_img, last_frame.img, cur_features, last_frame_features_back, status_back, err_back);
    cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.features, cur_features, status, err, cv::Size(11,11), 0);
    cv::calcOpticalFlowPyrLK(cur_img, last_frame.img, cur_features, last_frame_features_back, status_back, err_back, cv::Size(11,11), 0);
	if(TIME_DEBUG) {
		std::cout << "calcOpticalFlowPyrLK(2x): " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
	}
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

    // std::cout<<"length: "<<cv::norm(t)<<std::endl;
    // std::cout<<"ratio: "<<inlier_count / float(lastKF.features0.size())<<std::endl;

    //put together
    feature_tracking_result.lastKF_features = lastKF.features0;
    feature_tracking_result.cur_features = cur_features;
}

void StereoCamera::propagateState(State& cur_state, const cv::Mat& cur_img, double cur_time, Frame& last_frame, const CameraModel& cam) {
	bool brutePropagate = true;
	if(last_frame.feature_points.points.size() > 10) {
		Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
		Eigen::Vector3d t = Eigen::Vector3d::Zero();

		std::clock_t start;
		start = std::clock();
		double dist = pose_estimater.poseEstimate(last_frame.feature_points, last_frame.img, cam.K, cur_img, PROP_POSE_PYMD, PROP_POSE_ITER, R, t);

		if(true) {
			// std::cout<<"dist "<<dist<<std::endl;
			if(TIME_DEBUG) {
				std::cout << "propagateState:poseEstimate: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
			}

			start = std::clock();

			cv::Mat r_cv, R_cv, t_cv;
			cv::eigen2cv(R, R_cv);
			cv::eigen2cv(t, t_cv);
			cv::Rodrigues(R_cv, r_cv);
			std::vector<cv::Point2f> new_fts;
		    cv::projectPoints(last_frame.feature_points.points, r_cv, t_cv, cam.K, cv::Mat::zeros(1,4,CV_64F), new_fts);

		    std::vector<uchar> status;
		    std::vector<float> err;
		    // std::cout<<"before "<<new_fts[10]<<std::endl;
		    cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.feature_points.features, new_fts, status, err, cv::Size(11,11), 0);
		    // std::cout<<"after "<<new_fts[10]<<std::endl; 
		    cv::Mat inlier_mask;
		    // std::cout<<r_cv<<std::endl;
		    // std::cout<<t_cv<<std::endl<<std::endl;
		    cv::solvePnPRansac( last_frame.feature_points.points, new_fts, cam.K, cv::Mat(), r_cv, t_cv, false, 100, 1.0, 0.99, inlier_mask);
		    float inlier_ratio = float(inlier_mask.rows) / float(new_fts.size());
		    // std::cout<<"solvePnPRansac inlier_ratio "<<inlier_ratio<<std::endl;
		    // std::cout<<r_cv<<std::endl;
			cv::Rodrigues(r_cv, R_cv);
		    cv::cv2eigen(R_cv, R);
		    cv::cv2eigen(t_cv, t);

		    // update last_frame
		    std::vector<cv::Point3f> new_pts_inlier;
			std::vector<cv::Point2f> new_fts_inlier;

			// for(int i=0; i<last_frame.feature_points.points.size(); i++) {
			// 	// std::cout<<inlier_mask.at<int>(0,i)<<std::endl;
			// 	cv::Point3f& p = last_frame.feature_points.points[i];
			// 	cv::Mat new_p_m = R_cv * (cv::Mat_<double>(3,1) << p.x, p.y, p.z) + t_cv;
			// 	new_pts_inlier.push_back(cv::Point3f(new_p_m));

			// 	new_fts_inlier.push_back(new_fts[i]);
			// }

			for(int i=0; i<inlier_mask.rows; i++) {
				// std::cout<<inlier_mask.at<int>(0,i)<<std::endl;
				cv::Point3f& p = last_frame.feature_points.points[inlier_mask.at<int>(0,i)];
				cv::Mat new_p_m = R_cv * (cv::Mat_<double>(3,1) << p.x, p.y, p.z) + t_cv;
				new_pts_inlier.push_back(cv::Point3f(new_p_m));

				new_fts_inlier.push_back(new_fts[inlier_mask.at<int>(0,i)]);
			}

			last_frame.feature_points.points = new_pts_inlier;
			last_frame.feature_points.features = new_fts_inlier;

			cv::Mat proj_img = cur_img.clone();
		    cv::cvtColor(proj_img, proj_img, cv::COLOR_GRAY2BGR);
		    int marker_size = proj_img.rows / 50;
		    for(int i=0; i<new_fts_inlier.size(); i++) {
	        	double u = new_fts_inlier[i].x;
	        	double v = new_fts_inlier[i].y;

				if( 0<=u && u<proj_img.cols && 0<=v && v<proj_img.rows) {
			        cv::drawMarker(proj_img, new_fts_inlier[i], cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
				}

			}

			if(TIME_DEBUG) {
				std::cout << "propagateState:rest: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
			}

			cv::imshow("Propagate projection", proj_img);
			cv::waitKey(1);
// std::cout<<"position before "<<cur_state.pose.position<<std::endl;
// std::cout<<"orientation before "<<cur_state.pose.orientation.x()<<" "<<cur_state.pose.orientation.y()<<" "<<cur_state.pose.orientation.z()<<" "<<cur_state.pose.orientation.w()<<" "<<std::endl;
			// std::cout<<"Direct Propagate"<<std::endl;
			Eigen::Matrix3d _R = cur_state.pose.orientation.toRotationMatrix() * cam.R_C2B * R.transpose();
			Eigen::Matrix3d R_w = _R * cam.R_B2C;
			Eigen::Vector3d t_w = _R*(cam.t_B2C-t) + cur_state.pose.orientation.toRotationMatrix()*cam.t_C2B + cur_state.pose.position;
		    cur_state.velocity = (t_w - cur_state.pose.position) / (cur_time - last_time);
			cur_state.pose.position = t_w;
			cur_state.pose.orientation = Eigen::Quaterniond(R_w);

// std::cout<<"position after "<<cur_state.pose.position<<std::endl;
// std::cout<<"orientation after "<<cur_state.pose.orientation.x()<<" "<<cur_state.pose.orientation.y()<<" "<<cur_state.pose.orientation.z()<<" "<<cur_state.pose.orientation.w()<<" "<<std::endl;
			brutePropagate = false;
		}

	} 

	if(brutePropagate) {
		std::cout<<"Brute Propagate"<<std::endl;
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
	assert(feature_result.lastKF_features.size() == feature_result.cur_features.size());
	// remove outlier by pose estimation
	std::vector<cv::Point2f> lastKF_features_inlier, cur_features_inlier;
    for(unsigned int i=0; i<feature_result.cur_features.size(); i++) {
        if(feature_result.inlier_mask.at<uchar>(i,0)>0) 
        {
	    	lastKF_features_inlier.push_back(feature_result.lastKF_features[i]);
	    	cur_features_inlier.push_back(feature_result.cur_features[i]);
        }
    }

    //reconstruct 3D pts and bundle adjustment
	cv::Mat reproj = lastKF.img0.clone();
	std::vector<PointWithUncertainty> lastKF_pts;
	
	std::clock_t start;
	start = std::clock();
	reconstructor.reconstructAndBundleAdjust(lastKF_features_inlier, cur_features_inlier, cam0.K, feature_result.R_lastKF2Cur, feature_result.t_lastKF2Cur, BA_MAX_STEP, MAX_REPROJ_DIST, lastKF_pts, reproj);
	if(TIME_DEBUG) {
		std::cout << "reconstructAndBundleAdjust: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
	}

	// current scale
	Eigen::Matrix3d R_lastKF2Cur_eigen;
	cv::cv2eigen(feature_result.R_lastKF2Cur, R_lastKF2Cur_eigen);
	Eigen::Vector3d t_lastKF2Cur = lastKF.pose.position - cur_pose.position; // in world coord.
	Eigen::Matrix3d R_w2Cur = R_lastKF2Cur_eigen * cam0.R_B2C * lastKF.pose.orientation.toRotationMatrix().transpose();
	Eigen::Vector3d t_Cur = (Eigen::Matrix3d::Identity() - R_lastKF2Cur_eigen) * cam0.t_B2C;
	t_lastKF2Cur = R_w2Cur * t_lastKF2Cur + t_Cur;	// in current cam coord.

	// initial scale
	double scale = t_lastKF2Cur.norm();
	// std::cout<<"scale before "<<scale<<std::endl;
	if(fabs(scale)<0.01 || fabs(scale)>1 ) {
		std::cout<<"bad scale before: "<<scale<<std::endl;
		scale = 0.1;
	}


	// optimization
	double new_scale = scale;

	start = std::clock();
    if(!scale_optimizer.optimize(lastKF_features_inlier, lastKF_pts, new_scale, cam1, lastKF.img0, lastKF.img1, SCALE_PYMD, SCALE_MAX_STEP)) {
    	return false;
    }
	if(TIME_DEBUG) {
		std::cout << "scale optimize: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
	}

	if(TIME_LOG) {
		assert(lastKF_features_inlier.size() > 1);
		time_ofs << lastKF_features_inlier.size() << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
	}

    scale = new_scale;

	if(fabs(scale)<0.01 || fabs(scale)>1 ) {
		std::cout<<"bad scale bafter: "<<scale<<std::endl;
		scale = 0.1;
		return false;
	}

	curKF_fts_pts.features.clear();
	curKF_fts_pts.points.clear();
	curKF_fts_pts.uncertainties.clear();
	assert(cur_features_inlier.size() == lastKF_pts.size());
	std::vector<cv::Point3d> lastKF_pts_points;
	for(int i=0; i<cur_features_inlier.size(); i++) {
		curKF_fts_pts.features.push_back(cur_features_inlier[i]);
		cv::Point3d p = scale*lastKF_pts[i].point;
		cv::Mat p_cur = (feature_result.R_lastKF2Cur *(cv::Mat_<double>(3,1)<<p.x, p.y, p.z)) + scale*feature_result.t_lastKF2Cur;
		curKF_fts_pts.points.push_back(cv::Point3f(p_cur.at<double>(0,0), p_cur.at<double>(1,0), p_cur.at<double>(2,0)));
		lastKF_pts_points.push_back(p);
		curKF_fts_pts.uncertainties.push_back(lastKF_pts[i].uncertainty);
	}

	cv::Mat proj_after_opt = lastKF.img1.clone();
	project3DPtsToImg(lastKF_pts_points, 1.0, cam1, proj_after_opt);
	cv::hconcat(reproj, proj_after_opt, proj_img);

 //    // update current state
	// Eigen::Vector3d t_lastKF2Cur_eigen;
	// cv::cv2eigen(feature_result.t_lastKF2Cur, t_lastKF2Cur_eigen);
	// t_lastKF2Cur_eigen = scale * t_lastKF2Cur_eigen;
	// cur_pose.position = lastKF.pose.position - R_w2Cur.transpose() * (t_lastKF2Cur_eigen - t_Cur);
	// cur_pose.orientation = Eigen::Quaterniond(R_w2Cur.transpose() * cam0.R_B2C);

	return true;
}

