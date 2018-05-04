#include "stereo_camera/stereo_camera.hpp"
#include <cmath>

void StereoCamera::track(State& cur_state, const cv::Mat& _cur_img0, const cv::Mat& _cur_img1, double _cur_time) {
	std::clock_t frame_start = std::clock();
	//stereo rectify
	cv::Mat cur_img0, cur_img1;
  cv::remap(_cur_img0, cur_img0, camera0.rect_map_x, camera0.rect_map_y, cv::INTER_LINEAR);
  cv::remap(_cur_img1, cur_img1, camera1.rect_map_x, camera1.rect_map_y, cv::INTER_LINEAR);
	time_ofs << "-1 " << cur_time-init_time <<" "<< (std::clock() - frame_start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;

  cv::GaussianBlur(cur_img0, cur_img0, cv::Size(BLUR_SZ,BLUR_SZ), BLUR_VAR);
  cv::GaussianBlur(cur_img1, cur_img1, cv::Size(BLUR_SZ,BLUR_SZ), BLUR_VAR);

	cur_time = _cur_time;
	monoTrack(cur_state, cur_img0, cur_img1, camera0, camera1, last_frame0, keyframes0, "Left");
	// monoTrack(cur_state, cur_img1, cur_img0, camera1, camera0, last_frame1, keyframes1, "Right");
  cur_state.showPose();
  comparer->write_vo(cur_state.pose, cur_time, stereo_match_flag);
  if(DEBUG_FEATURE){
  	cv::waitKey();
  }
	time_ofs << "0 " << cur_time-init_time <<" "<< (std::clock() - frame_start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
}

void StereoCamera::monoTrack(State& cur_state, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1,
							 Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name) {
	std::clock_t frame_start = std::clock();
	cur_state.time = ros::Time(cur_time);

	// need to restart
	if(last_frame.time < 0 || last_frame.time > cur_time || (cur_time-last_frame.time) > 1.0 || param_changed) {
		std::cout<<"Reset"<<std::endl;
		cur_state.reset();
		keyframes.clear();
		frame_dropped_count = 0;
		param_changed = false;
		init_time = cur_time;
	}

	//initialize key_frame
  if (keyframes.empty() || keyframes.back().features0.size() < MIN_TRACK_POINTS 	// feature tracking has too less features
 	|| (last_frame.feature_points.points.size() < MIN_TRACK_POINTS									// state propagtion has too less points
 		)) {
		if(!keyframes.empty()) frame_dropped_count++;

		KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_img0, cur_img1, FeaturePoints(), cam0);
	  keyframes.push_back(keyframe);
		stereo_match_flag = true;
		time_ofs << "3 " << cur_time-init_time <<" "<< (std::clock() - frame_start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
		// std::cout<<"KeyFrame("<<keyframes.size()<<") Dropped ("<<frame_dropped_count<<") frames"<<std::endl;

	  last_frame.features = keyframe.features0;
	  last_frame.feature_points = keyframe.feature_points;

	  last_frame.img = keyframe.img0.clone();
		last_frame.time = cur_time;
		last_frame.pose = cur_state.pose;

  	return;
  }

  KeyFrame& lastKF = keyframes.back();

	std::vector<cv::Point2f> cur_features;		//result of featureTrack
	FeaturePoints cur_feature_points;									//result of propagateState
	cv::Mat prop_img;

	// std::clock_t start = std::clock();
	// // feature tracking
	// std::thread thread_featureTrack(&StereoCamera::featureTrack, this, std::ref(lastKF), std::ref(last_frame), std::ref(cur_img0), std::ref(cur_features));
	//
	// propagateState(cur_state, cur_img0, last_frame, cam0, cur_feature_points, prop_img);
	//
	// // //propagateState
	// // std::thread thread_propagateState(&StereoCamera::propagateState, this, std::ref(cur_state), std::ref(cur_img0), cur_time, std::ref(last_frame), std::ref(cam0),
	// // 								  std::ref(cur_feature_points), std::ref(prop_img));
	//
	// thread_featureTrack.join();
	// // thread_propagateState.join();

	if(!TEST_STEREO) featureTrack(lastKF, last_frame, cur_img0, cur_features);
	propagateState(cur_state, cur_img0, last_frame, cam0, cur_feature_points, prop_img);

	// std::cout << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;

	cv::imshow(window_name + " propagate projection", prop_img);

	last_frame.features = cur_features;
	last_frame.feature_points = cur_feature_points;

	time_ofs << "1 " << cur_time-init_time <<" "<< (std::clock() - frame_start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;

	// create new KeyFrame when moving enough distance
	// transformation from lastKF, accumulated by propagtion
	Eigen::Matrix3d R_lastKF_c2w = lastKF.pose.orientation.toRotationMatrix() * cam0.R_C2B;
	Eigen::Vector3d t_lastKF_c2w = lastKF.pose.orientation.toRotationMatrix() * cam0.t_C2B + lastKF.pose.position;
	Eigen::Matrix3d R_cur_c2w = cur_state.pose.orientation.toRotationMatrix() * cam0.R_C2B;
	Eigen::Vector3d t_cur_c2w = cur_state.pose.orientation.toRotationMatrix() * cam0.t_C2B + cur_state.pose.position;
	Eigen::Matrix3d R_lastKF2Cur = R_cur_c2w.transpose() * R_lastKF_c2w;
	Eigen::Vector3d t_lastKF2Cur = R_cur_c2w.transpose() * (t_lastKF_c2w - t_cur_c2w);
	if (t_lastKF2Cur.norm() > MIN_TRACK_DIST)
	{
		std::clock_t kf_start = std::clock();

		// new features for new KF, tracked by propagtion
		cv::Mat R, t;
		cv::eigen2cv(R_lastKF2Cur, R);
		cv::eigen2cv(t_lastKF2Cur, t);
		FeatureTrackingResult feature_tracking_result(lastKF.features0, cur_features, R, t);

		// create new KeyFrame
		cv::Mat proj_img;
		FeaturePoints curKF_fts_pts;
		if(TEST_STEREO || reconstructAndOptimize(feature_tracking_result, lastKF, cam0, cam1, cur_state.pose, curKF_fts_pts, proj_img, cur_img0, cur_img1)) {

	    // construct new keyframe
			KeyFrame keyframe;
			if(TEST_STEREO) {
				kf_start = std::clock();
				keyframe = createKeyFrame(cur_state.pose, cur_img0, cur_img1, FeaturePoints(), cam0);
			}
			else {
				keyframe = createKeyFrame(cur_state.pose, cur_img0, cur_img1, curKF_fts_pts);
				cv::imshow(window_name + " projection", proj_img);
			}
		  keyframes.push_back(keyframe);
			stereo_match_flag = false;
			// std::cout<<"KeyFrame("<<keyframes.size()<<") Dropped ("<<frame_dropped_count<<") frames"<<std::endl;
			if (LOOP_CLOSURE) {
				// std::clock_t start = std::clock();
				local_KF_optimizer.optimize(keyframes, 5, cam0);
				// std::cout << "local_KF_optimizer: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
			}

			// update last frame with new points
		  last_frame.features = keyframe.features0;
		  last_frame.feature_points = keyframe.feature_points;

			if(TEST_STEREO) time_ofs << "3 " << cur_time-init_time <<" "<< (std::clock() - kf_start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
			else time_ofs << "2 " << cur_time-init_time <<" "<< (std::clock() - kf_start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
		}
	}

  // update last_frame
	last_frame.img = cur_img0.clone();
	last_frame.pose = cur_state.pose;
	last_frame.time = cur_time;
	cv::waitKey(1);
}

void StereoCamera::featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features) {
  std::vector<cv::Point2f> last_frame_features_back, lastKF_features_inlier, last_frame_features_inlier, cur_features_inlier;
  std::vector<uchar> status, status_back;
  std::vector<float> err, err_back;

	// bi-directional optical flow to find feature correspondence temporally
	std::clock_t start = std::clock();
  // std::cout<<last_frame.features.size()<<std::endl;
  cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.features, cur_features, status, err, cv::Size(OF_size,OF_size), FEATURE_OF_PYMD-1);
  cv::calcOpticalFlowPyrLK(cur_img, last_frame.img, cur_features, last_frame_features_back, status_back, err_back, cv::Size(OF_size,OF_size), FEATURE_OF_PYMD-1);

	// remove outliers
	cv::Mat line_img;
	cv::vconcat(lastKF.img0, cur_img, line_img);
  for(int i=0; i<status.size(); i++){
		// remove outlier by forward-backward matching
  	if(!status[i] || !status_back[i] || cv::norm(last_frame.features[i]-last_frame_features_back[i]) > 1) continue;

  	lastKF_features_inlier.push_back(lastKF.features0[i]);
  	last_frame_features_inlier.push_back(last_frame.features[i]);
  	cur_features_inlier.push_back(cur_features[i]);
	  if(DEBUG_FEATURE) cv::line(line_img, lastKF.features0[i], cv::Point2f(cur_features[i].x, cur_features[i].y+lastKF.img0.rows), cv::Scalar(255,0,0));
  }

  if(DEBUG_FEATURE) cv::imshow("featureTrack", line_img);

  lastKF.features0 = lastKF_features_inlier;
	last_frame.features = last_frame_features_inlier;
	cur_features = cur_features_inlier;
	time_ofs << "11 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;
}

void StereoCamera::propagateState(State& cur_state, const cv::Mat& cur_img, Frame& last_frame, const CameraModel& cam, FeaturePoints& cur_feature_points, cv::Mat& prop_img) {
	prop_img = cur_img.clone();
	Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
	Eigen::Vector3d t = Eigen::Vector3d::Zero();

	// calculate pose by direct method
	std::clock_t start = std::clock();
	double dist = pose_estimater.poseEstimate(last_frame.feature_points, last_frame.img, cam.K, cur_img, PROP_POSE_PYMD, PROP_POSE_ITER, R, t);
	time_ofs << "121 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;

	// refine pose by optical flow
	start = std::clock();
	cv::Mat r_cv, R_cv, t_cv;
	cv::eigen2cv(R, R_cv);
	cv::eigen2cv(t, t_cv);
	cv::Rodrigues(R_cv, r_cv);
	std::vector<cv::Point2f> new_fts;
  std::vector<uchar> status;
  std::vector<float> err;
  cv::projectPoints(last_frame.feature_points.points, r_cv, t_cv, cam.K, cv::Mat::zeros(1,4,CV_64F), new_fts);
  cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.feature_points.features, new_fts, status, err, cv::Size(OF_size,OF_size), PROP_POSE_PYMD-1);
	// remove point outliers
  std::vector<cv::Point2f> _tmp0, _tmp1;
  std::vector<cv::Point3f> _tmp2;
  for(int i=0; i<status.size(); i++){
  	if(!status[i]) continue;
  	_tmp0.push_back(last_frame.feature_points.features[i]);
  	_tmp1.push_back(new_fts[i]);
  	_tmp2.push_back(last_frame.feature_points.points[i]);
  }
  last_frame.feature_points.features = _tmp0;
  new_fts = _tmp1;
  last_frame.feature_points.points = _tmp2;
	// refine pose
  cv::Mat inlier_mask;
  cv::solvePnPRansac( last_frame.feature_points.points, new_fts, cam.K, cv::Mat(), r_cv, t_cv, false, 50, 3.0, 0.9, inlier_mask);
  if(DEBUG_FEATURE) std::cout<<"PropagateState: solvePnPRansac inlier_ratio "<<float(inlier_mask.rows) << "/" << float(new_fts.size())<<std::endl;
	if(float(inlier_mask.rows) / float(new_fts.size()) < 0.7) return;
	cv::Rodrigues(r_cv, R_cv);
  cv::cv2eigen(R_cv, R);
  cv::cv2eigen(t_cv, t);

  // propagate feature_points to current frame
  cv::cvtColor(prop_img, prop_img, cv::COLOR_GRAY2BGR);
  int marker_size = prop_img.rows / 50;
	for(int i=0; i<inlier_mask.rows; i++) {
		cv::Point3f& p = last_frame.feature_points.points[inlier_mask.at<int>(0,i)];
		cv::Point2f& f = new_fts[inlier_mask.at<int>(0,i)];
		cv::Mat new_p_m = R_cv * (cv::Mat_<double>(3,1) << p.x, p.y, p.z) + t_cv;
		cur_feature_points.points.push_back(cv::Point3f(new_p_m));
		cur_feature_points.features.push_back(f);
    cv::drawMarker(prop_img, f, cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
	}
	time_ofs << "122 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;

	Eigen::Matrix3d _R = last_frame.pose.orientation.toRotationMatrix() * cam.R_C2B * R.transpose();
	Eigen::Matrix3d R_w = _R * cam.R_B2C;
	Eigen::Vector3d t_w = _R*(cam.t_B2C-t) + last_frame.pose.orientation.toRotationMatrix()*cam.t_C2B + last_frame.pose.position;
  Eigen::Vector3d velocity = (t_w - last_frame.pose.position) / (cur_time - last_frame.time);
  if(velocity.norm() > MAX_VEL) {
		std::cout<<"exceed max vel: "<<velocity.norm()<<std::endl;
		return;
	}
	cur_state.pose.position = t_w;
	cur_state.pose.orientation = Eigen::Quaterniond(R_w);
}

bool StereoCamera::reconstructAndOptimize(FeatureTrackingResult feature_result, const KeyFrame& lastKF,
										  const CameraModel& cam0, const CameraModel& cam1,
										  Pose& cur_pose, FeaturePoints& curKF_fts_pts, cv::Mat& proj_img,
										  const cv::Mat& cur_img0, const cv::Mat& cur_img1)
{
  //reconstruct 3D pts and bundle adjustment
	std::vector<PointWithUncertainty> lastKF_pts;		// pts with scale=1
	double scale = cv::norm(feature_result.t_lastKF2Cur);	// initial scale
	feature_result.t_lastKF2Cur = feature_result.t_lastKF2Cur / scale;	// normalize translation

	std::clock_t start = std::clock();
	assert(feature_result.lastKF_features.size() > 1);
	cv::Mat reproj = lastKF.img0.clone();
	cv::cvtColor(reproj, reproj, cv::COLOR_GRAY2BGR);
	reconstructor.reconstructAndBundleAdjust(feature_result.lastKF_features, feature_result.cur_features, cam0.K, feature_result.R_lastKF2Cur, feature_result.t_lastKF2Cur, BA_MAX_STEP, MAX_REPROJ_DIST, lastKF_pts, reproj);
	if(lastKF_pts.size() / feature_result.cur_features.size() < MONO_INLIER_THRES) return false;
	time_ofs << "131 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;

	// std::cout<<"scale before "<<scale<<std::endl;
	bool badPropagate = false;
	int opt_scale = 1;
	// std::cout<<"scale"<<scale<<std::endl;
	if(!(scale>0.01 && scale<MAX_SCALE)) {
		badPropagate = true;
		opt_scale = 2;
		std::cout<<"bad scale before: "<<scale<<std::endl;
		scale = INIT_SCALE;
	}


	// optimization
	double new_scale = scale;

// std::cout<<feature_result.lastKF_features.size()<<" "<<lastKF_pts.size()<<std::endl;
	start = std::clock();
  double scale_err = scale_optimizer.optimize(feature_result.lastKF_features, lastKF_pts, new_scale, cam1, lastKF.img0, lastKF.img1, opt_scale*SCALE_PYMD, opt_scale*SCALE_MAX_STEP);
	// std::cout<<"scale err = "<<scale_err<<std::endl;
	// if(scale_err > 2000.0) return false;
  time_ofs << "132 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << std::endl;

  scale = new_scale;

	if(!(scale>0.01 && scale<MAX_SCALE)) {
		std::cout<<"bad scale after: "<<scale<<std::endl;
		scale = INIT_SCALE;
		// return false;
	}

	std::vector<cv::Point3f> lastKF_pts_points;
	for(int i=0; i<lastKF_pts.size(); i++) {
		cv::Point3f p = scale*lastKF_pts[i].point;
		lastKF_pts_points.push_back(p);
	}

	// refine stereo correspondence
	std::vector<cv::Point2f> lastKF_pts_proj;
	cv::Mat r_cv;
	cv::Rodrigues(cam1.stereo.R, r_cv);
  cv::projectPoints(lastKF_pts_points, r_cv, cam1.stereo.t, cam1.K, cv::Mat::zeros(1,4,CV_64F), lastKF_pts_proj);

	cv::Mat proj_after_opt = lastKF.img1.clone();
	cv::cvtColor(proj_after_opt, proj_after_opt, cv::COLOR_GRAY2BGR);
	int marker_size = proj_after_opt.rows / 50;
	for(int i=0; i<lastKF_pts_proj.size(); i++) {
        double u = lastKF_pts_proj[i].x;
        double v = lastKF_pts_proj[i].y;
        if(0<=u && u<proj_after_opt.cols && 0<=v && v<proj_after_opt.rows) {
	        cv::drawMarker(proj_after_opt, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
	    }
	}
	cv::hconcat(reproj, proj_after_opt, proj_img);

	// refine with LK
	curKF_fts_pts.points.clear();
	if(REFINE_PIXEL) {
		assert(feature_result.lastKF_features.size() == lastKF_pts_proj.size());
	  reconstructor.refinePixel(lastKF.img0, lastKF.img1, feature_result.lastKF_features, lastKF_pts_proj);

		float b = fabs(cam0.stereo.t.at<double>(0,0));
		float f = cam0.K.at<double>(0,0);
		float cx = cam0.K.at<double>(0,2);
		float cy = cam0.K.at<double>(1,2);
		for(int i=0; i<feature_result.lastKF_features.size(); i++) {
			//triangulate point from stereo correspondence
			float u = feature_result.lastKF_features[i].x;
			float v = feature_result.lastKF_features[i].y;
			float d = fabs(u - lastKF_pts_proj[i].x);
			float b_d = b / d;
			cv::Point3f p(b_d*(u-cx), b_d*(v-cy), b_d*f);

			// bring it to current frame
			cv::Mat p_cur = (feature_result.R_lastKF2Cur *(cv::Mat_<double>(3,1)<<p.x, p.y, p.z)) + scale*feature_result.t_lastKF2Cur;

			curKF_fts_pts.points.push_back(cv::Point3f(p_cur.at<double>(0,0), p_cur.at<double>(1,0), p_cur.at<double>(2,0)));
		}
	} else {
		for(int i=0; i<lastKF_pts_points.size(); i++) {
			cv::Point3f p = lastKF_pts_points[i];
			// bring it to current frame
			cv::Mat p_cur = (feature_result.R_lastKF2Cur *(cv::Mat_<double>(3,1)<<p.x, p.y, p.z)) + scale*feature_result.t_lastKF2Cur;

			curKF_fts_pts.points.push_back(cv::Point3f(p_cur.at<double>(0,0), p_cur.at<double>(1,0), p_cur.at<double>(2,0)));
		}

	}

	curKF_fts_pts.features.clear();
	curKF_fts_pts.uncertainties.clear();
	for(int i=0; i<feature_result.cur_features.size(); i++) {
		curKF_fts_pts.features.push_back(feature_result.cur_features[i]);
		curKF_fts_pts.uncertainties.push_back(lastKF_pts[i].uncertainty);
	}

  // update current state if brute propagation
  // if(badPropagate)
	// {
	// 	Eigen::Matrix3d R_lastKF2Cur_eigen;
	// 	Eigen::Vector3d t_lastKF2Cur_eigen;
	// 	cv::cv2eigen(feature_result.R_lastKF2Cur, R_lastKF2Cur_eigen);
	// 	cv::cv2eigen(feature_result.t_lastKF2Cur, t_lastKF2Cur_eigen);
	// 	t_lastKF2Cur_eigen = scale * t_lastKF2Cur_eigen;
	// 	Eigen::Matrix3d R_w2Cur = R_lastKF2Cur_eigen * cam0.R_B2C * lastKF.pose.orientation.toRotationMatrix().transpose();
	// 	Eigen::Vector3d t_Cur = (Eigen::Matrix3d::Identity() - R_lastKF2Cur_eigen) * cam0.t_B2C;
	//
	// 	cur_pose.position = lastKF.pose.position - R_w2Cur.transpose() * (t_lastKF2Cur_eigen - t_Cur);
	// 	cur_pose.orientation = Eigen::Quaterniond(R_w2Cur.transpose() * cam0.R_B2C);
	// }

	return true;
}
