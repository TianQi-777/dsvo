#include "stereo_processor/stereo_processor.hpp"

void StereoProcessor::track(const cv::Mat& cur_img0, const cv::Mat& cur_img1, double _cur_time) {
	cur_time = _cur_time;
	// need to restart
	if(last_time > cur_time || (cur_time-last_time) > 1.0 || param_changed) {
		std::cout<<"Reset"<<std::endl;
		state.reset();
		keyframes0.clear();
		keyframes1.clear();
		point_cloud->clear();
		param_changed = false;
		init_time = cur_time;
	}

	std::clock_t frame_start = std::clock();

  cv::GaussianBlur(cur_img0, cur_img0, cv::Size(BLUR_SZ,BLUR_SZ), BLUR_VAR);
  cv::GaussianBlur(cur_img1, cur_img1, cv::Size(BLUR_SZ,BLUR_SZ), BLUR_VAR);
	monoTrack(cur_img0, cur_img1, camera0, camera1, last_frame0, keyframes0, "Left");
	// monoTrack(cur_img1, cur_img0, camera1, camera0, last_frame1, keyframes1, "Right");

  state.showPose();
  comparer->write_vo(state.pose, cur_time, stereo_match_flag);
	time_ofs << "0 " << cur_time-init_time <<" "<< (std::clock() - frame_start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;
  // if(DEBUG) cv::waitKey();
}

void StereoProcessor::monoTrack(const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1,
							 Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name) {

	std::clock_t frame_start = std::clock();
	state.time = ros::Time(cur_time);

	//initialize key_frame at start or when state propagtion has too few points
  if (keyframes.empty() || last_frame.feature_points.size() < PROP_MIN_POINTS) {
		// if(DEBUG)
		{
			if(last_frame.feature_points.size() < PROP_MIN_POINTS) std::cout<<"state propagtion does not has enough points: "<<last_frame.feature_points.size()<<std::endl;
		}

		KeyFrame keyframe = createKeyFrame(state.pose, cur_img0, cur_img1, cam0);
	  keyframes.push_back(keyframe);
		stereo_match_flag = true;
		time_ofs << "3 " << cur_time-init_time <<" "<< (std::clock() - frame_start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;

	  last_frame.features = keyframe.new_features;
	  last_frame.feature_points = keyframe.feature_points;
		last_frame.pose_from_lastKF = Pose();
	  last_frame.img = keyframe.img0.clone();
		last_time = cur_time;

  	return;
  }

  KeyFrame& lastKF = keyframes.back();

	// feature tracking for new points of new KeyFrame
	std::vector<cv::Point2f> cur_features;
	if(!TEST_STEREO) featureTrack(lastKF, last_frame, cur_img0, cur_features);
	last_frame.features = cur_features;

	// propagate current state using existing feature points
	FeaturePoints cur_feature_points;
	if(!propagateState(cur_img0, lastKF, last_frame, cam0, cur_feature_points)) return;
	last_frame.feature_points = cur_feature_points;

	time_ofs << "1 " << cur_time-init_time <<" "<< (std::clock() - frame_start) / (double)(CLOCKS_PER_SEC / 1000) << " " << last_frame.feature_points.size() << std::endl;

	// transformation from lastKF, accumulated by propagtion
	Eigen::Matrix3d R_lastKF_c2w = lastKF.pose.orientation.toRotationMatrix() * cam0.R_C2B;
	Eigen::Vector3d t_lastKF_c2w = lastKF.pose.orientation.toRotationMatrix() * cam0.t_C2B + lastKF.pose.position;
	Eigen::Matrix3d R_cur_c2w = state.pose.orientation.toRotationMatrix() * cam0.R_C2B;
	Eigen::Vector3d t_cur_c2w = state.pose.orientation.toRotationMatrix() * cam0.t_C2B + state.pose.position;
	Eigen::Matrix3d R_lastKF2Cur = R_cur_c2w.transpose() * R_lastKF_c2w;
	Eigen::Vector3d t_lastKF2Cur = R_cur_c2w.transpose() * (t_lastKF_c2w - t_cur_c2w);
	if(DEBUG) std::cout<<std::endl<<"propagateState: "<<cur_feature_points.size()<<" t_lastKF2Cur norm "<<t_lastKF2Cur.norm()<<std::endl;

	// create new KeyFrame after moving enough distance
	if (t_lastKF2Cur.norm()>KF_DIST)
	{
		std::clock_t kf_start = std::clock();

		if(TEST_STEREO) {
			KeyFrame keyframe;
			keyframe = createKeyFrame(state.pose, cur_img0, cur_img1, cam0);
		  keyframes.push_back(keyframe);

		  last_frame.feature_points = keyframe.feature_points;
			last_frame.pose_from_lastKF = Pose();

			time_ofs << "3 " << cur_time-init_time <<" "<< (std::clock() - kf_start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;
		} else {
			// new features for new KF, combined with current propagated feature points
			// std::vector<cv::Point2f> lastKF_features = lastKF.new_features;
			// lastKF_features.insert(lastKF_features.end(), lastKF.feature_points.features.begin(), lastKF.feature_points.features.end());
			// cur_features.insert(cur_features.end(), cur_feature_points.features.begin(), cur_feature_points.features.end());
			cv::Mat R, t;
			cv::eigen2cv(R_lastKF2Cur, R);
			cv::eigen2cv(t_lastKF2Cur, t);
			KFData kf_data(R, t, lastKF.feature_points, cur_feature_points.features(), lastKF.new_features, cur_features);

			// create new KeyFrame if reconstructAndOptimize succeed
			FeaturePoints new_feature_points;
			std::vector<bool> new_pts_flags;
			if(reconstructAndOptimize(kf_data, lastKF, cam0, cam1, state.pose, new_feature_points, new_pts_flags, cur_img0, cur_img1)) {

		    // construct new keyframe
				KeyFrame keyframe;
				keyframe = createKeyFrame(state.pose, cur_img0, cur_img1, cam0, new_feature_points, new_pts_flags);
				stereo_match_flag = false;
			  keyframes.push_back(keyframe);

				if (LOOP_CLOSURE) {
					// std::clock_t start = std::clock();
					local_KF_optimizer.optimize(keyframes, 5, cam0);
					// std::cout << "local_KF_optimizer: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
				}

				// update last frame with new points
			  last_frame.features = keyframe.new_features;
			  last_frame.feature_points = keyframe.feature_points;
				last_frame.pose_from_lastKF = Pose();

				time_ofs << "2 " << cur_time-init_time <<" "<< (std::clock() - kf_start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;
			}
		}
	}

  // update last_frame
	last_frame.img = cur_img0.clone();
	last_time = cur_time;
	cv::waitKey(1);
}

void StereoProcessor::featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features) {
	if(last_frame.features.empty()) return;

	std::clock_t start = std::clock();

	// bi-directional optical flow to find feature correspondence temporally
  std::vector<cv::Point2f> last_frame_features_back, lastKF_features_inlier, last_frame_features_inlier, cur_features_inlier;
  std::vector<uchar> status, status_back;
  std::vector<float> err, err_back;
  cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.features, cur_features, status, err, cv::Size(OF_size,OF_size), FEATURE_OF_PYMD-1);
  cv::calcOpticalFlowPyrLK(cur_img, last_frame.img, cur_features, last_frame_features_back, status_back, err_back, cv::Size(OF_size,OF_size), FEATURE_OF_PYMD-1);

	// remove outliers
	cv::Mat line_img;
	cv::vconcat(lastKF.img0, cur_img, line_img);
  for(int i=0; i<status.size(); i++){
		// remove outlier by forward-backward matching
  	if(!status[i] || !status_back[i] || cv::norm(last_frame.features[i]-last_frame_features_back[i]) > 1) continue;

  	lastKF_features_inlier.push_back(lastKF.new_features[i]);
  	last_frame_features_inlier.push_back(last_frame.features[i]);
  	cur_features_inlier.push_back(cur_features[i]);
	  if(DEBUG) cv::line(line_img, lastKF.new_features[i], cv::Point2f(cur_features[i].x, cur_features[i].y+lastKF.img0.rows), cv::Scalar(255,0,0));
  }

	if(DEBUG) std::cout<<"featureTrack: "<<status.size()<<" -> "<<cur_features_inlier.size()<<std::endl;
  if(DEBUG) cv::imshow("featureTrack", line_img);

  lastKF.new_features = lastKF_features_inlier;
	cur_features = cur_features_inlier;
	time_ofs << "11 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;
}

bool StereoProcessor::propagateState(const cv::Mat& cur_img, KeyFrame& lastKF, Frame& last_frame, const CameraModel& cam, FeaturePoints& cur_feature_points) {
	std::clock_t start = std::clock();

	/******************************************calculate pose by direct method******************************************/
	Eigen::Matrix3d R_last_frame = Eigen::Matrix3d::Identity();
	Eigen::Vector3d t_last_frame = Eigen::Vector3d::Zero();
	double dist = pose_estimater.poseEstimate(last_frame.feature_points, last_frame.img, cam.K, cur_img, PROP_PYMD, PROP_MAX_STEP, R_last_frame, t_last_frame);
	time_ofs << "121 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;
	/******************************************calculate pose by direct method******************************************/

	/******************************************find feature correspondences by optical flow******************************************/
	start = std::clock();
	cv::Mat r_cv, R_cv, t_cv;
	cv::eigen2cv(R_last_frame, R_cv);
	cv::eigen2cv(t_last_frame, t_cv);
	cv::Rodrigues(R_cv, r_cv);
	std::vector<cv::Point2f> cur_features, cur_features_refined;
  cv::projectPoints(last_frame.feature_points.points(), r_cv, t_cv, cam.K, cv::Mat::zeros(1,4,CV_64F), cur_features);
	cur_features_refined = cur_features;
  std::vector<uchar> status;
  std::vector<float> err;
	// cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.feature_points.features, cur_features_refined, status, err, cv::Size(OF_size,OF_size), PROP_PYMD);
  // cv::calcOpticalFlowPyrLK(last_frame.img, cur_img, last_frame.feature_points.features(), cur_features_refined, status, err, cv::Size(OF_size,OF_size), PROP_PYMD,
	// 												 cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
	std::vector<cv::Point3f> last_frame_feature_points_features_d;
	for(int i=0; i<last_frame.feature_points.size(); i++) {
		last_frame_feature_points_features_d.push_back(cv::Point3f(last_frame.feature_points[i].feature.x,
																															 last_frame.feature_points[i].feature.y,
																															 last_frame.feature_points[i].point.point.z));
	}
	klt.compute(last_frame.img, cur_img, last_frame_feature_points_features_d, cur_features_refined, cam.K, R_cv, t_cv, status, err, PROP_PYMD);

	// remove point outliers
	std::vector<cv::Point2f> lastKF_features_inlier, cur_features_refined_inlier;
  PointsWithUncertainties lastKF_points_inlier;
  for(int i=0; i<status.size(); i++){
		if(!status[i]) continue;

		cur_features_refined_inlier.push_back(cur_features_refined[i]);
		lastKF_features_inlier.push_back(lastKF.feature_points[i].feature);
		lastKF_points_inlier.push_back(lastKF.feature_points[i].point.point, cv::norm(cur_features[i] - cur_features_refined[i]));	//TODO
  }
	if(DEBUG) std::cout<<"propagate refine: "<<last_frame.feature_points.size()<<" -> "<<lastKF_points_inlier.size()<<std::endl;

	if(float(lastKF_points_inlier.size()) / last_frame.feature_points.size() < PROP_PTS_RATIO){
		std::cout<<"propagateState refine ratio too low "<<std::endl;
		last_frame.feature_points.clear();
		// cv::waitKey();
		return false;
	}
	/******************************************find feature correspondences by optical flow******************************************/

	/******************************************refine pose******************************************/
	Eigen::Matrix3d R_lastKF = R_last_frame * last_frame.pose_from_lastKF.orientation.toRotationMatrix();
	Eigen::Vector3d t_lastKF = R_last_frame * last_frame.pose_from_lastKF.position + t_last_frame;
	pose_estimater.refine_pose(lastKF_points_inlier, cur_features_refined_inlier, cam.K, R_lastKF, t_lastKF);

  // propagate feature_points to current frame
	cv::eigen2cv(R_lastKF, R_cv);
	cv::eigen2cv(t_lastKF, t_cv);
	cv::Scalar marker_color = cv::Scalar(0,0,255);
	if(TEST_STEREO || stereo_match_flag) marker_color = cv::Scalar(0,255,0);
	lastKF.feature_points.clear();
	cur_feature_points.clear();
	cv::Mat prop_img = cur_img.clone();
  cv::cvtColor(prop_img, prop_img, cv::COLOR_GRAY2BGR);
  int marker_size = prop_img.rows / 50;
	for(int i=0; i<lastKF_points_inlier.size(); i++) {
		// project points from lastKF to current frame according to refined pose
		cv::Point3f& p = lastKF_points_inlier[i].point;
		cv::Mat new_p_m = R_cv * (cv::Mat_<double>(3,1) << p.x, p.y, p.z) + t_cv;
		cv::Mat pm = cam.K*new_p_m;
		cv::Point2f proj = cv::Point2f(pm.at<double>(0) / pm.at<double>(2),pm.at<double>(1) / pm.at<double>(2));
		cv::Point2f& f = cur_features_refined_inlier[i];	// current feature position refined by optical flow
		cv::circle(prop_img, f, 3, cv::Scalar(0,255,0));
		cv::line(prop_img, f, proj, cv::Scalar(0,255,255));

		// remove outlier by reprojection error
		double reproj_err = sqrt((proj.x-f.x)*(proj.x-f.x)+(proj.y-f.y)*(proj.y-f.y));
		if(reproj_err<PROP_PROJ_DIST) {
			lastKF.feature_points.push_back(lastKF_features_inlier[i], lastKF_points_inlier[i].point, reproj_err);		//TODO
			cur_feature_points.push_back(f, cv::Point3f(new_p_m), reproj_err);
	    cv::drawMarker(prop_img, proj, marker_color, cv::MARKER_CROSS, marker_size);
		}
		else {
			cv::circle(prop_img, proj, 4, cv::Scalar(0,0,255));
		}
	}

	if(DEBUG) std::cout<<"propagateState ratio "<<lastKF.feature_points.size()<<" / "<<lastKF_points_inlier.size()<<std::endl;
	cv::imshow("propagate projection", prop_img);

	if(float(lastKF.feature_points.size()) / lastKF_points_inlier.size() < PROP_PTS_RATIO){
		std::cout<<"propagateState ratio too low "<<std::endl;
		last_frame.feature_points.clear();
		// cv::waitKey();
		return false;
	}

	time_ofs << "122 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;

	/******************************************refine pose******************************************/

	// propagate last frame pose
	last_frame.pose_from_lastKF.orientation = Eigen::Quaterniond(R_lastKF);
	last_frame.pose_from_lastKF.position = t_lastKF;

	// propagate state
	Eigen::Matrix3d _R = lastKF.pose.orientation.toRotationMatrix() * cam.R_C2B * R_lastKF.transpose();
	Eigen::Matrix3d R_w = _R * cam.R_B2C;
	Eigen::Vector3d t_w = _R*(cam.t_B2C-t_lastKF) + lastKF.pose.orientation.toRotationMatrix()*cam.t_C2B + lastKF.pose.position;
  Eigen::Vector3d velocity = (t_w - lastKF.pose.position) / (cur_time - lastKF.time);
	if(velocity.norm() > MAX_VEL) {
		std::cout<<"exceed max velocity: "<<velocity.norm()<<std::endl;
		last_frame.feature_points.clear();
		return false;
	}
	state.pose.position = t_w;
	state.pose.orientation = Eigen::Quaterniond(R_w);

	return true;
}

bool StereoProcessor::reconstructAndOptimize(KFData& kf_data, const KeyFrame& lastKF,
										  const CameraModel& cam0, const CameraModel& cam1,
										  Pose& cur_pose, FeaturePoints& curKF_fts_pts, std::vector<bool>& curKF_new_pts_flags,
										  const cv::Mat& cur_img0, const cv::Mat& cur_img1)
{
	std::clock_t start = std::clock();

  //store original scale and normalize translation
	double scale = cv::norm(kf_data.t_lastKF2Cur);	// initial scale
	kf_data.t_lastKF2Cur = kf_data.t_lastKF2Cur / scale;	// normalize translation

	/******************************************reconstruct and bundle adjust******************************************/
	float orig_fts_size = kf_data.size();
	cv::Mat _t_lastKF2Cur = kf_data.t_lastKF2Cur;
	cv::transpose(_t_lastKF2Cur, _t_lastKF2Cur);
	reconstructor.reconstructAndBundleAdjust(kf_data, cam0.K, BA_MAX_STEP, BA_REPROJ_DIST);
	cv::Mat _angle = _t_lastKF2Cur*kf_data.t_lastKF2Cur;
	double angle = _angle.at<double>(0,0);
	if(DEBUG) std::cout<<"reconstructAndBundleAdjust angle= "<<angle<<" ratio= "<<kf_data.size() << "/" << orig_fts_size<<std::endl;
	if(angle<0.9 || kf_data.size() / orig_fts_size < BA_INLIER_THRES) return false;

	time_ofs << "131 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;

	// draw reconstruction reprojection
	if(DEBUG) {
		cv::Mat r_vec;
		cv::Rodrigues(kf_data.R_lastKF2Cur, r_vec);
		std::vector<cv::Point2f> reproj0_pts, reproj1_pts;
	  cv::projectPoints(kf_data.points.points(), cv::Mat::zeros(3,1,CV_64F), cv::Mat::zeros(3,1,CV_64F), cam0.K, cv::Mat::zeros(1,4,CV_64F), reproj0_pts);
	  cv::projectPoints(kf_data.points.points(), r_vec, kf_data.t_lastKF2Cur, cam0.K, cv::Mat::zeros(1,4,CV_64F), reproj1_pts);
		cv::Mat reproj0(lastKF.img0), reproj1(cur_img0);
		cv::cvtColor(reproj0, reproj0, cv::COLOR_GRAY2BGR);
		cv::cvtColor(reproj1, reproj1, cv::COLOR_GRAY2BGR);
		int marker_size = reproj0.rows / 50;
		for(int i=0; i<kf_data.size(); i++) {
			cv::circle(reproj0, kf_data.lastKF_features[i], marker_size, cv::Scalar(255,0,0));
			cv::drawMarker(reproj0, reproj0_pts[i], cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
			cv::circle(reproj1, kf_data.cur_features[i], marker_size, cv::Scalar(255,0,0));
			cv::drawMarker(reproj1, reproj1_pts[i], cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
		}
		cv::Mat reproj;
		cv::hconcat(reproj0, reproj1, reproj);
		cv::imshow("reconstructAndBundleAdjust", reproj);
		cv::waitKey(1);
	}

	/******************************************reconstruct and bundle adjust******************************************/

	/******************************************scale optimization******************************************/
	start = std::clock();
	double new_scale = scale;
  double scale_err = scale_optimizer.optimize(kf_data.lastKF_features, kf_data.points, new_scale, cam1, lastKF.img0, lastKF.img1, SCALE_PYMD, SCALE_MAX_STEP);
	if(DEBUG) std::cout<<"scale optimize err "<<scale_err<<std::endl;
	// if(scale_err > 100.0) return false;
  time_ofs << "132 " << cur_time-init_time << " " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " -1" << std::endl;
	if(!(new_scale>0.01 && new_scale<2*scale)) {
		std::cout<<"bad scale after: "<<new_scale<<std::endl;
		return false;
	}
  scale = new_scale;

	// rescale translation
	kf_data.t_lastKF2Cur = scale*kf_data.t_lastKF2Cur;
	// rescale points generated from reconstructAndBundleAdjust
	for(int i=0; i<kf_data.size(); i++) {
		kf_data.points[i].point = scale*kf_data.points[i].point;
	}

	// show scale optimization result
	if(DEBUG) {
		std::vector<cv::Point2f> lastKF_pts_proj;
		cv::Mat r_cv;
		cv::Rodrigues(cam1.stereo.R, r_cv);
	  cv::projectPoints(kf_data.points.points(), r_cv, cam1.stereo.t, cam1.K, cv::Mat::zeros(1,4,CV_64F), lastKF_pts_proj);

		cv::Mat scale_img = lastKF.img1.clone();
		cv::cvtColor(scale_img, scale_img, cv::COLOR_GRAY2BGR);
		int marker_size = scale_img.rows / 50;
		for(int i=0; i<lastKF_pts_proj.size(); i++) {
	        double u = lastKF_pts_proj[i].x;
	        double v = lastKF_pts_proj[i].y;
	        if(0<=u && u<scale_img.cols && 0<=v && v<scale_img.rows) {
		        cv::drawMarker(scale_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);
		    }
		}
		cv::imshow("scale optimization", scale_img);
	}

	/******************************************scale optimization******************************************/

	// bring points to current frame
	for(int i=0; i<kf_data.size(); i++) {
		cv::Point3f p = kf_data.points[i].point;
		cv::Mat p_cur = (kf_data.R_lastKF2Cur *(cv::Mat_<double>(3,1)<<p.x, p.y, p.z)) + kf_data.t_lastKF2Cur;

		curKF_fts_pts.push_back(kf_data.cur_features[i],
														cv::Point3f(p_cur.at<double>(0,0), p_cur.at<double>(1,0), p_cur.at<double>(2,0)),
														kf_data.points[i].uncertainty);

		curKF_new_pts_flags.push_back(kf_data.new_pts_flags[i]);
	}

	// [deprecated] refine stereo correspondence with LK
	if(REFINE_PIXEL) {
		curKF_fts_pts.clear();
		std::vector<cv::Point2f> lastKF_pts_proj;
		cv::Mat r_cv;
		cv::Rodrigues(cam1.stereo.R, r_cv);
	  cv::projectPoints(kf_data.points.points(), r_cv, cam1.stereo.t, cam1.K, cv::Mat::zeros(1,4,CV_64F), lastKF_pts_proj);

		float MAX_DISP = 50.0;
		std::vector<uchar> status;
		std::vector<float> err;
		cv::calcOpticalFlowPyrLK(lastKF.img0, lastKF.img1, kf_data.lastKF_features, lastKF_pts_proj, status, err);

		float cx = cam0.K.at<double>(0,2);
		float cy = cam0.K.at<double>(1,2);
		float B = cam0.stereo.t.at<double>(0,0);
		float f = cam0.K.at<double>(0,0);
	  float fB = f*B;
		for(int i=0; i<status.size(); i++) {
			if(!status[i]) continue;
			//triangulate point from stereo correspondence
			float u = kf_data.lastKF_features[i].x;
			float v = kf_data.lastKF_features[i].y;
			float dx = u - lastKF_pts_proj[i].x;
			float dy = v - lastKF_pts_proj[i].y;
		  if (!(dx > 0 && dx<MAX_DISP && fabs(dy)<1)) continue;

			float z = fB / dx;
			float x = (u - cx) * z / f;
			float y = (v - cy) * z / f;
			cv::Point3f p(x, y, z);

			// bring it to current frame
			cv::Mat p_cur = (kf_data.R_lastKF2Cur *(cv::Mat_<double>(3,1)<<p.x, p.y, p.z)) + kf_data.t_lastKF2Cur;

			curKF_fts_pts.push_back(kf_data.cur_features[i],
															cv::Point3f(p_cur.at<double>(0,0), p_cur.at<double>(1,0), p_cur.at<double>(2,0)),
															kf_data.points[i].uncertainty);
		}
	}

	// update current state
	Eigen::Matrix3d R_lastKF2Cur_eigen;
	Eigen::Vector3d t_lastKF2Cur_eigen;
	cv::cv2eigen(kf_data.R_lastKF2Cur, R_lastKF2Cur_eigen);
	cv::cv2eigen(kf_data.t_lastKF2Cur, t_lastKF2Cur_eigen);
	Eigen::Matrix3d R_w2Cur = R_lastKF2Cur_eigen * cam0.R_B2C * lastKF.pose.orientation.toRotationMatrix().transpose();
	Eigen::Vector3d t_Cur = (Eigen::Matrix3d::Identity() - R_lastKF2Cur_eigen) * cam0.t_B2C;

	cur_pose.position = lastKF.pose.position - R_w2Cur.transpose() * (t_lastKF2Cur_eigen - t_Cur);
	cur_pose.orientation = Eigen::Quaterniond(R_w2Cur.transpose() * cam0.R_B2C);

	return true;
}
