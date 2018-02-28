#include "stereo_camera.hpp"
#include <cmath>

// parameters
int MAX_REPROJ_DIST, KP_BLOCKS, INIT_MAX_KP, MAX_OPT_STEP, MIN_OPT_STEP, WINDOW_SIZE, PYMD;
double LM_DAMP, NEW_KF_KP_RATIO, MONO_INLIER_THRES;
geometry_msgs::PointStamped new_pose_msg, last_pose_msg;

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

void StereoCamera::updateConfig(direct_stereo::DirectStereoConfig &config, uint32_t level) {
	// std::cout<<"Init param"<<std::endl;
	MAX_REPROJ_DIST = config.MAX_REPROJ_DIST;
	KP_BLOCKS = config.KP_BLOCKS;
	INIT_MAX_KP = config.INIT_MAX_KP;
	MAX_OPT_STEP = config.MAX_OPT_STEP;
	MIN_OPT_STEP = config.MIN_OPT_STEP;
	WINDOW_SIZE = config.WINDOW_SIZE;
	PYMD = config.PYMD;
	LM_DAMP = config.LM_DAMP;
	NEW_KF_KP_RATIO = config.NEW_KF_KP_RATIO;
	MONO_INLIER_THRES = config.MONO_INLIER_THRES;
    param_changed = true;
}

void StereoCamera::truth_Callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	new_pose_msg.header = msg->header;
	new_pose_msg.header.frame_id = "/map";
	new_pose_msg.point = msg->point;
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
	pcl_pub = nh.advertise<PointCloud> ("points2", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 1000);

	f = boost::bind(&StereoCamera::updateConfig, this, _1, _2);
  	server.setCallback(f);
    
    last_time = -1;
    frame_dropped_count = 0;
    // directSolver_ptr = shared_ptr<DirectSolver>(new DirectSolver());
    param_changed = true;
    init_time = -1;

	truth_sub = nh.subscribe("/leica/position", 1000, &StereoCamera::truth_Callback, this);
	vo_ofs.open("vo.txt", std::ofstream::out | std::ofstream::trunc);
	gt_ofs.open("truth.txt", std::ofstream::out | std::ofstream::trunc);
}

void StereoCamera::reconstruct3DPts(std::vector<cv::Point2f>& features0, std::vector<cv::Point2f>& features1, 
									const cv::Mat& K, const cv::Mat& R, const cv::Mat& t,
									std::vector<cv::Point3d>& pts, cv::Mat& reproj_img){
	//get perspective projection matrix
    cv::Mat Pl, Pr;
    Pl = K*(cv::Mat_<double>(3,4) <<1,0,0,0,0,1,0,0,0,0,1,0);
    cv::hconcat(R, t, Pr);
    Pr = K*Pr;

    //reconstruct 3d feature points
	std::vector<cv::Point3d> _pts;
	std::vector<cv::Point2f> _features0 = features0;
	std::vector<cv::Point2f> _features1 = features1;
    for(unsigned int i=0; i<_features0.size(); i++) {
        float ul = _features0[i].x;
        float vl = _features0[i].y;
        float ur = _features1[i].x;
        float vr = _features1[i].y;
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
	features0.clear();
	features1.clear();
	pts.clear();
    cv::cvtColor(reproj_img, reproj_img, cv::COLOR_GRAY2BGR);
	for(int i=0; i<_pts.size(); i++) {
        cv::Mat X0 = (cv::Mat_<double>(3,1) << _pts[i].x, _pts[i].y, _pts[i].z);
        cv::Mat u1 = K*X0;
        double u = u1.at<double>(0,0) / u1.at<double>(2,0);
        double v = u1.at<double>(1,0) / u1.at<double>(2,0);

		// reject outlier by reprojection
		if(cv::norm(_features0[i]-cv::Point2f(u, v)) > MAX_REPROJ_DIST) continue;

        if(0<=u && u<reproj_img.cols && 0<=v && v<reproj_img.rows) {
	        cv::drawMarker(reproj_img, cv::Point2d(u, v), cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
	    }

		cv::circle(reproj_img, _features0[i], 5, cv::Scalar(0,255,0));

		features0.push_back(_features0[i]);
		features1.push_back(_features1[i]);
		pts.push_back(_pts[i]);
	}
}

void StereoCamera::projectToImg(const std::vector<cv::Point3d>& pts, double scale, const CameraModel& cam, cv::Mat& proj_img) {
    cv::Mat P01;
    cv::hconcat(cam.stereo.R, cam.stereo.t, P01);
    P01 = cam.K*P01;
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
	//stereo rectify
	cv::Mat cur_img0, cur_img1;
    cv::remap(_cur_img0, _cur_img0, camera0.rect_map_x, camera0.rect_map_y, cv::INTER_LINEAR);
    cv::GaussianBlur(_cur_img0, cur_img0, cv::Size(3,3), 0.5);
    cv::remap(_cur_img1, _cur_img1, camera1.rect_map_x, camera1.rect_map_y, cv::INTER_LINEAR);
    cv::GaussianBlur(_cur_img1, cur_img1, cv::Size(3,3), 0.5);

	if(init_time < 0) {
		init_time = cur_time;
	}
	monoTrack(cur_state, cur_time, cur_img0, cur_img1, camera0, camera1, last_frame0, keyframes0, "Left");
	// monoTrack(cur_state, cur_time, cur_img1, cur_img0, camera1, camera0, last_frame1, keyframes1, "Right");
}

void StereoCamera::monoTrack(State& cur_state, double cur_time, const cv::Mat& cur_img0, const cv::Mat& cur_img1, const CameraModel& cam0, const CameraModel& cam1, 
							 Frame& last_frame, std::vector<KeyFrame>& keyframes, const std::string& window_name) {
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

	//feature tracking
	FeatureTrackingResult feature_tracking_result;
	featureTrack(lastKF, last_frame, cur_img0, cam0.K, feature_tracking_result);

	// propagate state based on feature(pnp) tracking  
	propagateState(cur_state, cur_time, lastKF, feature_tracking_result.cur_feature_pnp, cam0);

    // update last_frame
	last_frame.img = cur_img0.clone();
	last_frame.features = feature_tracking_result.cur_features;
    last_frame.feature_pnp = feature_tracking_result.cur_feature_pnp;

	if (feature_tracking_result.track_inliers>MONO_INLIER_THRES)
	{
		std::cout<<"inlier"<<feature_tracking_result.track_inliers<<std::endl;
		Pose new_cur_pose = cur_state.pose;
		cv::Mat proj_img;
		std::vector<double> errs;
		FeaturePoints lastKF_fts_pts;
		if(reconstructAndOptimize(feature_tracking_result, lastKF, cam0, cam1, new_cur_pose, lastKF_fts_pts, proj_img, errs)) {
			std::string proj_window_name = window_name + " projection";
			std::string err_window_name = window_name + " error";
			cv::imshow(proj_window_name, proj_img);
			cv::Ptr<cv::plot::Plot2d> eplot = cv::plot::Plot2d::create(errs);
			cv::Mat err_plot;
			eplot->setInvertOrientation( true );
			eplot->render(err_plot);
			cv::imshow(err_window_name, err_plot);
		    cur_state.pose = new_cur_pose;
		    cur_state.velocity = (cur_state.pose.position - lastKF.pose.position) / (cur_time - lastKF.time);
		    cur_state.showPose();
			vo_ofs << cur_time - init_time << " " << cur_state.pose.position[0] << " " << cur_state.pose.position[1] << " " << cur_state.pose.position[2] << " " 
				   << lastKF_fts_pts.points.size() << " " << errs.front() << " " << errs.back() << " " << errs.size() <<std::endl;
			gt_ofs << new_pose_msg.header.stamp.toSec() - init_time << " "
			   	   << new_pose_msg.point.x << " "
			  	   << new_pose_msg.point.y << " "
			  	   << new_pose_msg.point.z << std::endl;
			double vvn = cur_state.velocity.norm();
			double tvn = sqrt((new_pose_msg.point.x - last_pose_msg.point.x)*(new_pose_msg.point.x - last_pose_msg.point.x)  
				   			 +(new_pose_msg.point.y - last_pose_msg.point.y)*(new_pose_msg.point.y - last_pose_msg.point.y)  
				   			 +(new_pose_msg.point.z - last_pose_msg.point.z)*(new_pose_msg.point.z - last_pose_msg.point.z)) / (cur_time - lastKF.time);
			last_pose_msg = new_pose_msg;
	  	    double diff = fabs(vvn - tvn);
			if( diff > 0.2) {
			   	std::cout<<diff<<std::endl;
				// cv::waitKey();
			}
			// cv::waitKey();
		}

		cv::waitKey(1);
        // construct new keyframe
		// std::cout<<"New KeyFrame"<<std::endl;
		KeyFrame keyframe = createKeyFrame(cur_state.pose, cur_time, cur_img0, cur_img1, lastKF_fts_pts);
		std::cout<<"KeyFrame("<<keyframes.size()<<") Dropped ("<<frame_dropped_count<<") frames"<<std::endl;
	    keyframes.push_back(keyframe);
	    last_frame.img = keyframe.img0.clone(); 
	    last_frame.features = keyframe.features0;
	    last_frame.feature_pnp = keyframe.feature_points.features;

 	}
	last_time = cur_time;
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
		    cv::goodFeaturesToTrack(keyframe.img0(subRegion), subFeatures0, INIT_MAX_KP, 0.01, 10);
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

void StereoCamera::featureTrack(KeyFrame& lastKF, Frame& last_frame, const cv::Mat& cur_img, const cv::Mat& K, FeatureTrackingResult& feature_tracking_result) {
	cv::Mat line_img, line_img_pnp;
    cv::hconcat(lastKF.img0, cur_img, line_img);
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
	    	if(cv::norm(last_frame.feature_pnp[i]-last_frame_pnp_features_back[i]) > 16) continue;

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
    	if(cv::norm(last_frame.features[i]-last_frame_features_back[i]) > 16) continue;

    	lastKF_features_inlier.push_back(lastKF.features0[i]);
    	last_frame_features_inlier.push_back(last_frame.features[i]);
    	cur_features_inlier.push_back(cur_features[i]);
    	// cv::line(line_img, lastKF.features0[i], cv::Point2f(cur_features[i].x+lastKF.img0.cols, cur_features[i].y), cv::Scalar(255,0,0));
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
    Est  = cv::findEssentialMat(lastKF.features0, cur_features, K, cv::RANSAC, 0.99, 1.0, inlier_mask);
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
		// std::cout<<"brutePropagate"<<std::endl;
		cur_state.pose.position += cur_state.velocity * (cur_time - last_time);
	}

	// cur_state.showPose();
	// cv::waitKey();
}

bool StereoCamera::reconstructAndOptimize(const FeatureTrackingResult feature_result, const KeyFrame& lastKF, 
										  const CameraModel& cam0, const CameraModel& cam1,
										  Pose& cur_pose, FeaturePoints& lastKF_fts_pts, cv::Mat& proj_img, std::vector<double>& errs)
{
	assert(feature_result.lastKF_features.size() == feature_result.cur_features.size());
	// remove outlier by pose estimation
	std::vector<cv::Point2f> lastKF_features_inlier, cur_features_inlier;
    for(unsigned int i=0; i<feature_result.cur_features.size(); i++) {
        if(feature_result.inlier_mask.at<uchar>(i,0)>0) {
	    	lastKF_features_inlier.push_back(feature_result.lastKF_features[i]);
	    	cur_features_inlier.push_back(feature_result.cur_features[i]);
        }
    }

    //reconstruct 3D pts
	cv::Mat reproj = lastKF.img0.clone();
	std::vector<cv::Point3d> lastKF_pts;
	reconstruct3DPts(lastKF_features_inlier, cur_features_inlier, cam0.K, feature_result.R_lastKF2Cur, feature_result.t_lastKF2Cur, lastKF_pts, reproj);

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
	cv::Mat proj_before_opt = lastKF.img1.clone();
	projectToImg(lastKF_pts, scale, cam1, proj_before_opt);
	// std::cout<<"scale before "<<scale<<std::endl;
	if(fabs(scale)<0.01 || fabs(scale)>1 ) {
		// std::cout<<"bad scale"<<std::endl;
		scale = 0.1;
	}

	// optimization
	double new_scale = scale;
    if(!optimize(lastKF_features_inlier, lastKF_pts, new_scale, cam1, lastKF.img0, lastKF.img1, errs)) {
    	return false;
    }
    scale = new_scale;

	// final scale
	// std::cout<<"scale after "<<scale<<std::endl;
	if(fabs(scale)>1 ) {
		// std::cout<<"bad scale"<<std::1endl;
		scale = 0.1;
	}

    // show projections
	cv::Mat proj_after_opt = lastKF.img1.clone();
	projectToImg(lastKF_pts, scale, cam1, proj_after_opt);
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
	// 	projectToImg(lastKF_pts, s, cam1.K, cam0.stereo.R, cam0.stereo.t, proj_try);

	//     cv::imshow("proj try", proj_try);
	//     cv::waitKey();
	// }

	lastKF_fts_pts.features.clear();
	lastKF_fts_pts.points.clear();
	assert(lastKF_features_inlier.size() == lastKF_pts.size());
	for(int i=0; i<lastKF_features_inlier.size(); i++) {
		lastKF_fts_pts.features.push_back(lastKF_features_inlier[i]);
		lastKF_fts_pts.points.push_back(scale*cv::Point3f(lastKF_pts[i]));
	}
	

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

bool StereoCamera::optimize(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, double& scale, const CameraModel& cam, const cv::Mat& img0, const cv::Mat& img1, std::vector<double>& errs) 
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
		
		cv::Mat K = cam.K/pymd_scale;
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
    X_mat = cam.stereo.R*X_mat;

	for(int i=PYMD-1; i>=0; i--)
	{
		// std::cout<<"Prymaid "<<i<<std::endl;
		if(!optimize_pymd(fts_pymd[i], pts, X_mat, scale, img0_pymd[i], img1_pymd[i], cam.stereo, K_pymd[i], errs)) return false;
	
		// cv::imshow("err", err_plot);
		// cv::waitKey();
	}

	return true;

}

bool StereoCamera::optimize_pymd(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, const cv::Mat& X_mat, double& scale, 
								 const cv::Mat& img0, const cv::Mat& img1, const StereoModel& stereo, const cv::Mat& K, std::vector<double>& errs) 
{
	// optimize scale inverse for speed
	// double scale_inv = 1.0/scale;

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
		b_mat.block(i,0,1,WINDOW_SIZE*WINDOW_SIZE) = tmp;
    }

	// int delta = 0;
	// int ddepth = CV_16S;
	// cv::Mat grad_x, grad_y;
	// cv::Sobel( img1, grad_x, ddepth, 1, 0, 3, 1, delta, cv::BORDER_DEFAULT );
	// cv::Sobel( img1, grad_y, ddepth, 0, 1, 3, 1, delta, cv::BORDER_DEFAULT );

	// cv::Mat abs_grad_x;
	// convertScaleAbs( grad_x, abs_grad_x );
	// cv::imshow("x grad", abs_grad_x);
    CameraModel cam;
    cam.K = K;
    cam.stereo = stereo;
    double lm_damp_cur = LM_DAMP;
	double err_offset = 0.0;
    for(int opt_step=0; opt_step<MAX_OPT_STEP; opt_step++) {
    	for(int i=0; i<pts.size(); i++) {
    		double u = u_base_vec[i] + 1.0/scale * u_off_vec[i];
    		double v = v_vec[i];

			Eigen::MatrixXd f;
			getBatchAround(img1, u, v, f);

			f_b_vec(i,0) = (b_mat.block(i,0,1,WINDOW_SIZE*WINDOW_SIZE) - f).sum() + scale * scale;

			Eigen::MatrixXd aI_ap(1,2);
			aI_ap << (getPixelValue(img1, u+1.0, v)-getPixelValue(img1, u, v)), 
					 (getPixelValue(img1, u, v+1.0)-getPixelValue(img1, u, v));
		
			Eigen::MatrixXd ap_as(2,1);
			ap_as << u_off_vec[i], 0;

			Eigen::MatrixXd _grad = -1.0/scale/scale * aI_ap * ap_as;
			Jac(i,0) = _grad(0,0) + 2*scale;
	    }
	    Eigen::MatrixXd JTJ = Jac.transpose() * Jac;
	    if(JTJ.determinant() == 0) return false;

	    // std::cout<<Jac<<std::endl;
	    Eigen::MatrixXd _d_scale = (1.0 / ((JTJ + lm_damp_cur*Eigen::MatrixXd(JTJ.diagonal().asDiagonal()))(0,0)) ) * (Jac.transpose() * f_b_vec);
	    double d_scale = _d_scale(0,0);


		if(opt_step==0 && !errs.empty()) {
			err_offset = f_b_vec.norm()/pts.size() - errs.back();
		}
	    double cur_err = f_b_vec.norm()/pts.size() - err_offset;

	    // converge detection
	    // std::cout<<cur_err<<std::endl;
	    if(opt_step < MIN_OPT_STEP && !errs.empty() && cur_err > errs.back()) {
	    	// lm_damp_cur *=2;
	    	continue;
	    }
	    if(opt_step > MIN_OPT_STEP && cur_err > errs.back()){
	    	// std::cout<<"steps: "<<opt_step<<std::endl;
	    	return true;
	    } 

	    // scale_inv += d_scale_inv;
	    scale += d_scale;

	    errs.push_back(cur_err);

	    // scale = 1.0 / scale_inv;
	    if(scale > 0.5) return false;

		// cv::Mat opt = img1.clone();
		// projectToImg(pts, scale, cam, opt);
		// cv::namedWindow("opt", cv::WINDOW_NORMAL);
		// cv::imshow("opt", opt);
		// cv::waitKey();
    }
	// std::cout<<"steps: "<<MAX_OPT_STEP<<std::endl;

    return true;
}