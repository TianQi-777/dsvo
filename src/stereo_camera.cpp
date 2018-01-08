#include "stereo_camera.hpp"
#define PI 3.1415926


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

	cam0.K = cv::Mat::zeros(3,3,CV_64F);
	cam0.K.at<double>(0,0) = K0[0];
	cam0.K.at<double>(1,1) = K0[1];
	cam0.K.at<double>(0,2) = K0[2];
	cam0.K.at<double>(1,2) = K0[3];

	cam0.frame_size = cv::Size(frame_size0[0], frame_size0[1]);

	cam0.dist_coeff= cv::Mat(dist_coeff0);

	cam1.E = cv::Mat(E1);
	cam1.E = cam1.E.reshape(0,4);
	cv::Mat R1 = cam1.E(cv::Rect(0,0,3,3));
	cv::Mat t1 = cam1.E(cv::Rect(3,0,1,3));

	cam1.K = cv::Mat::zeros(3,3,CV_64F);
	cam1.K.at<double>(0,0) = K1[0];
	cam1.K.at<double>(1,1) = K1[1];
	cam1.K.at<double>(0,2) = K1[2];
	cam1.K.at<double>(1,2) = K1[3];

	cam1.frame_size = cv::Size(frame_size1[0], frame_size1[1]);

	cam1.dist_coeff= cv::Mat(dist_coeff1);

	
	cv::Mat R1T;
	cv::transpose(R1, R1T);
	R01 = R1T * R0;
	t01 = R1T * (t0 - t1);

	// std::cout<<t01<<std::endl;

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("cam_pose", 1000);
	new_keyframe_required = true;
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
    // pcl_pub.publish(point_cloud);
}


float monoTrack(const cv::Mat& last_frame_img, std::vector<cv::Point2f>& last_frame_features, 
	                         const cv::Mat& last_keyframe_img, std::vector<cv::Point2f>& last_keyframe_features, 
	                         const cv::Mat& cur_img, std::vector<cv::Point2f>& cur_features, 
	                         const cv::Mat& K, cv::Mat& line_img, cv::Mat& R, cv::Mat& t) {
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
    	if(!status[i] || cv::norm(last_frame_features[i]-cur_features[i]) > 25) continue;

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
    cv::Mat inlier_mask, Est;
    Est  = cv::findEssentialMat(last_keyframe_features, cur_features, K, cv::RANSAC, 0.99, 1.0, inlier_mask);
    int inlier_count = cv::recoverPose(Est, last_keyframe_features, cur_features, K, R, t, inlier_mask);

    // std::cout<<"ratio: "<<inlier_count / float(last_keyframe_features.size())<<std::endl;
    return inlier_count / float(last_keyframe_features.size()); 

 //    for(unsigned int i=0; i<cur_features.size(); i++) {
 //        if(inlier_mask.at<uchar>(i,0)>0) {
	//     	last_keyframe_features_inlier.push_back(last_keyframe_features[i]);
	//     	last_frame_features_inlier.push_back(last_frame_features[i]);
 //        }
 //    }

 //    last_keyframe_features = last_keyframe_features_inlier; last_keyframe_features_inlier.clear();
	// last_frame_features = last_frame_features_inlier; last_frame_features_inlier.clear();
	// cur_features = cur_features_inlier;
}

KeyFrame StereoCamera::createKeyFrame(const cv::Mat& cur_img0, const cv::Mat& cur_img1, const Eigen::Matrix3d& R, const Eigen::Vector3d& t){
	std::cout<<"New KeyFrame"<<std::endl;
	KeyFrame keyframe = KeyFrame();

	keyframe.R = R;
	keyframe.t = t;

	keyframe.img0 = cur_img0.clone();
	keyframe.img1 = cur_img1.clone();

    // detect feature points
    cv::goodFeaturesToTrack(keyframe.img0, keyframe.features0, 500, 0.01, 5);
    keyframe.init_feature0_count = keyframe.features0.size();
    cv::goodFeaturesToTrack(keyframe.img1, keyframe.features1, 500, 0.01, 5);
    keyframe.init_feature1_count = keyframe.features1.size();

    // publish pose
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "/map";

    pose.pose.position.x = t[2];
    pose.pose.position.y = t[0];
    pose.pose.position.z = t[1];

    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 0;

    Eigen::Quaterniond q(R);
    pose.pose.orientation.w = q.w(); 
    pose.pose.orientation.x = q.z();
    pose.pose.orientation.y = q.x();
    pose.pose.orientation.z = q.y();

    pose_pub.publish(pose);

    return keyframe;
}

void StereoCamera::track(const cv::Mat& cur_img0, const cv::Mat& cur_img1) {
	//initialize key_frame
    if(keyframes.empty() || keyframes.back().features0.size() < 10) {
    	KeyFrame keyframe;
    	if(keyframes.empty())
	    	keyframe = createKeyFrame(cur_img0, cur_img1, Eigen::Matrix3d::Identity(), Eigen::Vector3d(0,0,0));
	    else
	    	keyframe = createKeyFrame(cur_img0, cur_img1, keyframes.back().R, keyframes.back().t);

	    last_frame.img0 = keyframe.img0.clone(); 
	    last_frame.img1 = keyframe.img1.clone(); 
	    last_frame.features0 = keyframe.features0;
	    last_frame.features1 = keyframe.features1;

	    keyframes.push_back(keyframe);
	    new_keyframe_required = false;

    	return;
    }

    KeyFrame& last_keyframe = keyframes.back();

	cv::Mat line_img0, line_img1, R0, t0, R1, t1;
    cv::Vec3d r_vec0, r_vec1;
	std::vector<cv::Point2f> cur_features0, cur_features1;
	float track0 = monoTrack(last_frame.img0, last_frame.features0, last_keyframe.img0, last_keyframe.features0, cur_img0, cur_features0, cam0.K, line_img0, R0, t0);
	float track1 = monoTrack(last_frame.img1, last_frame.features1, last_keyframe.img1, last_keyframe.features1, cur_img1, cur_features1, cam1.K, line_img1, R1, t1);
	cv::imshow("Left temp match", line_img0);
    cv::waitKey(1); 
	cv::imshow("Right temp match", line_img1);
    cv::waitKey(1); 
	// if (track0<0.3	|| !track1<0.3) {
		// R0 = cv::Mat::eye(3,3,CV_64F);
		// t0 = cv::Mat::zeros(3,1,CV_64F);
		// R1 = cv::Mat::eye(3,3,CV_64F);
		// t1 = cv::Mat::zeros(3,1,CV_64F);
		// std::cout<<"drop frame"<<std::endl;
    	// new_keyframe_required = true;
    // } else {


    // update last_frame
	last_frame.img0 = cur_img0.clone();
	last_frame.features0 = cur_features0;
	last_frame.img1 = cur_img1.clone();
	last_frame.features1 = cur_features1;

	if (track0>0.5 && track1>0.5) {
    	std::cout<<"track frame"<<std::endl;

	    cv::Rodrigues(R0, r_vec0);
	    cv::Rodrigues(R1, r_vec1);
	    if(cv::norm(r_vec0) > PI/30.0) // far enough
	    {  
			// A[lambda_0, lambda_1] = b
			cv::Mat b = R1*t01 - t01;
			cv::Mat A;
			cv::hconcat(R01*t0, -t1, A);

			cv::Mat AT, ATA, ATA_1;
			transpose(A, AT);
			ATA = AT * A;
			ATA_1 = ATA.inv();

			cv::Mat lambda = ATA_1*AT*b;

			// std::cout<<lambda.type()<<std::endl;
			t0 = lambda.at<double>(0,0) * t0;

			Eigen::Matrix3d R0_eigen;
			Eigen::Vector3d t0_eigen;
			cv::cv2eigen(R0, R0_eigen);
			cv::cv2eigen(t0, t0_eigen);
	        Eigen::Vector3d cur_t = R0_eigen*last_keyframe.t + t0_eigen;
	        Eigen::Matrix3d cur_R  = R0_eigen*last_keyframe.R;
			KeyFrame keyframe = createKeyFrame(cur_img0, cur_img1, cur_R, cur_t);
		    last_frame.img0 = keyframe.img0.clone(); 
		    last_frame.img1 = keyframe.img1.clone(); 
		    last_frame.features0 = keyframe.features0;
		    last_frame.features1 = keyframe.features1;

		    keyframes.push_back(keyframe);
		    new_keyframe_required = false;
		}
 	}
}