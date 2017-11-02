#include "state.hpp"

StereoCamera::StereoCamera(const std::vector<double>& E0, 
						   const std::vector<double>& K0, 
						   const std::vector<double>& frame_size0, 
						   const std::vector<double>& dist_coeff0,
						   const std::vector<double>& E1, 
						   const std::vector<double>& K1, 
						   const std::vector<double>& frame_size1, 
						   const std::vector<double>& dist_coeff1) {
	cam0.E = cv::Mat(E0);
	cam0.E = cam0.E.reshape(4,4);

	cam0.K = cv::Mat::zeros(3,3,CV_64F);
	cam0.K.at<double>(0,0) = K0[0];
	cam0.K.at<double>(1,1) = K0[1];
	cam0.K.at<double>(0,2) = K0[2];
	cam0.K.at<double>(1,2) = K0[3];

	cam0.frame_size = cv::Size(frame_size0[0], frame_size0[1]);

	cam0.dist_coeff= cv::Mat(dist_coeff0);

	cam1.E = cv::Mat(E1);
	cam1.E = cam1.E.reshape(4,4);

	cam1.K = cv::Mat::zeros(3,3,CV_64F);
	cam1.K.at<double>(0,0) = K1[0];
	cam1.K.at<double>(1,1) = K1[1];
	cam1.K.at<double>(0,2) = K1[2];
	cam1.K.at<double>(1,2) = K1[3];

	cam1.frame_size = cv::Size(frame_size1[0], frame_size1[1]);

	cam1.dist_coeff= cv::Mat(dist_coeff1);
	
	last_keyframe_required = true;
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

void StereoCamera::track(const cv::Mat& cur_img0, const cv::Mat& cur_img1) {
	//initialize key_frame
    if(last_keyframe.active_features0.empty() || last_keyframe_required) {
    	last_keyframe = KeyFrame();
    	last_keyframe.img0 = cur_img0.clone();
    	last_keyframe.img1 = cur_img1.clone();

	    // detect feature points
	    cv::goodFeaturesToTrack(last_keyframe.img0, last_keyframe.features0, 500, 0.01, 5);
	    for(unsigned int i=0; i<last_keyframe.features0.size(); i++) 
	    	last_keyframe.active_features0.push_back(i);

	    last_keyframe_required = false;

    	last_frame = Frame();
	    last_frame.img0 = last_keyframe.img0; 
	    last_frame.features0 = last_keyframe.features0;

    	return;
    }

	std::vector<cv::Point2f> cur_features0;
    std::vector<uchar> status;
    std::vector<float> err;
    // std::cout<<last_frame.features0.size()<<std::endl;
    cv::calcOpticalFlowPyrLK(last_frame.img0, cur_img0, last_frame.features0, cur_features0, status, err);

    // filter matches
    std::vector<unsigned int> new_active_features0;
    cv::Mat f_img = cur_img0.clone();
    std::vector<cv::Point2f> last_frame_features0_inlier, cur_features0_inlier;
    for(int i=0; i<status.size(); i++){
    	if(!status[i] || cv::norm(last_frame.features0[i]-cur_features0[i]) > 25) continue;

    	last_frame_features0_inlier.push_back(last_frame.features0[i]);
    	cur_features0_inlier.push_back(cur_features0[i]);
    	new_active_features0.push_back(last_keyframe.active_features0[i]);
    	cv::line(f_img, last_keyframe.features0[last_keyframe.active_features0[i]], cur_features0[i], cv::Scalar(255,0,0));
    }
	cv::imshow("temp match", f_img);
    cv::waitKey(1);  

    last_keyframe.active_features0 = new_active_features0;

//	TODO
    if(true) { // not far enough 
	    // update last_frame
		last_frame.img0 = cur_img0.clone();
		last_frame.features0 = cur_features0_inlier;
	} else {	// far enough for triangulation
		//get active features of last keyframe
	    std::vector<cv::Point2f> last_keyframe_features0_inlier;
	    for(int i=0; i<last_keyframe.active_features0.size(); i++) {
	    	last_keyframe_features0_inlier.push_back(last_keyframe.features0[last_keyframe.active_features0[i]]);
	    }

	    //recover pose
	    cv::Mat inliner_mask, Est, R, t;
	    Est  = cv::findEssentialMat(last_keyframe_features0_inlier, cur_features0_inlier, cam0.K, cv::RANSAC, 0.999, 0.7, inliner_mask);
	    recoverPose(Est, last_keyframe_features0_inlier, cur_features0_inlier, cam0.K, R, t, inliner_mask);

	    // remove outliers by Essential matrix
		std::vector<cv::Point2f> last_keyframe_inlier_Est, cur_features0_inlier_Est;
	    for(unsigned int i=0; i<cur_features0_inlier.size(); i++) {
	        if(inliner_mask.at<uchar>(i,0)>0) {
	            last_keyframe_inlier_Est.push_back(last_keyframe_features0_inlier[i]);
	            cur_features0_inlier_Est.push_back(cur_features0_inlier[i]);
	        }
	    }

	    // triangulate points
	    std::vector<cv::Point3d> pts;
	    reconstruct3DPts(last_keyframe_inlier_Est, cur_features0_inlier_Est, R, t, pts);

		last_keyframe_required = true;
	}
}