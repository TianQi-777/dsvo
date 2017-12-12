#include "stereo_camera.hpp"

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
    cam0.K.at<double>(2,2) = 1.0;

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

void StereoCamera::track(const cv::Mat& cur_img0, const cv::Mat& cur_img1, State& cur_state) {
	//initialize key_frame
    if(last_frame.features0.empty()) {
	    last_frame.img0 = cur_img0.clone(); 
	    cv::goodFeaturesToTrack(last_frame.img0, last_frame.features0, MAX_FEATURE, 0.01, 5);
	    last_frame.state = cur_state;

    	return;
    }

	std::vector<cv::Point2f> cur_features0;
    std::vector<uchar> status;
    std::vector<float> err;
    // std::cout<<last_frame.features0.size()<<std::endl;
    cv::calcOpticalFlowPyrLK(last_frame.img0, cur_img0, last_frame.features0, cur_features0, status, err);

    // filter matches
    cv::Mat f_img = cur_img0.clone();
    std::vector<cv::Point2f> last_features0_inlier, cur_features0_inlier;
    for(int i=0; i<status.size(); i++){
    	if(!status[i] || cv::norm(last_frame.features0[i]-cur_features0[i]) > 50) continue;

    	last_features0_inlier.push_back(last_frame.features0[i]);
    	cur_features0_inlier.push_back(cur_features0[i]);
    	cv::line(f_img, last_frame.features0[i], cur_features0[i], cv::Scalar(255,0,0));
    }
	cv::imshow("temp match", f_img);
    cv::waitKey(1);  

    //recover pose
    cv::Mat inliner_mask, Est;
    Est  = cv::findEssentialMat(last_features0_inlier, cur_features0_inlier, cam0.K, cv::RANSAC, 0.999, 0.7, inliner_mask);
    // recoverPose(Est, last_features0_inlier, cur_features0_inlier, cam0.K, R, t, inliner_mask);
    // std::cout<<R<<std::endl<<t<<std::endl;

    int n = cv::sum(inliner_mask)[0];

    // normalize points: K.inv()*p
    cv::Mat U(3,n, CV_64F, 1);
    cv::Mat V(3,n, CV_64F, 1);
    int count = 0;
    for(int i=0; i<last_features0_inlier.size(); i++) {
        if(inliner_mask.at<uchar>(i,0)>0) {
            cv::Point2f u = last_features0_inlier[i];
            U.at<double>(0,count) = u.x;
            U.at<double>(1,count) = u.y;
            cv::Point2f v = cur_features0_inlier[i];
            V.at<double>(0,count) = v.x;
            V.at<double>(1,count) = v.y;
            count++;
        }
    }
    cv::Mat U_n = cam0.K.inv()*U;
    cv::Mat V_n = cam0.K.inv()*V;
    // A for essential matrix: A*E_l=0
    Eigen::MatrixXd A;
    A.resize(n,9);
    for(int i=0; i<n; i++) {
        double ux = U_n.at<double>(0,i)/U_n.at<double>(2,i);
        double uy = U_n.at<double>(1,i)/U_n.at<double>(2,i);
        double vx = V_n.at<double>(0,i)/V_n.at<double>(2,i);
        double vy = V_n.at<double>(1,i)/V_n.at<double>(2,i);
        A.row(i) << ux*vx, uy*vx, vx, ux*vy, uy*vy, vy, ux, uy, 1;
    }

    // linearlize estimated Essential matrix and its error zm
    Eigen::Matrix<double,9,1> E_l_m;
    E_l_m << Est.at<double>(0,0),Est.at<double>(0,1),Est.at<double>(0,2),
          Est.at<double>(1,0),Est.at<double>(1,1),Est.at<double>(1,2),
          Est.at<double>(2,0),Est.at<double>(2,1),Est.at<double>(2,2);
    Eigen::VectorXd zm = A*E_l_m;

    // estimation covariance
    Eigen::MatrixXd Q = zm*zm.transpose();
    Eigen::VectorXd Q_d = Q.diagonal();
    Q = Q_d.asDiagonal();

    // get relative pose R t
    Eigen::Matrix3d last_R = last_frame.state.pose.orientation.toRotationMatrix();
    Eigen::Matrix3d cur_R = cur_state.pose.orientation.toRotationMatrix();
    Eigen::Matrix3d R = cur_R*last_R.transpose();
    Eigen::Vector3d t = cur_R*(cur_state.pose.position - last_frame.state.pose.position);

    // construct essential matrix(linearlized) from state transition and its error zc
    Eigen::Matrix<double,9,1> E_c;
    E_c <<  t(2)*R(0,1) - t(1)*R(0,2),
           -t(2)*R(0,0) + t(0)*R(0,2),
            t(1)*R(0,0) - t(0)*R(0,1),
            t(2)*R(1,1) - t(1)*R(1,2),
           -t(2)*R(1,0) + t(0)*R(1,2),
            t(1)*R(1,0) - t(0)*R(1,1),
            t(2)*R(2,1) - t(1)*R(2,2),
           -t(2)*R(2,0) + t(0)*R(2,2),
            t(1)*R(2,0) - t(0)*R(2,1);
    Eigen::VectorXd zc = A*E_c;

    // a(E_c) / a(t)
    Eigen::Matrix3d r_skew;
    r_skew << 0      , -R(0,2), R(0,1) ,
              R(0,2) , 0      , -R(0,0),
              -R(0,1), R(0,0) , 0      ;          
    Eigen::Matrix<double,9,3> aE_c_at;
    aE_c_at.block<3,3>(0,0) = r_skew;
    aE_c_at.block<3,3>(3,0) = r_skew;
    aE_c_at.block<3,3>(6,0) = r_skew;

    // a(zc) / a(position)
    Eigen::MatrixXd azc_ap = A * aE_c_at * cur_R;

    // a(E_c) / a(R)
    Eigen::Matrix3d t_skew;
    t_skew << 0    , t(2) , -t(1),
              -t(2), 0    , t(0) ,
              t(1) , -t(0), 0    ; 
    Eigen::Matrix<double,9,9> aE_c_aR;
    aE_c_aR.block<3,3>(0,0) = t_skew;
    aE_c_aR.block<3,3>(3,3) = t_skew;
    aE_c_aR.block<3,3>(6,6) = t_skew;

    // a(R) / a(cur_R)
    Eigen::Matrix<double,9,9> aR_acurR;
    aR_acurR.block<3,3>(0,0) = last_R;
    aR_acurR.block<3,3>(3,3) = last_R;
    aR_acurR.block<3,3>(6,6) = last_R;

    // a(cur_R) / a(orientation)
    double w = cur_state.pose.orientation.w();
    double x = cur_state.pose.orientation.x();
    double y = cur_state.pose.orientation.y();
    double z = cur_state.pose.orientation.z();
    Eigen::Matrix<double,9,4> acurR_aq;
    acurR_aq <<    0,    0, -4*y, -4*z,
          2*z,  2*y,  2*x,  2*w,
         -2*y,  2*z, -2*w,  2*x,
         -2*z,  2*y,  2*x, -2*w,
            0, -4*x,    0, -4*z,
          2*x,  2*w,  2*z,  2*y,
          2*y,  2*z,  2*w,  2*x,
         -2*x, -2*w,  2*z,  2*y,
            0, -4*x, -4*y,    0; 

    // a(zc) / a(orientation)
    Eigen::MatrixXd azc_aq = A * aE_c_aR * aR_acurR * acurR_aq;

    // H = [(zc) / a(position) , a(zc) / a(orientation) , 0 , 0 , 0]
    Eigen::MatrixXd H;
    H.resize(n,16);
    H.setZero();
    for(int i=0; i<3; i++) {
        H.col(i) = azc_ap.col(i);
    }
    for(int i=0; i<4; i++) {
        H.col(i+3) = azc_aq.col(i);
    }

    // Kalman filter update
    Eigen::MatrixXd S = H*cur_state.covariance*H.transpose() + Q;
    Eigen::MatrixXd K = cur_state.covariance*H.transpose()*S.inverse();
    Eigen::VectorXd d_state = K*(zm - zc);
    std::cout<<S.norm()<<std::endl<<d_state<<std::endl;
    cur_state.pose.position += d_state.segment<3>(0);
    cur_state.pose.orientation.w() += d_state(3);
    cur_state.pose.orientation.x() += d_state(4);
    cur_state.pose.orientation.y() += d_state(5);
    cur_state.pose.orientation.z() += d_state(6);
    cur_state.pose.orientation.normalize();
    cur_state.velocity += d_state.segment<3>(7);
    cur_state.imu_bias.acceleration += d_state.segment<3>(10);
    cur_state.imu_bias.rotation += d_state.segment<3>(13);
    std::cout<<"before "<<cur_state.covariance.norm()<<std::endl;
    cur_state.covariance = (Eigen::Matrix<double,16,16>::Zero() - K*H)*cur_state.covariance;
    std::cout<<"after "<<cur_state.covariance.norm()<<std::endl;

    // show pose
    cur_state.showPose();
    cv::waitKey(5);

    // update last_frame
	last_frame.img0 = cur_img0.clone();
    cv::goodFeaturesToTrack(last_frame.img0, last_frame.features0, MAX_FEATURE, 0.01, 5);
    last_frame.state = cur_state;
	    
}