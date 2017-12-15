#include "stereo_camera.hpp"

StereoCamera::StereoCamera(const std::vector<double>& T_BS0, 
						   const std::vector<double>& K0, 
						   const std::vector<double>& frame_size0, 
						   const std::vector<double>& dist_coeff0,
						   const std::vector<double>& T_BS1, 
						   const std::vector<double>& K1, 
						   const std::vector<double>& frame_size1, 
						   const std::vector<double>& dist_coeff1) {
	cam0.T_BS = cv::Mat::zeros(4,4,CV_64F);
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            cam0.T_BS.at<double>(i,j) = T_BS0[i*4+j];
        }
    }

	cam0.K = cv::Mat::zeros(3,3,CV_64F);
	cam0.K.at<double>(0,0) = K0[0];
	cam0.K.at<double>(1,1) = K0[1];
	cam0.K.at<double>(0,2) = K0[2];
	cam0.K.at<double>(1,2) = K0[3];
    cam0.K.at<double>(2,2) = 1.0;

	cam0.frame_size = cv::Size(frame_size0[0], frame_size0[1]);

	cam0.dist_coeff= cv::Mat(dist_coeff0);

	cam1.T_BS = cv::Mat::zeros(4,4,CV_64F);
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            cam1.T_BS.at<double>(i,j) = T_BS1[i*4+j];
        }
    }

	cam1.K = cv::Mat::zeros(3,3,CV_64F);
	cam1.K.at<double>(0,0) = K1[0];
	cam1.K.at<double>(1,1) = K1[1];
	cam1.K.at<double>(0,2) = K1[2];
	cam1.K.at<double>(1,2) = K1[3];

	cam1.frame_size = cv::Size(frame_size1[0], frame_size1[1]);

	cam1.dist_coeff= cv::Mat(dist_coeff1);

    cv::Mat R0 = cam0.T_BS(cv::Rect(0,0,3,3));
    cv::Mat t0 = cam0.T_BS(cv::Rect(3,0,1,3));
    cv::Mat R1 = cam1.T_BS(cv::Rect(0,0,3,3));
    cv::Mat t1 = cam1.T_BS(cv::Rect(3,0,1,3));
    cv::Mat R0_T;
    cv::transpose(R0, R0_T);
    cR = R1*R0_T;
    ct = R1*(t1-t0);
	   
    pcl_pub = nh.advertise<PointCloud> ("points2", 1);
    frame_count = 0;
}

void kp_match(const cv::Mat& img0, const cv::Mat& img1, const std::vector<cv::Point2f>& features0, const std::vector<cv::Point2f>& features1, std::vector<std::pair<int, int> >& matches) {
    std::vector<cv::KeyPoint> _kp0, _kp1;
    for( size_t i = 0; i < features0.size(); i++ ) {
        _kp0.push_back(cv::KeyPoint(features0[i], 1.f));
    }
    for( size_t i = 0; i < features1.size(); i++ ) {
        _kp1.push_back(cv::KeyPoint(features1[i], 1.f));
    }

    cv::Mat desc0, desc1;
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->compute(img0, _kp0, desc0);
    detector->compute(img1, _kp1, desc1);
    assert(_kp0.size()==desc0.rows && _kp1.size()==desc1.rows);

    std::vector<std::vector<cv::DMatch> > lr_matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->knnMatch(desc0, desc1, lr_matches, 2);

    for(unsigned int i=0; i<lr_matches.size(); i++) {
        if(float(lr_matches[i][0].distance) < 0.7*float(lr_matches[i][1].distance))
        {
            matches.push_back({lr_matches[i][0].queryIdx, lr_matches[i][0].trainIdx});
        }
    }
}

void StereoCamera::stereoMatch(const cv::Mat& img0, const cv::Mat& img1, std::vector<cv::Point2f>& kp0, std::vector<cv::Point2f>& kp1) {
    std::vector<cv::KeyPoint> _features0, _features1;
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->detect(img0, _features0);
    detector->detect(img1, _features1);
    std::vector<cv::Point2f> features0, features1;
    for(int i=0; i<_features0.size(); i++) {
        features0.push_back(_features0[i].pt);
    }
    for(int i=0; i<_features1.size(); i++) {
        features1.push_back(_features1[i].pt);
    }
    // cv::goodFeaturesToTrack(img0, features0, MAX_FEATURE, 0.01, 5);
    // cv::goodFeaturesToTrack(img1, features1, MAX_FEATURE, 0.01, 5);
    std::vector<std::pair<int, int> > matches;
    kp_match(img0, img1, features0, features1, matches);
    for(size_t i=0; i<matches.size(); i++)
    {
        double dy = features0[matches[i].first].y - features1[matches[i].second].y;
        if(dy*dy < 400.0) {
            kp0.push_back(features0[matches[i].first]);
            kp1.push_back(features1[matches[i].second]);
        }
    }
}

void StereoCamera::temporalMatch(const cv::Mat& img_kp, const std::vector<cv::Point2f>& kp, const cv::Mat& img0, const cv::Mat& img1, 
                                 std::vector<cv::Point2f>& kp0, std::vector<cv::Point2f>& kp1, std::vector<int>& kp_idx) {
    std::vector<cv::Point2f> _kp0, _kp1;
    stereoMatch(img0, img1, _kp0, _kp1);
    std::vector<std::pair<int, int> > matches;
    kp_match(img_kp, img0, kp, _kp0, matches);
    for(size_t i=0; i<matches.size(); i++) {
        kp_idx.push_back(matches[i].first);
        kp0.push_back(_kp0[matches[i].second]);
        kp1.push_back(_kp1[matches[i].second]);
    }
}

void StereoCamera::reconstruct3DPts(const std::vector<cv::Point2f>& features0, 
									   const std::vector<cv::Point2f>& features1, 
									   std::vector<cv::Point3d>& pts){
	//get perspective projection matrix
    cv::Mat Pl, Pr;
    Pl = cam0.K*(cv::Mat_<double>(3,4) <<1,0,0,0,0,1,0,0,0,0,1,0);
    cv::hconcat(cR, ct, Pr);
    Pr = cam0.K*Pr;

    PointCloud::Ptr point_cloud (new PointCloud);

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

        double cx = V.at<double>(0,3) / V.at<double>(3,3);
        double cy = V.at<double>(1,3) / V.at<double>(3,3);
        double cz = V.at<double>(2,3) / V.at<double>(3,3);

        cv::Mat pt_c = (cv::Mat_<double>(4,1) << cx, cy, cz, 1.0);
        cv::Mat pt_w = cam0.T_BS.inv() * pt_c;

        double x = pt_w.at<double>(0,0) / pt_w.at<double>(3,0);
        double y = pt_w.at<double>(1,0) / pt_w.at<double>(3,0);
        double z = pt_w.at<double>(2,0) / pt_w.at<double>(3,0);
  
  // std::cout<<cx<<" , "<<cy<<" , "<<cz<<std::endl;

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
    pcl_pub.publish(point_cloud);
}

void StereoCamera::stereoTrack(const cv::Mat& cur_img0, const cv::Mat& cur_img1, State& cur_state) {
    // initialized
    // if(!last_frame.features0.empty()) {
    if(frame_count > 5) {
        frame_count = 0;
        std::vector<int> last_pts_idx;
        std::vector<cv::Point2f> cur_kp0, cur_kp1;
        temporalMatch(last_frame.img0, last_frame.features0, cur_img0, cur_img1, cur_kp0, cur_kp1, last_pts_idx);

        // last frame 3D pts
        std::vector<cv::Point3d> last_pts;
        for(int i=0; i<last_pts_idx.size(); i++) {
            last_pts.push_back(last_frame.pts[last_pts_idx[i]]);
        }

        // current frame 3D pts
        std::vector<cv::Point3d> cur_pts;
        reconstruct3DPts(cur_kp0, cur_kp1, cur_pts);

        // Kalman filter update pt by pt
        for (int i=0; i<last_pts.size(); i++) {   
            // get relative pose R t
            Eigen::Matrix3d last_R = last_frame.state.pose.orientation.toRotationMatrix();
            Eigen::Matrix3d cur_R = cur_state.pose.orientation.toRotationMatrix();
            Eigen::Matrix3d R = cur_R*last_R.transpose();
            Eigen::Vector3d t = cur_R*(cur_state.pose.position - last_frame.state.pose.position);

            // construct pts
            double last_x = last_pts[i].x;
            double last_y = last_pts[i].y;
            double last_z = last_pts[i].z;
            Eigen::Vector3d last_pt;
            last_pt << last_x, last_y, last_z; 
            Eigen::Vector3d zm, zc; 
            zm << cur_pts[i].x, cur_pts[i].y, cur_pts[i].z;
            zc = R * last_pt + t;

            // a(zc) / a(position)
            Eigen::MatrixXd azc_ap = cur_R;

            // a(zc) / a([R])
            Eigen::Matrix<double,3,9> azc_aR;
            azc_aR << last_x, last_y, last_z, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, last_x, last_y, last_z, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, last_x, last_y, last_z;

            // a([R]) / a([cur_R])
            Eigen::Matrix<double,9,9> aR_acurR = Eigen::Matrix<double,9,9>::Zero();
            aR_acurR.block<3,3>(0,0) = last_R;
            aR_acurR.block<3,3>(3,3) = last_R;
            aR_acurR.block<3,3>(6,6) = last_R;

            // a([cur_R]) / a(q)
            double w = cur_state.pose.orientation.w();
            double x = cur_state.pose.orientation.x();
            double y = cur_state.pose.orientation.y();
            double z = cur_state.pose.orientation.z();
            Eigen::Matrix<double,9,4> acurR_aq;
            acurR_aq <<    0,    0, -4*y, -4*z,
                        -2*z,  2*y,  2*x, -2*w,
                         2*y,  2*z,  2*w,  2*x,
                         2*z,  2*y,  2*x,  2*w,
                           0, -4*x,    0, -4*z,
                        -2*x, -2*w,  2*z,  2*y,
                        -2*y,  2*z, -2*w,  2*x,
                         2*x,  2*w,  2*z,  2*y,
                           0, -4*x, -4*y,    0; 
            // acurR_aq <<    0,    0, -4*y, -4*z,
            //       2*z,  2*y,  2*x,  2*w,
            //      -2*y,  2*z, -2*w,  2*x,
            //      -2*z,  2*y,  2*x, -2*w,
            //         0, -4*x,    0, -4*z,
            //       2*x,  2*w,  2*z,  2*y,
            //       2*y,  2*z,  2*w,  2*x,
            //      -2*x, -2*w,  2*z,  2*y,
            //         0, -4*x, -4*y,    0; 

            // a(zc) / a(q)
            Eigen::MatrixXd azc_aq = azc_aR * aR_acurR * acurR_aq;

            // H = [(zc) / a(position) , a(zc) / a(orientation) , 0 , 0 , 0]
            Eigen::Matrix<double,3,16> H;
            H.setZero();
            for(int i=0; i<3; i++) {
                H.col(i) = azc_ap.col(i);
            }        
            for(int i=0; i<4; i++) {
                H.col(i+3) = azc_aq.col(i);
            }

            // estimate error
            Eigen::Vector3d Q_d;
            double dzx = zm(0) - zc(0);
            double dzy = zm(1) - zc(1);
            double dzz = zm(2) - zc(2);
            Q_d << sqrt(dzx*dzx), sqrt(dzy*dzy), sqrt(dzz*dzz);
            // Q_d << 0.01*sqrt(cur_pts[i].x * cur_pts[i].x),
            //        0.01*sqrt(cur_pts[i].y * cur_pts[i].y),
            //        0.01*sqrt(cur_pts[i].z * cur_pts[i].z);
            Eigen::Matrix3d Q = Q_d.asDiagonal();
            // Eigen::Matrix3d Q = 0.0000*Eigen::Matrix3d::Identity();


            // Kalman filter update
            Eigen::MatrixXd S = H*cur_state.covariance*H.transpose() + Q;
            S = (S + S.transpose()) / 2;
            Eigen::MatrixXd K = cur_state.covariance*H.transpose()*S.inverse();
            Eigen::VectorXd d_state = K*(zm-zc);
            std::cout<<d_state<<std::endl<<std::endl;
            // std::cout<<zm<<std::endl<<std::endl<<zc<<std::endl<<std::endl<<std::endl<<std::endl;
            cur_state.pose.position += d_state.segment<3>(0);
            cur_state.pose.orientation.w() += d_state(3);
            cur_state.pose.orientation.x() += d_state(4);
            cur_state.pose.orientation.y() += d_state(5);
            cur_state.pose.orientation.z() += d_state(6);
            cur_state.pose.orientation.normalize();
            cur_state.velocity += d_state.segment<3>(7);
            cur_state.imu_bias.acceleration += d_state.segment<3>(10);
            cur_state.imu_bias.rotation += d_state.segment<3>(13);
            cur_state.covariance = (Eigen::Matrix<double,16,16>::Identity() - K*H)*cur_state.covariance;

        }

        // show pose
        cur_state.showPose();

    }

    if(frame_count==0) {
        // update last frame
        last_frame.img0 = cur_img0.clone();

        std::vector<cv::Point2f> kp0, kp1;
        stereoMatch(cur_img0, cur_img1, kp0, kp1);
        cv::Mat stereo_match;
        cv::hconcat(cur_img0, cur_img1, stereo_match);
        for(int i=0; i<kp0.size(); i++) {
            cv::line(stereo_match, kp0[i], cv::Point2f(kp1[i].x+cur_img0.cols,kp1[i].y), cv::Scalar(255,0,0));
        }
        cv::imshow("Stereo match", stereo_match);
        cv::waitKey(1);
        last_frame.features0 = kp0;

        std::vector<cv::Point3d> pts;
        reconstruct3DPts(kp0, kp1, pts);
        last_frame.pts = pts;

        last_frame.state = cur_state;
    }

    frame_count ++;

    return;

}

void StereoCamera::track(const cv::Mat& cur_img0, const cv::Mat& cur_img1, State& cur_state) {
    // std::cout<<cur_state.covariance<<std::endl<<std::endl;
    //initialize key_frame
    if(last_keyframe.active_features0.empty() || frame_count<1) {
        last_keyframe = KeyFrame();
        last_keyframe.img0 = cur_img0.clone();
        last_keyframe.img1 = cur_img1.clone();
        last_keyframe.state = cur_state;

        // detect feature points
        cv::goodFeaturesToTrack(last_keyframe.img0, last_keyframe.features0, MAX_FEATURE, 0.01, 5);
        for(unsigned int i=0; i<last_keyframe.features0.size(); i++) 
            last_keyframe.active_features0.push_back(i);

        last_frame.img0 = cur_img0.clone();
        last_frame.features0 = last_keyframe.features0;

        frame_count++;
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
    double ave_dist = 0;
    for(int i=0; i<status.size(); i++){
        if(!status[i] || cv::norm(last_frame.features0[i]-cur_features0[i]) > 25) continue;

        last_frame_features0_inlier.push_back(last_frame.features0[i]);
        cur_features0_inlier.push_back(cur_features0[i]);
        new_active_features0.push_back(last_keyframe.active_features0[i]);
        cv::line(f_img, last_keyframe.features0[last_keyframe.active_features0[i]], cur_features0[i], cv::Scalar(255,0,0));
        double dx = last_keyframe.features0[last_keyframe.active_features0[i]].x - cur_features0[i].x;
        double dy = last_keyframe.features0[last_keyframe.active_features0[i]].y - cur_features0[i].y;
        ave_dist += sqrt(dx*dx + dy*dy);
    }
    last_keyframe.active_features0 = new_active_features0;

    ave_dist /= new_active_features0.size();
    if(frame_count < 5) { // not far enough 
        // update last_frame
        // std::cout<<cur_features0_inlier.size()<<std::endl;
        last_frame.img0 = cur_img0.clone();
        last_frame.features0 = cur_features0_inlier;    
        frame_count++;
        return;
    }

    cv::imshow("temp match", f_img);
    cv::waitKey(1);  
    //get active features of last keyframe
    std::vector<cv::Point2f> last_keyframe_features0_inlier;
    for(int i=0; i<last_keyframe.active_features0.size(); i++) {
        last_keyframe_features0_inlier.push_back(last_keyframe.features0[last_keyframe.active_features0[i]]);
    }

    //recover pose
    cv::Mat inliner_mask, Est;
    if(ave_dist<5) {
        Est = cv::Mat::zeros(3,3,CV_64F);
        inliner_mask = cv::Mat::ones(cur_features0_inlier.size(),1,CV_8U);
    } else {
        Est  = cv::findEssentialMat(last_keyframe_features0_inlier, cur_features0_inlier, cam0.K, cv::LMEDS, 0.999, 0.7, inliner_mask);
        // recoverPose(Est, last_keyframe_features0_inlier, cur_features0_inlier, cam0.K, R, t, inliner_mask);
    }
    std::cout<<Est<<std::endl;
    // cv::waitKey();

        // remove outliers by Essential matrix
    std::vector<cv::Point2f> last_keyframe_inlier_Est, cur_features0_inlier_Est;
    for(unsigned int i=0; i<cur_features0_inlier.size(); i++) {
        if(inliner_mask.at<uchar>(i,0)>0) {
            last_keyframe_inlier_Est.push_back(last_keyframe_features0_inlier[i]);
            cur_features0_inlier_Est.push_back(cur_features0_inlier[i]);
        }
    }

    int n = cv::sum(inliner_mask)[0];

    // normalize points: K.inv()*p
    cv::Mat U(3,n, CV_64F, 1);
    cv::Mat V(3,n, CV_64F, 1);
    int count = 0;
    for(int i=0; i<last_keyframe_inlier_Est.size(); i++) {
        if(inliner_mask.at<uchar>(i,0)>0) {
            cv::Point2f u = last_keyframe_inlier_Est[i];
            U.at<double>(0,count) = u.x;
            U.at<double>(1,count) = u.y;
            cv::Point2f v = cur_features0_inlier_Est[i];
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
    Eigen::Matrix3d last_R = last_keyframe.state.pose.orientation.toRotationMatrix();
    Eigen::Matrix3d cur_R = cur_state.pose.orientation.toRotationMatrix();
    Eigen::Matrix3d R = cur_R*last_R.transpose();
    Eigen::Vector3d t = cur_R*(cur_state.pose.position - last_keyframe.state.pose.position);

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
    Eigen::Matrix<double,9,3> aE_c_at = Eigen::Matrix<double,9,3>::Zero();
    aE_c_at.block<3,3>(0,0) = r_skew;
    aE_c_at.block<3,3>(3,0) = r_skew;
    aE_c_at.block<3,3>(6,0) = r_skew;

    // a(zc) / a(position)
    Eigen::MatrixXd azc_ap = A * aE_c_at * cur_R;

    // a(E_c) / a(r)
    Eigen::Matrix3d t_skew;
    t_skew << 0    , t(2) , -t(1),
              -t(2), 0    , t(0) ,
              t(1) , -t(0), 0    ; 
    Eigen::Matrix<double,9,9> aE_c_ar = Eigen::Matrix<double,9,9>::Zero();
    aE_c_ar.block<3,3>(0,0) = t_skew;
    aE_c_ar.block<3,3>(3,3) = t_skew;
    aE_c_ar.block<3,3>(6,6) = t_skew;

    // a(r) / a(cur_R)
    Eigen::Matrix<double,9,9> ar_acurR = Eigen::Matrix<double,9,9>::Zero();
    ar_acurR.block<3,3>(0,0) = last_R;
    ar_acurR.block<3,3>(3,3) = last_R;
    ar_acurR.block<3,3>(6,6) = last_R;

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
    Eigen::MatrixXd azc_aq = A * aE_c_ar * ar_acurR * acurR_aq;

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

    cur_state.showPose();
    // Kalman filter update
    Eigen::MatrixXd S = H*cur_state.covariance*H.transpose() + Q;
    Eigen::MatrixXd K = cur_state.covariance*H.transpose()*S.inverse();
    Eigen::VectorXd d_state = -K*zc;
    std::cout<<d_state<<std::endl<<std::endl;
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
    cur_state.covariance = (Eigen::Matrix<double,16,16>::Identity() - K*H)*cur_state.covariance;
    std::cout<<"after "<<cur_state.covariance.norm()<<std::endl;

    // show pose
    cur_state.showPose();

    frame_count = 0;
}
