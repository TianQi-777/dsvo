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

    frame_count = 0;
}

void dscrpt_match(const cv::Mat& dscrpt0, const cv::Mat& dscrpt1, std::vector<std::pair<int, int> >& matches) {
    if(dscrpt0.cols != dscrpt1.cols) return;
    std::vector<std::vector<cv::DMatch> > lr_matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->knnMatch(dscrpt0, dscrpt1, lr_matches, 2);

    for(unsigned int i=0; i<lr_matches.size(); i++) {
        if(float(lr_matches[i][0].distance) < 0.7*float(lr_matches[i][1].distance))
        {
            matches.push_back({lr_matches[i][0].queryIdx, lr_matches[i][0].trainIdx});
        }
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
    }
}

void StereoCamera::stereoMatch(const cv::Mat& img0, const cv::Mat& img1, std::vector<cv::Point3d>& pts, cv::Mat& kp0_dscrpt) {
    std::vector<cv::KeyPoint> _kp0, _kp1;
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    // cv::goodFeaturesToTrack(img0, features0, MAX_FEATURE, 0.01, 5);
    // cv::goodFeaturesToTrack(img1, features1, MAX_FEATURE, 0.01, 5);
    detector->detect(img0, _kp0);
    detector->detect(img1, _kp1);

    cv::Mat dscrpt0, dscrpt1;
    detector->compute(img0, _kp0, dscrpt0);
    detector->compute(img1, _kp1, dscrpt1);
    assert(_kp0.size()==dscrpt0.rows && _kp1.size()==dscrpt1.rows);

    std::vector<std::pair<int, int> > matches;
    dscrpt_match(dscrpt0, dscrpt1, matches);

    std::vector<cv::Point2f> kp0, kp1;
    kp0_dscrpt.release();
    for(size_t i=0; i<matches.size(); i++)
    {
        double dy = _kp0[matches[i].first].pt.y - _kp1[matches[i].second].pt.y;
        // if(dy*dy < 25.0) 
        {
            kp0.push_back(_kp0[matches[i].first].pt);
            kp1.push_back(_kp1[matches[i].second].pt);
            if(kp0_dscrpt.empty()) {
                kp0_dscrpt = dscrpt0.row(matches[i].first);
            } else {
                cv::vconcat(kp0_dscrpt, dscrpt0.row(matches[i].first), kp0_dscrpt);
            }
        }
    }
    
    cv::Mat stereo_match;
    cv::hconcat(img0, img1, stereo_match);
    for(int i=0; i<kp0.size(); i++) {
        cv::line(stereo_match, kp0[i], cv::Point2f(kp1[i].x+img0.cols,kp1[i].y), cv::Scalar(255,0,0));
    }
    cv::imshow("Stereo match", stereo_match);
    cv::waitKey(1);

    // construct 3D pts
    reconstruct3DPts(kp0, kp1, pts);
}

void StereoCamera::stereoTrack(const cv::Mat& cur_img0, const cv::Mat& cur_img1, State& state) {
    std::cout<<"Updating... "<<std::endl<<std::endl;
    // get current pts and descriptor
    std::vector<cv::Point3d> cur_pts;
    cv::Mat cur_dscrt;
    stereoMatch(cur_img0, cur_img1, cur_pts, cur_dscrt);
    // find correspondence
    std::vector<std::pair<int, int> > matches;
    dscrpt_match(cur_dscrt, state.landmarks_dscrt, matches);
    std::cout<<"Match size "<<matches.size()<<std::endl;

    // initalize keyframe

    // get relative pose R t
    Eigen::Vector3d cur_pos = state.pose.position;
    Eigen::Matrix3d cur_R = state.pose.orientation.toRotationMatrix();
    if(state.landmarks_dscrt.empty()) {
        std::cout<<"Adding new landmarks... "<<std::endl<<std::endl;

        // a([R]) / a(orientation)
        double w = state.pose.orientation.w();
        double x = state.pose.orientation.x();
        double y = state.pose.orientation.y();
        double z = state.pose.orientation.z();
        Eigen::Matrix<double,9,4> aR_aq;
        aR_aq <<    0,    0, -4*y, -4*z,
                 -2*z,  2*y,  2*x, -2*w,
                  2*y,  2*z,  2*w,  2*x,
                  2*z,  2*y,  2*x,  2*w,
                    0, -4*x,    0, -4*z,
                 -2*x, -2*w,  2*z,  2*y,
                 -2*y,  2*z, -2*w,  2*x,
                  2*x,  2*w,  2*z,  2*y,
                    0, -4*x, -4*y,    0; 

        // update covariance
        state.covariance.conservativeResize(16+3*cur_pts.size(), 16+3*cur_pts.size());

        for(int i=0; i<cur_pts.size(); i++) {

            // add new landmark
            Eigen::Vector3d pt_w = cur_R.transpose() * Eigen::Vector3d(cur_pts[i].x,cur_pts[i].y,cur_pts[i].z) + cur_pos;
            state.landmarks.push_back(cv::Point3d(pt_w(0),pt_w(1),pt_w(2)));
  
            if(state.landmarks_dscrt.empty()) {
                state.landmarks_dscrt = cur_dscrt.row(i);
            } else {
                cv::vconcat(state.landmarks_dscrt, cur_dscrt.row(i), state.landmarks_dscrt);
            }

            // a(pt_w) / a(position)
            Eigen::Matrix3d aptw_apos = Eigen::Matrix3d::Identity();

            // a(pt_w) / a([R])
            Eigen::Matrix<double,3,9> aptw_aR;
            aptw_aR <<cur_pts[i].x, 0, 0, cur_pts[i].y, 0, 0, cur_pts[i].z, 0, 0,
                      0, cur_pts[i].x, 0, 0, cur_pts[i].y, 0, 0, cur_pts[i].z, 0,
                      0, 0, cur_pts[i].x, 0, 0, cur_pts[i].y, 0, 0, cur_pts[i].z;

            // a(pt_w) / a(orientation)
            Eigen::MatrixXd aptw_aq = aptw_aR * aR_aq;

            // H_R = [a(pt_w) / a(position) , a(pt_w) / a(orientation) , 0 , 0 , 0, ..., 0]
            Eigen::MatrixXd H_R = Eigen::Matrix<double, 3, 16>::Zero();
            H_R.block<3,3>(0,0) = aptw_apos;
            H_R.block<3,4>(0,3) = aptw_aq;

            // H_Li = a(pt_w) / a(zm)
            Eigen::MatrixXd H_Li = cur_R.transpose();

            // estimate error
            Eigen::Vector3d R_d;
            double dcx = cur_pts[i].x;
            double dcy = cur_pts[i].y;
            double dcz = cur_pts[i].z;
            R_d << sqrt(dcx*dcx), sqrt(dcy*dcy), sqrt(dcz*dcz);
            // R_d << 0.01*sqrt(cur_pts[i].x * cur_pts[i].x),
            //        0.01*sqrt(cur_pts[i].y * cur_pts[i].y),
            //        0.01*sqrt(cur_pts[i].z * cur_pts[i].z);
            Eigen::Matrix3d R = 4*R_d.asDiagonal();

            Eigen::MatrixXd _HHP = -H_Li.transpose()*H_R*state.covariance.block(0,0,16,16+3*i);
            Eigen::MatrixXd HHPH_RH = H_Li.transpose()*(H_R*state.covariance.block<16,16>(0,0)*H_R.transpose() + R)*H_Li;
            state.covariance.block(16+3*i,0,3,16+3*i) = _HHP;
            state.covariance.block(0,16+3*i,16+3*i,3) = _HHP.transpose();
            state.covariance.block<3,3>(16+3*i,16+3*i) = HHPH_RH;
        }
        std::cout<<"Adding new landmarks end "<<std::endl<<std::endl;
    }

        
    if(matches.size() >= 4) {
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
        A.resize(3*matches.size(), 12);
        b.resize(3*matches.size());
        A.setZero();
        b.setZero();
        for (int i=0; i<matches.size(); i++) {   
            double landmark_x = state.landmarks[matches[i].second].x;
            double landmark_y = state.landmarks[matches[i].second].y;
            double landmark_z = state.landmarks[matches[i].second].z;
            A(3*i, 0) = landmark_x; A(3*i, 1) = landmark_y; A(3*i, 2) = landmark_z;
            A(3*i+1, 3) = landmark_x; A(3*i+1, 4) = landmark_y; A(3*i+1, 5) = landmark_z;
            A(3*i+2, 6) = landmark_x; A(3*i+2, 7) = landmark_y; A(3*i+2, 8) = landmark_z;
            A(3*i, 9) = 1; A(3*i+1, 10) = 1; A(3*i+2, 11) = 1;
             
            b(3*i) = cur_pts[matches[i].first].x;
            b(3*i+1) = cur_pts[matches[i].first].y;
            b(3*i+2) = cur_pts[matches[i].first].z;
        }

        Eigen::MatrixXd ATA = A.transpose()*A;
        Eigen::VectorXd ps = ATA.inverse() * A.transpose() * b;
        Eigen::Matrix3d _R;
        Eigen::Vector3d _t;
        _R << ps(0), ps(1), ps(2),
              ps(3), ps(4), ps(5),
              ps(6), ps(7), ps(8);
        _t << ps(9), ps(10), ps(11);
        Eigen::AngleAxisd newAngleAxis(_R);
        double angle = newAngleAxis.angle();
        std::cout<<"Angle "<<angle/3.14159*180.0<<std::endl;
    }

    std::vector<bool> cur_pts_updated(cur_pts.size(), false);

    Eigen::MatrixXd I;
    I.resize(16+3*state.landmarks.size(), 16+3*state.landmarks.size());
    I.setIdentity();
    // Kalman filter update pt by pt
    for (int i=0; i<matches.size(); i++) {   

        // construct pts from landmark
        double landmark_x = state.landmarks[matches[i].second].x;
        double landmark_y = state.landmarks[matches[i].second].y;
        double landmark_z = state.landmarks[matches[i].second].z;
        Eigen::Vector3d landmark_w;
        landmark_w << landmark_x, landmark_y, landmark_z; 
        Eigen::Vector3d zc = cur_R * (landmark_w - cur_pos);

        // measurement
        Eigen::Vector3d zm;
        zm << cur_pts[matches[i].first].x,
              cur_pts[matches[i].first].y,
              cur_pts[matches[i].first].z;
        cur_pts_updated[matches[i].first] = true;

        // a(zc) / a(position)
        Eigen::MatrixXd azc_ap = -cur_R;

        // a(zc) / a(landmark)
        Eigen::MatrixXd azc_almk = cur_R;

        // a(zc) / a([R])
        Eigen::Matrix<double,3,9> azc_aR;
        Eigen::Vector3d landmark_pos = landmark_w - cur_pos;
        azc_aR << landmark_pos(0), landmark_pos(1), landmark_pos(2), 0, 0, 0, 0, 0, 0,
                  0, 0, 0, landmark_pos(0), landmark_pos(1), landmark_pos(2), 0, 0, 0,
                  0, 0, 0, 0, 0, 0, landmark_pos(0), landmark_pos(1), landmark_pos(2);

        // a([R]) / a(orientation)
        double w = state.pose.orientation.w();
        double x = state.pose.orientation.x();
        double y = state.pose.orientation.y();
        double z = state.pose.orientation.z();
        Eigen::Matrix<double,9,4> aR_aq;
        aR_aq <<    0,    0, -4*y, -4*z,
                    -2*z,  2*y,  2*x, -2*w,
                     2*y,  2*z,  2*w,  2*x,
                     2*z,  2*y,  2*x,  2*w,
                       0, -4*x,    0, -4*z,
                    -2*x, -2*w,  2*z,  2*y,
                    -2*y,  2*z, -2*w,  2*x,
                     2*x,  2*w,  2*z,  2*y,
                       0, -4*x, -4*y,    0; 
        // aR_aq <<    0,    0, -4*y, -4*z,
        //       2*z,  2*y,  2*x,  2*w,
        //      -2*y,  2*z, -2*w,  2*x,
        //      -2*z,  2*y,  2*x, -2*w,
        //         0, -4*x,    0, -4*z,
        //       2*x,  2*w,  2*z,  2*y,
        //       2*y,  2*z,  2*w,  2*x,
        //      -2*x, -2*w,  2*z,  2*y,
        //         0, -4*x, -4*y,    0; 

        // a(zc) / a(orientation)
        Eigen::MatrixXd azc_aq = azc_aR * aR_aq;

        // H = [(zc) / a(position) , a(zc) / a(orientation) , 0 , 0 , 0, ..., 0, a(zc) / a(landmark), 0, ..., 0]
        Eigen::MatrixXd H;
        H.resize(3, 16+3*state.landmarks.size());
        H.setZero();
        H.block(0,0,3,3) = azc_ap;
        H.block(0,4,3,4) = azc_aq;
        H.block(0,16+3*matches[i].second,3,3) = azc_almk;

        // estimate error
        Eigen::Vector3d Q_d;
        double dzx = zm(0) - zc(0);
        double dzy = zm(1) - zc(1);
        double dzz = zm(2) - zc(2);
        Q_d << sqrt(dzx*dzx), sqrt(dzy*dzy), sqrt(dzz*dzz);
        // Q_d << 0.01*sqrt(cur_pts[i].x * cur_pts[i].x),
        //        0.01*sqrt(cur_pts[i].y * cur_pts[i].y),
        //        0.01*sqrt(cur_pts[i].z * cur_pts[i].z);
        Eigen::Matrix3d Q = 4*Q_d.asDiagonal();
        // Eigen::Matrix3d Q = 0.0000*Eigen::Matrix3d::Identity();


        // Kalman filter update
        Eigen::MatrixXd S = H*state.covariance*H.transpose() + Q;
        S = (S + S.transpose()) / 2;
        Eigen::MatrixXd K = state.covariance*H.transpose()*S.inverse();
        Eigen::VectorXd d_state = K*(zm-zc);
        // std::cout<<d_state<<std::endl<<std::endl;
        // std::cout<<zm<<std::endl<<std::endl<<zc<<std::endl<<std::endl<<std::endl<<std::endl;
        state.pose.position += d_state.segment<3>(0);
        state.pose.orientation.w() += d_state(3);
        state.pose.orientation.x() += d_state(4);
        state.pose.orientation.y() += d_state(5);
        state.pose.orientation.z() += d_state(6);
        double quat_scale = state.pose.orientation.norm();
        state.pose.orientation.normalize();
        state.velocity += d_state.segment<3>(7);
        state.imu_bias.acceleration += d_state.segment<3>(10);
        state.imu_bias.rotation += d_state.segment<3>(13);
        state.landmarks[matches[i].second].x += d_state(16+3*matches[i].second+0);
        state.landmarks[matches[i].second].y += d_state(16+3*matches[i].second+1);
        state.landmarks[matches[i].second].z += d_state(16+3*matches[i].second+2);
        state.covariance = (I - K*H)*state.covariance;
        // normalize covariance w.r.t. quatertion scale
        Eigen::MatrixXd J = I;
        J.block<4,4>(3,3) = 1.0/quat_scale*Eigen::Matrix4d::Identity();
        state.covariance = J * state.covariance * J.transpose();


    }
    std::cout<<"Updating end "<<std::endl<<std::endl;


    // show pose
    state.showPose();

    return;

}
