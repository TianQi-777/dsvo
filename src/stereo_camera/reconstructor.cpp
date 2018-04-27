#include "stereo_camera/reconstructor.hpp"

void Reconstructor::reconstructAndBundleAdjust(std::vector<cv::Point2f>& features0, std::vector<cv::Point2f>& features1, 
									const cv::Mat& K, cv::Mat& R, cv::Mat& t, int max_opt_step, int max_reproj_dist,
									std::vector<PointWithUncertainty>& pts, cv::Mat& reproj_img) {

	std::vector<cv::Point3d> pts_svd;
	// get perspective projection matrix
    cv::Mat Pl, Pr;
    Pl = K*(cv::Mat_<double>(3,4) <<1,0,0,0,0,1,0,0,0,0,1,0);
    cv::hconcat(R, t, Pr);
    Pr = K*Pr;

    // reconstruct 3d feature points
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
  
	    // cv::waitKey(0);
        pts_svd.push_back(cv::Point3d(x,y,z));
    }

    std::vector<Eigen::Vector3d> pts_svd_in;
    std::vector<cv::Point2f> features0_in, features1_in;
    std::vector<double> reproj0_errs, reproj1_errs;
    std::vector<cv::Point2d> proj_pts0, proj_pts1;
    cv::Mat rVec;
    cv::Rodrigues(R, rVec);
    cv::projectPoints(pts_svd, cv::Mat::zeros(3,1,CV_64F), cv::Mat::zeros(3,1,CV_64F), K, cv::Mat::zeros(1,4,CV_64F), proj_pts0);
    cv::projectPoints(pts_svd, rVec, t, K, cv::Mat::zeros(1,4,CV_64F), proj_pts1);
    for(int i=0; i<pts_svd.size(); i++) {
        double u0 = proj_pts0[i].x;
        double v0 = proj_pts0[i].y;
        double u1 = proj_pts1[i].x;
        double v1 = proj_pts1[i].y;

        // reject outlier by reprojection
        double reproj0_err = cv::norm(features0[i]-cv::Point2f(u0,v0));
        double reproj1_err = cv::norm(features1[i]-cv::Point2f(u1,v1));
        if( reproj0_err < max_reproj_dist &&
            reproj1_err < max_reproj_dist )  {

            Eigen::Vector3d p;
            p << pts_svd[i].x, pts_svd[i].y, pts_svd[i].z;
            pts_svd_in.push_back(p);
            features0_in.push_back(features0[i]);
            features1_in.push_back(features1[i]);
            reproj0_errs.push_back(reproj0_err);
            reproj1_errs.push_back(reproj1_err);
        }

    }

	double fx = K.at<double>(0,0);
	double fy = K.at<double>(1,1);
	double cx = K.at<double>(0,2);
	double cy = K.at<double>(1,2);
	Eigen::Matrix3d R_eigen;
	Eigen::Vector3d t_eigen;
	cv::cv2eigen(R, R_eigen);
	cv::cv2eigen(t, t_eigen);
	std::vector<cv::Point3d> _pts;

	// set up optimizer
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    optimizer.setAlgorithm( algorithm );
    optimizer.setVerbose( false );
    
    // add camera parameters
    g2o::CameraParameters* cam = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    cam->setId(0);
	if (!optimizer.addParameter(cam)) {
		assert(false);
	}
    
    // add camera0 pose
    g2o::VertexSE3Expmap* v0 = new g2o::VertexSE3Expmap();
    v0->setId(0);
    v0->setFixed( true ); 
    v0->setEstimate( g2o::SE3Quat() );
    optimizer.addVertex( v0 );

    // add camera1 pose
    g2o::VertexSE3Expmap* v1 = new g2o::VertexSE3Expmap();
    v1->setId(1);
    // v1->setFixed( true ); 
    v1->setEstimate( g2o::SE3Quat(R_eigen, t_eigen) );		// initialize with feature tracking result
    optimizer.addVertex( v1 );

    // initialize PointXYZ
    std::vector<g2o::EdgeProjectXYZ2UV*> edge0s, edge1s;
    for ( size_t i=0; i<pts_svd_in.size(); i++ )
    {
        // add triangulated point
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );
        v->setMarginalized(true);
        v->setEstimate(pts_svd_in[i]);
        optimizer.addVertex( v );

        // point uncertainty
        Eigen::Vector3d r0 = pts_svd_in[i];
        Eigen::Vector3d r1 = pts_svd_in[i] - t_eigen;
        r0 = r0 / r0.norm();
        r1 = r1 / r1.norm();
        double angle = r0.transpose() * r1;

        // project to feature0
        g2o::EdgeProjectXYZ2UV*  edge0 = new g2o::EdgeProjectXYZ2UV();
        edge0->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (v) );
        edge0->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (v0) );
        edge0->setMeasurement( Eigen::Vector2d(features0_in[i].x, features0_in[i].y ) );
        edge0->setInformation( Eigen::Matrix2d::Identity() );
        // edge0->setInformation( 1.0/reproj0_errs[i]*Eigen::Matrix2d::Identity() );
        edge0->setParameterId(0, 0);
        edge0->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge0 );
        edge0s.push_back(edge0);

        // project to feature1
        g2o::EdgeProjectXYZ2UV*  edge1 = new g2o::EdgeProjectXYZ2UV();
        edge1->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (v) );
        edge1->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (v1) );
        edge1->setMeasurement( Eigen::Vector2d(features1_in[i].x, features1_in[i].y ) );
        edge1->setInformation( Eigen::Matrix2d::Identity() );
        // edge1->setInformation( 1.0/reproj1_errs[i]*Eigen::Matrix2d::Identity() );
        edge1->setParameterId(0,0);
        edge1->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge1 );
        edge1s.push_back(edge1);
    }
    
    // optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    // optimizer.save("ba_before.g2o");
    optimizer.optimize(max_opt_step);
    // optimizer.save("ba_after.g2o");
    // cv::waitKey();

    // if(optimizer.activeChi2() < 100) {
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    R_eigen = pose.rotation();
    t_eigen = pose.translation();
    double t_norm = t_eigen.norm();
    t_eigen = t_eigen / t_norm;
    //return R t
	cv::eigen2cv(R_eigen, R);
    cv::eigen2cv(t_eigen, t);

    std::vector<cv::Point2f> _features0, _features1;
    std::vector<double> dists;
    for ( size_t i=0; i<features0_in.size(); i++ )
    {
        edge0s[i]->computeError();
        edge1s[i]->computeError();
        double dist = edge0s[i]->chi2()+edge1s[i]->chi2();
        if(dist > 0.5) {
            // std::cout<<edge0s[i]->chi2()<<" "<<edge1s[i]->chi2()<<std::endl;
            continue;
        }

        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        // std::cout<<"vertex id "<<i+2<<", pos = ";
        Eigen::Vector3d pos = v->estimate() / t_norm;
        // std::cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<std::endl;
        _features0.push_back(features0_in[i]);
        _features1.push_back(features1_in[i]);

        _pts.push_back(cv::Point3d(pos(0),pos(1),pos(2)));
        dists.push_back(dist);
    }
	    // std::cout<<"BA ratio "<<float(_pts.size())/features0_in.size()<<std::endl;

	// remove outlier by reproject to left img
	features0.clear();
	features1.clear();
	pts.clear();
	proj_pts0.clear();
    proj_pts1.clear();
    cv::cvtColor(reproj_img, reproj_img, cv::COLOR_GRAY2BGR);
    cv::Rodrigues(R, rVec);
    cv::projectPoints(_pts, cv::Mat::zeros(3,1,CV_64F), cv::Mat::zeros(3,1,CV_64F), K, cv::Mat::zeros(1,4,CV_64F), proj_pts0);
    cv::projectPoints(_pts, rVec, t, K, cv::Mat::zeros(1,4,CV_64F), proj_pts1);
    int marker_size = reproj_img.rows / 50;
	for(int i=0; i<_pts.size(); i++) {
        double u0 = proj_pts0[i].x;
        double v0 = proj_pts0[i].y;
        double u1 = proj_pts1[i].x;
        double v1 = proj_pts1[i].y;

		// reject outlier by reprojection
		if( !(0<=u0 && u0<reproj_img.cols && 0<=v0 && v0<reproj_img.rows) 
          // || cv::norm(_features0[i]-cv::Point2f(u0,v0)) > max_reproj_dist 
          || !(0<=u0 && u0<reproj_img.cols && 0<=v0 && v0<reproj_img.rows) 
          // || cv::norm(_features1[i]-cv::Point2f(u1,v1)) > max_reproj_dist
            ) 
			continue;

        cv::drawMarker(reproj_img, cv::Point2d(u0, v0), cv::Scalar(0,0,255), cv::MARKER_CROSS, marker_size);

		cv::circle(reproj_img, _features0[i], marker_size*1.2, cv::Scalar(0,255,0));

		features0.push_back(_features0[i]);
		features1.push_back(_features1[i]);
		pts.push_back(PointWithUncertainty(_pts[i], dists[i]));
	}
}