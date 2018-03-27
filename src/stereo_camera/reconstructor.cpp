#include "stereo_camera/reconstructor.hpp"

void Reconstructor::reconstructAndBundleAdjust(std::vector<cv::Point2f>& features0, std::vector<cv::Point2f>& features1, 
									const cv::Mat& K, cv::Mat& R, cv::Mat& t, int max_reproj_dist,
									std::vector<cv::Point3d>& pts, cv::Mat& reproj_img) {


	std::vector<Eigen::Vector3d> pts_svd;
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
	    Eigen::Vector3d p;
	    p << x, y, z;
        pts_svd.push_back(p);
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
    // g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
    // g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
    // g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );
    
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
    v1->setEstimate( g2o::SE3Quat(R_eigen, t_eigen) );		// initialize with feature tracking result
    optimizer.addVertex( v1 );

    // initialize PointXYZ
    std::vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts_svd.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );
        v->setMarginalized(true);
        v->setEstimate(pts_svd[i]);
        optimizer.addVertex( v );

        // project to feature0
        g2o::EdgeProjectXYZ2UV*  edge0 = new g2o::EdgeProjectXYZ2UV();
        edge0->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (v) );
        edge0->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (v0) );
        edge0->setMeasurement( Eigen::Vector2d(features0[i].x, features0[i].y ) );
        edge0->setInformation( Eigen::Matrix2d::Identity() );
        edge0->setParameterId(0, 0);
        edge0->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge0 );
        edges.push_back(edge0);

        // project to feature1
        g2o::EdgeProjectXYZ2UV*  edge1 = new g2o::EdgeProjectXYZ2UV();
        edge1->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (v) );
        edge1->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (v1) );
        edge1->setMeasurement( Eigen::Vector2d(features1[i].x, features1[i].y ) );
        edge1->setInformation( Eigen::Matrix2d::Identity() );
        edge1->setParameterId(0,0);
        edge1->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge1 );
        edges.push_back(edge1);
    }
    
    // optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    // if(optimizer.activeChi2() < 100) {
	    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
	    Eigen::Isometry3d pose = v->estimate();
	    R_eigen = pose.rotation();
	    t_eigen = pose.translation();
	    double t_norm = t_eigen.norm();
	    t_eigen = t_eigen / t_norm;
    	cv::eigen2cv(R_eigen, R);
	    cv::eigen2cv(t_eigen, t);
    
	    for ( size_t i=0; i<features0.size(); i++ )
	    {
	        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
	        // std::cout<<"vertex id "<<i+2<<", pos = ";
	        Eigen::Vector3d pos = v->estimate() / t_norm;
	        // std::cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<std::endl;

	        _pts.push_back(cv::Point3d(pos(0),pos(1),pos(2)));
	    }
	    
	    // // 估计inlier的个数
	    // int inliers = 0;
	    // for ( auto e:edges )
	    // {
	    //     e->computeError();
	    //     // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
	    //     if ( e->chi2() > 1 )
	    //     {
	    //         std::cout<<"error = "<<e->chi2()<<std::endl;
	    //     }
	    //     else 
	    //     {
	    //         inliers++;
	    //     }
	    // }
	    
	    // std::cout<<"inliers in total points: "<<inliers<<"/"<<features0.size()+features1.size()<<std::endl;
	    // optimizer.save("ba.g2o");

    // } else 
    // {
    // }

	// remove outlier by reproject to left img
	std::vector<cv::Point2f> _features0 = features0;
	std::vector<cv::Point2f> _features1 = features1;
	features0.clear();
	features1.clear();
	pts.clear();
	std::vector<cv::Point2d> proj_pts0, proj_pts1;
    cv::cvtColor(reproj_img, reproj_img, cv::COLOR_GRAY2BGR);
    cv::Mat rVec;
    cv::Rodrigues(R, rVec);
    cv::projectPoints(_pts, cv::Mat::zeros(3,1,CV_64F), cv::Mat::zeros(3,1,CV_64F), K, cv::Mat::zeros(1,4,CV_64F), proj_pts0);
    cv::projectPoints(_pts, rVec, t, K, cv::Mat::zeros(1,4,CV_64F), proj_pts1);
	for(int i=0; i<_pts.size(); i++) {
        double u0 = proj_pts0[i].x;
        double v0 = proj_pts0[i].y;
        double u1 = proj_pts1[i].x;
        double v1 = proj_pts1[i].y;

		// reject outlier by reprojection
		if( !(0<=u0 && u0<reproj_img.cols && 0<=v0 && v0<reproj_img.rows) || 
			cv::norm(_features0[i]-cv::Point2f(u0,v0)) > max_reproj_dist || 
		    !(0<=u0 && u0<reproj_img.cols && 0<=v0 && v0<reproj_img.rows) || 
			cv::norm(_features1[i]-cv::Point2f(u1,v1)) > max_reproj_dist) 
			continue;

        cv::drawMarker(reproj_img, cv::Point2d(u0, v0), cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);

		cv::circle(reproj_img, _features0[i], 5, cv::Scalar(0,255,0));

		features0.push_back(_features0[i]);
		features1.push_back(_features1[i]);
		pts.push_back(_pts[i]);
	}
}