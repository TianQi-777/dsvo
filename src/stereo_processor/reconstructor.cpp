#include "stereo_processor/reconstructor.hpp"

void Reconstructor::reconstructAndBundleAdjust(KFData& kf_data,	const cv::Mat& K, int max_opt_step, double max_reproj_dist) {

	// camera parameters
	double fx = K.at<double>(0,0);
	double fy = K.at<double>(1,1);
	double cx = K.at<double>(0,2);
	double cy = K.at<double>(1,2);

	// initial R t
	Eigen::Matrix3d R_eigen;
	Eigen::Vector3d t_eigen;
	cv::cv2eigen(kf_data.R_lastKF2Cur, R_eigen);
	cv::cv2eigen(kf_data.t_lastKF2Cur, t_eigen);

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
	if (!optimizer.addParameter(cam)) assert(false);

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
  std::vector<g2o::EdgeProjectXYZ2UV*> edge0s, edge1s;
  for ( size_t i=0; i<kf_data.size(); i++ )
  {
		double z = 1;
		double x = ( kf_data.lastKF_features[i].x - cx ) * z / fx;
		double y = ( kf_data.lastKF_features[i].y - cy ) * z / fy;
		if(!kf_data.new_pts_flags[i]){
			x = kf_data.points[i].point.x;
			y = kf_data.points[i].point.y;
			z = kf_data.points[i].point.z;
		}

    // add triangulated point
    g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
    v->setId( 2 + i );
    v->setMarginalized(true);
    v->setEstimate(Eigen::Vector3d(x,y,z));
    optimizer.addVertex( v );

    // project to feature0
    g2o::EdgeProjectXYZ2UV*  edge0 = new g2o::EdgeProjectXYZ2UV();
    edge0->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (v) );
    edge0->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (v0) );
    edge0->setMeasurement( Eigen::Vector2d(kf_data.lastKF_features[i].x, kf_data.lastKF_features[i].y ) );
		edge0->setInformation( (1.0/kf_data.points[i].uncertainty)*Eigen::Matrix2d::Identity() );
    // edge0->setInformation( Eigen::Matrix2d::Identity() );
    edge0->setParameterId(0, 0);
    edge0->setRobustKernel( new g2o::RobustKernelHuber() );
    optimizer.addEdge( edge0 );
    edge0s.push_back(edge0);

    // project to feature1
    g2o::EdgeProjectXYZ2UV*  edge1 = new g2o::EdgeProjectXYZ2UV();
    edge1->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (v) );
    edge1->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (v1) );
    edge1->setMeasurement( Eigen::Vector2d(kf_data.cur_features[i].x, kf_data.cur_features[i].y ) );
		edge1->setInformation( (1.0/kf_data.points[i].uncertainty)*Eigen::Matrix2d::Identity() );
    // edge1->setInformation( Eigen::Matrix2d::Identity() );
    edge1->setParameterId(0,0);
    edge1->setRobustKernel( new g2o::RobustKernelHuber() );
    optimizer.addEdge( edge1 );
    edge1s.push_back(edge1);
  }

  // optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(max_opt_step);

  g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
  Eigen::Isometry3d pose = v->estimate();
	double direct = t_eigen.transpose() * pose.translation()>0?1.0:-1.0;		// handle -X, -t
  R_eigen = pose.rotation();
  t_eigen = pose.translation();
  double t_norm = direct*t_eigen.norm();
  t_eigen = t_eigen / t_norm;

  //return R t
	cv::eigen2cv(R_eigen, kf_data.R_lastKF2Cur);
  cv::eigen2cv(t_eigen, kf_data.t_lastKF2Cur);

	std::vector<cv::Point3d> _pts;
  for ( size_t i=0; i<kf_data.points.size(); i++ )
  {
      g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
      Eigen::Vector3d pos = v->estimate() / t_norm;
      _pts.push_back(cv::Point3d(pos(0),pos(1),pos(2)));
  }

	// remove outlier by reproject to left img
	std::vector<cv::Point2f> _features0(kf_data.lastKF_features), _features1(kf_data.cur_features);
	std::vector<bool> _new_pts_flags(kf_data.new_pts_flags);
	kf_data.lastKF_features.clear();
	kf_data.cur_features.clear();
	kf_data.points.clear();
	kf_data.new_pts_flags.clear();
  std::vector<cv::Point2d> proj_pts0, proj_pts1;
	cv::Mat rVec;
  cv::Rodrigues(kf_data.R_lastKF2Cur, rVec);
  cv::projectPoints(_pts, cv::Mat::zeros(3,1,CV_64F), cv::Mat::zeros(3,1,CV_64F), K, cv::Mat::zeros(1,4,CV_64F), proj_pts0);
  cv::projectPoints(_pts, rVec, kf_data.t_lastKF2Cur, K, cv::Mat::zeros(1,4,CV_64F), proj_pts1);

	for(int i=0; i<_pts.size(); i++) {
    double u0 = proj_pts0[i].x;
    double v0 = proj_pts0[i].y;
    double u1 = proj_pts1[i].x;
    double v1 = proj_pts1[i].y;

		// reject outlier by reprojection
		if(!(cv::norm(_features0[i]-cv::Point2f(u0,v0)) < max_reproj_dist && cv::norm(_features1[i]-cv::Point2f(u1,v1)) < max_reproj_dist))
		 	continue;

		kf_data.lastKF_features.push_back(_features0[i]);
		kf_data.cur_features.push_back(_features1[i]);
		kf_data.points.push_back(_pts[i], cv::norm(_features0[i]-cv::Point2f(u0,v0)) + cv::norm(_features1[i]-cv::Point2f(u1,v1)));
		kf_data.new_pts_flags.push_back(_new_pts_flags[i]);
	}
}

// void Reconstructor::refinePixel(const cv::Mat& src_img, const cv::Mat& dest_img, const std::vector<cv::Point2f>& src_fts, std::vector<cv::Point2f>& dest_fts) {
// 	std::vector<uchar> status;
// 	std::vector<float> err;
// 	cv::calcOpticalFlowPyrLK(src_img, dest_img, src_fts, dest_fts, status, err);
// 	return;
//
// 	g2o::SparseOptimizer optimizer;
//   std::unique_ptr<PixelBlockSolver::LinearSolverType> linearSolver;
//   linearSolver = g2o::make_unique<PixelLinearSolver>();
//   g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
//       g2o::make_unique<PixelBlockSolver>(std::move(linearSolver)));
//
//   optimizer.setAlgorithm(algorithm);
//   optimizer.setVerbose(false);
//
//   for(int i=0; i<src_fts.size(); i++) {
//     optimizer.clear();
//
//     VertexPixel* v = new VertexPixel();
//     v->setEstimate(dest_fts[i].x);
//     v->setId(0);
//     optimizer.addVertex(v);
//
//     PixelEdge* e = new PixelEdge(src_fts[i].y, dest_img);
//     e->setVertex(0, v);
//     ScaleBatch batch;
//     getBatchAround(src_img, src_fts[i].x, src_fts[i].y, batch);
//     e->setMeasurement(batch);
//     // e->setInformation(1.0/uncertaintys[i]*Eigen::Matrix<double,1,1>::Identity());
//     e->setInformation(Eigen::Matrix<double,1,1>::Identity());
//     // e->setRobustKernel( new g2o::RobustKernelHuber() );
//     e->setId(1);
//
//     optimizer.addEdge(e);
//     optimizer.initializeOptimization();
//     optimizer.optimize(10);
//
//     if( fabs(dest_fts[i].x - v->estimate()) < 1.0) {
//     // std::cout<<dest_fts[i].x << "=" << v->estimate()<<std::endl;
//         dest_fts[i].x = v->estimate();
//         dest_fts[i].y = src_fts[i].y;
//     }
//   }
// }
