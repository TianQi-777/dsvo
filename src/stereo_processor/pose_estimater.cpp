#include "stereo_processor/pose_estimater.hpp"
bool DEBUG_POSE = false;

double PoseEstimater::poseEstimatePymd(const std::vector<Eigen::Vector3d>& pts, const std::vector<double>& uncertainties, const std::vector<cv::Point2f>& fts,
																			 const cv::Mat& source_img, const cv::Mat& dest_img, const Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t, int iter){
	g2o::SparseOptimizer poseOptimizer;

	std::unique_ptr<g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>>(std::move(linearSolver)));

	poseOptimizer.setAlgorithm(algorithm);
	poseOptimizer.setVerbose(false);

	g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
	pose->setEstimate(g2o::SE3Quat(R, t));
	pose->setId(0);
	poseOptimizer.addVertex(pose);
	for(int i=0; i<fts.size(); i++) {
		PoseEdge* edge = new PoseEdge(pts[i], K, dest_img);
		edge->setVertex(0, pose);
		Eigen::VectorXd batch;
		helper::getBatchAround(source_img,fts[i].x,fts[i].y,batch);
		edge->setMeasurement(batch);
		// edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
		edge->setInformation((1.0/uncertainties[i])*Eigen::Matrix<double,1,1>::Identity());
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
		edge->setId(i+1);

		poseOptimizer.addEdge(edge);
	}

	poseOptimizer.initializeOptimization();
	poseOptimizer.optimize(iter);

	Eigen::Isometry3d T = pose->estimate();
	R = T.rotation();
	t = T.translation();

	return poseOptimizer.activeChi2();
}

double PoseEstimater::poseEstimate(const FeaturePoints& fts_pts, const cv::Mat& source_img, const cv::Mat& K, const cv::Mat& dest_img, int pymd, int iter, Eigen::Matrix3d& R, Eigen::Vector3d& t){
	//TODO: multi-thread
	std::vector<Eigen::Vector3d> pts_eigen;
	for(int i=0; i<fts_pts.size(); i++) {
		Eigen::Vector3d tmp;
		tmp << fts_pts[i].point.point.x, fts_pts[i].point.point.y, fts_pts[i].point.point.z;
		pts_eigen.push_back(tmp);
	}

	// create pymd
	cv::Mat tmp0 = source_img.clone();
	cv::Mat tmp1 = dest_img.clone();
	std::vector<cv::Mat> source_img_pymd;
	std::vector<cv::Mat> dest_img_pymd;
	std::vector<Eigen::Matrix3d> K_pymd;
	std::vector<std::vector<cv::Point2f>> fts_pymd;
	double pymd_scale = 1.0;
	for(int i=0; i<pymd; i++)
	{
		source_img_pymd.push_back(tmp0);
		dest_img_pymd.push_back(tmp1);
		cv::pyrDown(tmp0, tmp0, cv::Size(tmp0.cols/2, tmp0.rows/2));
		cv::pyrDown(tmp1, tmp1, cv::Size(tmp1.cols/2, tmp1.rows/2));

		cv::Mat K_tmp = K/pymd_scale;
		K_tmp.at<double>(2,2) = 1.0;
		Eigen::Matrix3d K_eigen;
		cv::cv2eigen(K_tmp, K_eigen);
		K_pymd.push_back(K_eigen);

		std::vector<cv::Point2f> fts_tmp;
		for(int i=0; i<fts_pts.size(); i++) {
			fts_tmp.push_back(cv::Point2f(fts_pts[i].feature.x/pymd_scale, fts_pts[i].feature.y/pymd_scale));
		}
		fts_pymd.push_back(fts_tmp);

		pymd_scale *= 2.0;
	}

	double dist = 0.0;
	for(int i=pymd-1; i>=0; i--)
	{
		if(DEBUG_POSE)
		{
			cv::Mat proj_img = dest_img.clone();
			helper::project3DPtsToImg(fts_pts.points(), K, R, t, proj_img);
			cv::imshow("Pose projection"+std::to_string(i), proj_img);
			cv::waitKey(1);
		}
		dist = poseEstimatePymd(pts_eigen, fts_pts.uncertainties(), fts_pymd[i], source_img_pymd[i], dest_img_pymd[i], K_pymd[i], R, t, iter);
	}
		if(DEBUG_POSE)
		{
			cv::Mat proj_img = dest_img.clone();
			helper::project3DPtsToImg(fts_pts.points(), K, R, t, proj_img);
			cv::imshow("Pose projection"+std::to_string(0), proj_img);
			cv::waitKey(1);
		}
	return dist;
}

void PoseEstimater::refine_pose(const PointsWithUncertainties& points, const std::vector<cv::Point2f>& features, const cv::Mat& K, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
	assert(points.size()==features.size());

	// set up optimizer
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

  optimizer.setAlgorithm( algorithm );
  optimizer.setVerbose( false );

  // add camera parameters
	double fx = K.at<double>(0,0);
	double fy = K.at<double>(1,1);
	double cx = K.at<double>(0,2);
	double cy = K.at<double>(1,2);
  g2o::CameraParameters* cam = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
  cam->setId(0);
	if (!optimizer.addParameter(cam)) assert(false);

  // add pose
  g2o::VertexSE3Expmap* v_pose = new g2o::VertexSE3Expmap();
  v_pose->setId(0);
  v_pose->setEstimate( g2o::SE3Quat(R, t) );
  optimizer.addVertex( v_pose );

  for ( size_t i=0; i<points.size(); i++ )
  {
      // add points
      g2o::VertexSBAPointXYZ* v_points = new g2o::VertexSBAPointXYZ();
      v_points->setId( 1 + i );
			v_points->setFixed( true );
      v_points->setMarginalized(true);
      v_points->setEstimate(Eigen::Vector3d(points[i].point.x,points[i].point.y,points[i].point.z));
      optimizer.addVertex( v_points );

      // project to features
      g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
      edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (v_points) );
      edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (v_pose)   );
      edge->setMeasurement( Eigen::Vector2d(features[i].x, features[i].y ) );
			// edge->setInformation(  Eigen::Matrix2d::Identity() );
      edge->setInformation( 1/points[i].uncertainty * Eigen::Matrix2d::Identity() );
      edge->setParameterId(0, 0);
      edge->setRobustKernel( new g2o::RobustKernelHuber() );
      optimizer.addEdge( edge );
  }

  // optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(50);				// TODO

  g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(0) );
  Eigen::Isometry3d pose = v->estimate();
  R = pose.rotation();
  t = pose.translation();
}
