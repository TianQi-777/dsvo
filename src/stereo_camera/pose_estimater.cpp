#include "stereo_camera/pose_estimater.hpp"

double PoseEstimater::poseEstimatePymd(const std::vector<Eigen::Vector3d>& pts, const std::vector<cv::Point2f>& fts, const cv::Mat& source_img, const cv::Mat& dest_img, const Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t, int iter){
	g2o::SparseOptimizer poseOptimizer;

	// g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>::PoseMatrixType>();
	// g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>* solver_ptr = new g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>(linearSolver);
	// g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

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
		ScaleBatch batch;
		getBatchAround(source_img,fts[i].x,fts[i].y,batch);
		edge->setMeasurement(batch);
		edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
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
	for(int i=0; i<fts_pts.points.size(); i++) {
		Eigen::Vector3d tmp;
		tmp << fts_pts.points[i].x, fts_pts.points[i].y, fts_pts.points[i].z;
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
		for(int i=0; i<fts_pts.features.size(); i++) {
			fts_tmp.push_back(cv::Point2f(fts_pts.features[i].x/pymd_scale, fts_pts.features[i].y/pymd_scale));
		}
		fts_pymd.push_back(fts_tmp);

		pymd_scale *= 2.0;
	}

	double dist = 0.0;
	for(int i=pymd-1; i>=0; i--)
	{
		// cv::Mat proj_img = dest_img.clone();
		// project3DPtsToImg(fts_pts.points, K, R, t, proj_img);
		// cv::imshow("Pose projection", proj_img);
		// cv::waitKey();
		dist = poseEstimatePymd(pts_eigen, fts_pymd[i], source_img_pymd[i], dest_img_pymd[i], K_pymd[i], R, t, iter);
	}
		// cv::Mat proj_img = dest_img.clone();
		// project3DPtsToImg(fts_pts.points, K, R, t, proj_img);
		// cv::imshow("Pose projection", proj_img);
		// cv::waitKey();

	assert(dist>0.0);
	return dist;
}