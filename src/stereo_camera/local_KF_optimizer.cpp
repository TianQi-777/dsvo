#include "stereo_camera/local_KF_optimizer.hpp"
#define PYMD 3

void poseEstimatePymd(const std::vector<Eigen::Vector3d>& pts, const std::vector<cv::Point2f>& fts, const cv::Mat& source_img, const cv::Mat& dest_img, const Eigen::Matrix3d& K, Eigen::Matrix3d& R, Eigen::Vector3d& t){
	g2o::SparseOptimizer poseOptimizer;

	// g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>::PoseMatrixType>();
	// g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>* solver_ptr = new g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>(linearSolver);
	// g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	std::unique_ptr<g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>::PoseMatrixType>>();
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<g2o::BlockSolver<g2o::BlockSolverTraits<6,1>>>(std::move(linearSolver)));

	poseOptimizer.setAlgorithm(algorithm);
	poseOptimizer.setVerbose(false);

	g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
	pose->setEstimate(g2o::SE3Quat(R, t));
	pose->setId(0);
	poseOptimizer.addVertex(pose);

	for(int i=0; i<fts.size(); i++) {
		EdgeLocalKF* edge = new EdgeLocalKF(pts[i], K, dest_img);
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
	poseOptimizer.optimize(50);

	Eigen::Isometry3d T = pose->estimate();
	R = T.rotation();
	t = T.translation();
}

void poseEstimate(const FeaturePoints& fts_pts, const cv::Mat& source_img, const cv::Mat& K, const cv::Mat& dest_img, Eigen::Matrix3d& R, Eigen::Vector3d& t){
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
	for(int i=0; i<PYMD; i++)
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

	for(int i=PYMD-1; i>=0; i--)
	{
		poseEstimatePymd(pts_eigen, fts_pymd[i], source_img_pymd[i], dest_img_pymd[i], K_pymd[i], R, t);
	}
}

void projAndDrawPts(const std::vector<cv::Point3f>& points, const cv::Mat& img, const cv::Mat& K, const Eigen::Matrix3d& R_cur2KF, const Eigen::Vector3d& t_cur2KF, const std::string& window_name) {
	cv::Mat proj_img = img.clone();
	cv::Mat R, rvec, t;
	cv::eigen2cv(R_cur2KF, R);
	cv::eigen2cv(t_cur2KF, t);
	cv::Rodrigues(R, rvec);
	std::vector<cv::Point2f> proj_pts;
    cv::projectPoints(points, rvec, t, K, cv::Mat::zeros(1,4,CV_64F), proj_pts);
    cv::cvtColor(proj_img, proj_img, cv::COLOR_GRAY2BGR);
    for(auto& p:proj_pts) {
    	if(p.x>=0 && p.x<=proj_img.cols && p.y>=0 && p.y<=proj_img.rows)
	        cv::drawMarker(proj_img, p, cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
    }
    cv::imshow(window_name, proj_img);
    // cv::waitKey(0);
    std::cout<<"rvec"<<rvec<<std::endl<<"t"<<t<<std::endl;
}

bool LocalKFOptimizer::optimize(const std::vector<KeyFrame>& keyframes, int KF_count, const CameraModel& cam0, const cv::Mat& cur_img0, const FeaturePoints& cur_fts_pts, Pose& cur_pose) {
	// g2o::LinearSolverCSparse< g2o::BlockSolver_6_3::PoseMatrixType >* linearSolver = new g2o::LinearSolverCSparse< g2o::BlockSolver_6_3::PoseMatrixType >();
	// linearSolver->setBlockOrdering(false);
	// g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3(linearSolver);
	// g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
	
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
	// linearSolver->setBlockOrdering(false);
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));


	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(algorithm);
	optimizer.setVerbose(false);

	//add cur_pose to optimizer
	g2o::VertexSE3 *cur_pose_v = new g2o::VertexSE3();
	cur_pose_v->setId(0);
	cur_pose_v->setEstimate(g2o::SE3Quat(cur_pose.orientation.toRotationMatrix(), cur_pose.position));
	optimizer.addVertex(cur_pose_v);
	
	int total_KF_count = keyframes.size();
	for(int kf_i=1; kf_i<=KF_count; kf_i++) {
	// for(int kf_i=7; kf_i<=7; kf_i++) {
		if(total_KF_count<kf_i) break;

		KeyFrame keyframe = keyframes[total_KF_count-kf_i];
		Pose KF_pose = keyframe.pose;

		Eigen::Matrix3d R_cur2KF = cam0.R_B2C * KF_pose.orientation.toRotationMatrix().transpose() * cur_pose.orientation.toRotationMatrix() * cam0.R_C2B;
		Eigen::Vector3d t_cur2KF = cam0.R_B2C * KF_pose.orientation.toRotationMatrix().transpose() 
								 * (cur_pose.orientation.toRotationMatrix() * cam0.t_C2B
								 -  KF_pose.orientation.toRotationMatrix() * cam0.t_C2B
								 +  cur_pose.position - KF_pose.position);

		// t_cur2KF(0) += 0.07;

		// projAndDrawPts(cur_fts_pts.points, keyframe.img0, cam0.K, R_cur2KF, t_cur2KF, "local loop before");
		poseEstimate(cur_fts_pts, cur_img0, cam0.K, keyframe.img0, R_cur2KF, t_cur2KF);
		// projAndDrawPts(cur_fts_pts.points, keyframe.img0, cam0.K, R_cur2KF, t_cur2KF, "local loop after");
	
		Eigen::Matrix3d R = cam0.R_C2B * R_cur2KF * cam0.R_B2C;
		Eigen::Vector3d t = cam0.R_C2B * R_cur2KF * cam0.t_B2C + cam0.R_C2B * t_cur2KF + cam0.t_C2B;

		//add keyframe to optimizer
		g2o::VertexSE3 *KF_v = new g2o::VertexSE3();
		KF_v->setId(kf_i);
		KF_v->setFixed(true);				// TODO
		KF_v->setEstimate(g2o::SE3Quat(KF_pose.orientation.toRotationMatrix(), KF_pose.position));
		optimizer.addVertex(KF_v);

		g2o::EdgeSE3* edge = new g2o::EdgeSE3();
		edge->setVertex( 0, cur_pose_v);
		edge->setVertex( 1, KF_v);
		edge->setRobustKernel( new g2o::RobustKernelHuber() );
		edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
		edge->setMeasurement(g2o::SE3Quat(R, t));

		optimizer.addEdge(edge);

	}
	
    optimizer.save("/home/kimiwings/Desktop/result_before.g2o");
    optimizer.initializeOptimization();
    optimizer.optimize( 100 ); 
    optimizer.save( "/home/kimiwings/Desktop/result_after.g2o" );

	g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(0));
	Eigen::Isometry3d pose = vertex->estimate();

	// write back to cur_pose
	// cur_pose.position = pose.translation();
	// Eigen::Matrix3d cur_R = pose.rotation();
	// Eigen::Quaterniond cur_q(cur_R);
	// cur_pose.orientation = cur_q;

	return true;

}
