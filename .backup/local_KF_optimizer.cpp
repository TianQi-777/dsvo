#include "stereo_processor/local_KF_optimizer.hpp"

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
    // std::cout<<"rvec"<<rvec<<std::endl<<"t"<<t<<std::endl;
}

bool LocalKFOptimizer::optimize(std::vector<KeyFrame>& keyframes, int KF_count, const CameraModel& cam0) {
	if(keyframes.size()-1 < KF_count) return false;

	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
	// linearSolver->setBlockOrdering(false);
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm(algorithm);
	optimizer.setVerbose(false);

	// oldest KF
	int base_KF_idx = keyframes.size()-1-KF_count;

	// add base_KF to optimizer, set as fixed
	KeyFrame& base_KF = keyframes[base_KF_idx];
	g2o::VertexSE3 *last_KF_v = new g2o::VertexSE3();
	last_KF_v->setId(0);
	last_KF_v->setFixed(true);
	last_KF_v->setEstimate(g2o::SE3Quat(base_KF.pose.orientation.toRotationMatrix(), base_KF.pose.position));
	optimizer.addVertex(last_KF_v);

	// add rest KF vertex, as well as edge to base KF
	std::vector<g2o::EdgeSE3M*> edges;
	for(int i=1; i<=KF_count; i++) {
		KeyFrame kf_i = keyframes[base_KF_idx + i];

		// add previous keyframe as vertex
		g2o::VertexSE3* kf_i_v = new g2o::VertexSE3();
		kf_i_v->setId(i);
		kf_i_v->setEstimate(g2o::SE3Quat(kf_i.pose.orientation.toRotationMatrix(), kf_i.pose.position));
		optimizer.addVertex(kf_i_v);
		//add edges to optimizer
		for(int j=0; j<i; j++) {
			KeyFrame& kf_j = keyframes[base_KF_idx+j];

			Eigen::Isometry3d T;
			double dist = getTransformBetweenKF(kf_i, kf_j, cam0, T);

		    g2o::EdgeSE3M* e = new g2o::EdgeSE3M();
		    e->setVertex(1, kf_i_v);
		    e->setVertex(0, optimizer.vertex(j));
		    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
        	information(0,0) = information(1,1) = information(2,2) = dist;
	        information(3,3) = information(4,4) = information(5,5) = dist;
		    e->setInformation(information);
			e->setMeasurement(T);
			// e->setMeasurementFromState();
			optimizer.addEdge(e);
			edges.push_back(e);
		}

	}

	// optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    // optimizer.save("/home/jiawei/Desktop/result_before.g2o");
    // cv::waitKey();
    optimizer.optimize( 10 );
    // optimizer.save( "/home/jiawei/Desktop/result_after.g2o" );

	// write back to KFs
	for(int i=0; i<KF_count; i++) {
		KeyFrame& kf_i = keyframes[base_KF_idx+i];

		g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(i));
		Eigen::Isometry3d pose = vertex->estimate();

		kf_i.pose.position = pose.translation();
		Eigen::Matrix3d cur_R = pose.rotation();
		Eigen::Quaterniond cur_q(cur_R);
		kf_i.pose.orientation = cur_q;
	}
	return true;

}

double LocalKFOptimizer::getTransformBetweenKF(const KeyFrame& KF_from, const KeyFrame& KF_to, const CameraModel& cam0, Eigen::Isometry3d& T) {
	if(KF_from.feature_points.size() < 10) return false; //too few points

	Eigen::Matrix3d R_from2to = cam0.R_B2C * KF_to.pose.orientation.toRotationMatrix().transpose() * KF_from.pose.orientation.toRotationMatrix() * cam0.R_C2B;
	Eigen::Vector3d t_from2to = cam0.R_B2C * KF_to.pose.orientation.toRotationMatrix().transpose()
							 * (KF_from.pose.orientation.toRotationMatrix() * cam0.t_C2B
							 -  KF_to.pose.orientation.toRotationMatrix() * cam0.t_C2B
							 +  KF_from.pose.position - KF_to.pose.position);

	// t_from2to(0) += 0.1;
	// projAndDrawPts(KF_from.feature_points.points, KF_to.img0, cam0.K, R_from2to, t_from2to, "local loop before");
	double dist = pose_estimater.poseEstimate(KF_from.feature_points, KF_from.img0, cam0.K, KF_to.img0, 4, 20, R_from2to, t_from2to);
	// projAndDrawPts(KF_from.feature_points.points, KF_to.img0, cam0.K, R_from2to, t_from2to, "local loop after");

	Eigen::Matrix3d R = cam0.R_C2B * R_from2to * cam0.R_B2C;
	Eigen::Vector3d t = cam0.R_C2B * R_from2to * cam0.t_B2C + cam0.R_C2B * t_from2to + cam0.t_C2B;
	Eigen::AngleAxisd angle(R);
	T = Eigen::Isometry3d::Identity();
	T =  angle;
    T(0,3) = t(0);
    T(1,3) = t(1);
    T(2,3) = t(2);

	return dist;
}
