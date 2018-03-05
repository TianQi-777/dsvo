#include "stereo_camera/optimizer.hpp"

bool Optimizer::optimize(const std::vector<cv::Point2f>& fts, const std::vector<cv::Point3d>& pts, double& scale, 
						 const CameraModel& cam0, const CameraModel& cam1, const cv::Mat& img0, const cv::Mat& img1, int pymd, int max_opt_step) 
{
	std::vector<Eigen::Vector3d> pts_eigen;
	for(int i=0; i<pts.size(); i++) {
		Eigen::Vector3d tmp;
		tmp << pts[i].x, pts[i].y, pts[i].z;
		pts_eigen.push_back(tmp);
	}

	// create pymd 
	cv::Mat tmp0 = img0.clone();
	cv::Mat tmp1 = img1.clone();
	std::vector<cv::Mat> img0_pymd;
	std::vector<cv::Mat> img1_pymd;
	std::vector<Eigen::Matrix3d> K1_pymd;
	std::vector<std::vector<cv::Point2f>> fts_pymd;
	double pymd_scale = 1.0;
	for(int i=0; i<pymd; i++)
	{
		img0_pymd.push_back(tmp0);
		img1_pymd.push_back(tmp1);
		cv::pyrDown(tmp0, tmp0, cv::Size(tmp0.cols/2, tmp0.rows/2));
		cv::pyrDown(tmp1, tmp1, cv::Size(tmp1.cols/2, tmp1.rows/2));
		
		cv::Mat K1 = cam1.K/pymd_scale;
		K1.at<double>(2,2) = 1.0;
		Eigen::Matrix3d K1_eigen;
		cv::cv2eigen(K1, K1_eigen);
		K1_pymd.push_back(K1_eigen);

		std::vector<cv::Point2f> fts_tmp;
		for(int i=0; i<fts.size(); i++) {
			fts_tmp.push_back(cv::Point2f(fts[i].x/pymd_scale, fts[i].y/pymd_scale));
		}
		fts_pymd.push_back(fts_tmp);

		pymd_scale *= 2.0;
	}

	for(int i=pymd-1; i>=0; i--)
	{
		cv::Mat proj_img1 = img1.clone();
		project3DPtsToImg(pts, scale, cam1, proj_img1);
		cv::imshow("proj_img1", proj_img1);
		cv::waitKey();

		std::cout<<"Prymaid "<<i<<std::endl;
		if(!optimize_pymd(fts_pymd[i], pts_eigen, scale, img0_pymd[i], img1_pymd[i], cam0.stereo.t.at<double>(0,0), K1_pymd[i], max_opt_step)) return false;
	
		proj_img1 = img1.clone();
		project3DPtsToImg(pts, scale, cam1, proj_img1);
		cv::imshow("proj_img1", proj_img1);
		cv::waitKey();
	}

	return true;

}

bool Optimizer::optimize_pymd(const std::vector<cv::Point2f>& fts, const std::vector<Eigen::Vector3d>& pts, double& scale, 
							  const cv::Mat& img0, const cv::Mat& img1, double tx, const Eigen::Matrix3d& K1, int max_opt_step) 
{
	// optimize scale inverse for speed
	double scale_inv = 1.0/scale;

	g2o::SparseOptimizer optimizer;
	ScaleLinearSolver* linearSolver = new ScaleLinearSolver();
	ScaleBlockSolver* solver_ptr = new ScaleBlockSolver(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(true);

	VertexScale* v = new VertexScale();
	v->setEstimate(scale_inv);
	v->setId(0);
	optimizer.addVertex(v);

	for(int i=0; i<pts.size(); i++) {
		EdgeScaleDirect* e = new EdgeScaleDirect(pts[i], K1, tx, img1);
		e->setVertex(0, v);
		ScaleBatch batch;
		getBatchAround(img0, fts[i].x, fts[i].y, batch);
		e->setMeasurement(batch);
		e->setInformation(Eigen::Matrix<double,1,1>::Identity());
	    // e->setRobustKernel( new g2o::RobustKernelHuber() );
		e->setId(i+1);

		optimizer.addEdge(e);
	}

	optimizer.initializeOptimization();
	optimizer.optimize(max_opt_step);

	scale_inv = v->estimate();
	scale = 1.0/scale_inv;

    return true;
}


bool Optimizer::localLoopOptimization(const std::vector<KeyFrame>& keyframes, int KF_count, const CameraModel& cam0, const cv::Mat& cur_img0, const FeaturePoints& cur_fts_pts, Pose& cur_pose) {
	int total_KF_count = keyframes.size();
	for(int kf_i=1; kf_i<=KF_count; kf_i++) {
		if(total_KF_count<kf_i) return true;

		KeyFrame keyframe = keyframes[total_KF_count-kf_i];
		Pose KF_pose = keyframe.pose;
		
		Eigen::Matrix3d R_cur2KF = cam0.R_B2C * KF_pose.orientation.toRotationMatrix().transpose() * cur_pose.orientation.toRotationMatrix() * cam0.R_C2B;
		Eigen::Vector3d t_cur2KF = cam0.R_B2C * KF_pose.orientation.toRotationMatrix().transpose() 
								 * (cur_pose.orientation.toRotationMatrix() * cam0.t_C2B
								 -  KF_pose.orientation.toRotationMatrix() * cam0.t_C2B
								 +  cur_pose.position - KF_pose.position);

		// project current Feature points to KF
		cv::Mat R, rvec, t;
		cv::eigen2cv(R_cur2KF, R);
		cv::eigen2cv(t_cur2KF, t);
		cv::Rodrigues(R, rvec);
		std::vector<cv::Point2f> proj_pts;
	    cv::projectPoints(cur_fts_pts.points, rvec, t, cam0.K, cv::Mat::zeros(1,4,CV_64F), proj_pts);
		cv::Mat proj_img = keyframe.img0.clone();
	    cv::cvtColor(proj_img, proj_img, cv::COLOR_GRAY2BGR);
	    for(auto& p:proj_pts) {
	    	if(p.x>=0 && p.x<=proj_img.cols && p.y>=0 && p.y<=proj_img.rows)
		        cv::drawMarker(proj_img, p, cv::Scalar(0,0,255), cv::MARKER_CROSS, 4);
	    }
	    cv::imshow("local loop", proj_img);
	    // cv::waitKey(0);

	}

	return true;

}
