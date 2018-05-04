#include "stereo_camera/scale_optimizer.hpp"

bool DEBUG_SCALE = false;

double ScaleOptimizer::optimize(const std::vector<cv::Point2f>& fts, std::vector<PointWithUncertainty>& pts, double& scale,
						 const CameraModel& cam1, const cv::Mat& img0, const cv::Mat& img1, int pymd, int max_opt_step)
{
	std::vector<Eigen::Vector3d> pts_eigen;
	std::vector<cv::Point3d> pts_cv;
	std::vector<double> uncertaintys;
	for(int i=0; i<pts.size(); i++) {
		Eigen::Vector3d tmp;
		tmp << pts[i].point.x, pts[i].point.y, pts[i].point.z;
		pts_eigen.push_back(tmp);
		uncertaintys.push_back(pts[i].uncertainty);

		pts_cv.push_back(pts[i].point);
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

	double err;
	for(int i=pymd-1; i>=0; i--)
	{
		if(DEBUG_SCALE) {
			std::cout<<"current scale = "<<scale<<std::endl;
			cv::Mat proj_img = img1.clone();
			project3DPtsToImg(pts_cv, scale, cam1,proj_img);
			cv::imshow("Scale projection", proj_img);
			cv::waitKey();
		}
		err = optimize_pymd(fts_pymd[i], pts_eigen, uncertaintys, scale, img0_pymd[i], img1_pymd[i], cam1.stereo.t.at<double>(0,0), K1_pymd[i], max_opt_step);
	}

	if(DEBUG_SCALE) {
		std::cout<<"current scale = "<<scale<<std::endl;
		cv::Mat proj_img = img1.clone();
		project3DPtsToImg(pts_cv, scale, cam1,proj_img);
		cv::imshow("Scale projection", proj_img);
		cv::waitKey();
	}

	for(int i=0; i<pts.size(); i++) {
		pts[i].uncertainty = uncertaintys[i];
	}

	return err;

}

double ScaleOptimizer::optimize_pymd(const std::vector<cv::Point2f>& fts, const std::vector<Eigen::Vector3d>& pts, std::vector<double>& uncertaintys,
								   double& scale, const cv::Mat& img0, const cv::Mat& img1, double tx, const Eigen::Matrix3d& K1, int max_opt_step)
{
	// optimize scale inverse for speed
	double scale_inv = 1.0/scale;

	g2o::SparseOptimizer optimizer;
	std::unique_ptr<ScaleBlockSolver::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<ScaleLinearSolver>();
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<ScaleBlockSolver>(std::move(linearSolver)));

	optimizer.setAlgorithm(algorithm);
	optimizer.setVerbose(false);

	VertexScale* v = new VertexScale();
	v->setEstimate(scale_inv);
	v->setId(0);
	optimizer.addVertex(v);

	std::vector<EdgeScaleDirect*> edges;
	for(int i=0; i<pts.size(); i++) {
		EdgeScaleDirect* e = new EdgeScaleDirect(pts[i], K1, tx, img1);
		e->setVertex(0, v);
		ScaleBatch batch;
		getBatchAround(img0, fts[i].x, fts[i].y, batch);
		e->setMeasurement(batch);
		// e->setInformation(1.0/uncertaintys[i]*Eigen::Matrix<double,1,1>::Identity());
		e->setInformation(Eigen::Matrix<double,1,1>::Identity());
	    e->setRobustKernel( new g2o::RobustKernelHuber() );
		e->setId(i+1);

		optimizer.addEdge(e);
		edges.push_back(e);
	}

	optimizer.initializeOptimization();
	optimizer.optimize(max_opt_step);

	double total_err = 0;
	for(int i=0; i<pts.size(); i++) {
		edges[i]->computeError();
		uncertaintys[i] = edges[i]->chi2();
		total_err += uncertaintys[i];
	}

	scale_inv = v->estimate();
	scale = 1.0/scale_inv;

    return total_err / pts.size();
}
