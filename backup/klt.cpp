#include "stereo_processor/klt.hpp"
bool DEBUG_KLT = false;

void KLT::compute_pymd(const cv::Mat& prev_img, const cv::Mat& next_img, const std::vector<cv::Point3f>& prev_fts_d, std::vector<cv::Point2f>& next_fts,
								const cv::Mat& K, const cv::Mat& R, const cv::Mat& t,
								std::vector<uchar>& status, std::vector<float>& err)
{
	g2o::SparseOptimizer optimizer;
	std::unique_ptr<KLTBlockSolver::LinearSolverType> linearSolver;
	linearSolver = g2o::make_unique<KLTLinearSolver>();
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
	    g2o::make_unique<KLTBlockSolver>(std::move(linearSolver)));

	optimizer.setAlgorithm(algorithm);

	cv::Mat next_Ix, next_Iy;
	cv::Sobel(next_img, next_Ix, CV_32F, 1, 0, 3);
	cv::Sobel(next_img, next_Iy, CV_32F, 0, 1, 3);

	cv::Mat K_inv = K.inv();
	for(int i=0; i<status.size(); i++) {
		// remove outliers and edge points
		if(!status[i]) continue;
		cv::Point2f prev_fts = cv::Point2f(prev_fts_d[i].x, prev_fts_d[i].y);
		if(!(prev_fts.x>=KLT_BATCH_SIZE && prev_fts.x<(prev_img.cols-KLT_BATCH_SIZE) && prev_fts.y>=KLT_BATCH_SIZE && prev_fts.y<(prev_img.rows-KLT_BATCH_SIZE))) continue;
		if(!(next_fts[i].x>=KLT_BATCH_SIZE && next_fts[i].x<(next_img.cols-KLT_BATCH_SIZE) && next_fts[i].y>=KLT_BATCH_SIZE && next_fts[i].y<(next_img.rows-KLT_BATCH_SIZE))) continue;
		double d = prev_fts_d[i].z;
		if(d<0.001) continue;

		Eigen::Vector2d est;
		est << next_fts[i].x, next_fts[i].y;
		VertexKLT* v = new VertexKLT();
		v->setEstimate(est);
		v->setId(0);
		optimizer.addVertex(v);

		EdgeKLT* e = new EdgeKLT(next_img, next_Ix, next_Iy);
		e->setVertex(0, v);
		//get homography from next to prev
		cv::Mat R_t;
		cv::transpose(R, R_t);
		cv::Mat H = K*(R_t-(1.0/d)*R_t*t*(cv::Mat_<double>(1,3)<<0, 0, 1))*K_inv;
		Eigen::VectorXd batch;
		helper::getBatchAround(prev_img, prev_fts.x, prev_fts.y, batch, KLT_BATCH_SIZE);
		// helper::getBatchAroundWarp(prev_img, prev_fts.x, prev_fts.y, batch, KLT_BATCH_SIZE, H);
		e->setMeasurement(batch);
		// e->setInformation(Eigen::Matrix<double,1,1>::Identity());
		e->setInformation(Eigen::Matrix<double,1,1>::Identity());
    // e->setRobustKernel( new g2o::RobustKernelHuber() );
		e->setId(1);
		optimizer.addEdge(e);

		// optimizer.setVerbose(true);
		optimizer.initializeOptimization();
		optimizer.optimize(10);

		if(e->level()>0) status[i] = 0;
		next_fts[i].x = v->estimate()(0);
		next_fts[i].y = v->estimate()(1);
		e->computeError();
		err[i] = e->chi2();

		optimizer.clear();
	}

}

void KLT::compute(const cv::Mat& prev_img, const cv::Mat& next_img, const std::vector<cv::Point3f>& prev_fts_d, std::vector<cv::Point2f>& next_fts,
								const cv::Mat& K, const cv::Mat& R, const cv::Mat& t,
								std::vector<uchar>& status, std::vector<float>& err, int maxLevel)
{
	// create pymd
	cv::Mat tmp0 = prev_img.clone();
	cv::Mat tmp1 = next_img.clone();
	std::vector<cv::Mat> prev_img_pymd, next_img_pymd;
	std::vector<cv::Mat> K_pymd;
	std::vector<std::vector<cv::Point3f>> prev_fts_pymd;
	double pymd_scale = 1.0;
	for(int i=0; i<maxLevel; i++)
	{
		prev_img_pymd.push_back(tmp0);
		next_img_pymd.push_back(tmp1);
		cv::pyrDown(tmp0, tmp0, cv::Size(tmp0.cols/2, tmp0.rows/2));
		cv::pyrDown(tmp1, tmp1, cv::Size(tmp1.cols/2, tmp1.rows/2));

		cv::Mat K_tmp = K/pymd_scale;
		K_tmp.at<double>(2,2) = 1.0;
		K_pymd.push_back(K_tmp);

		std::vector<cv::Point3f> fts_tmp;
		for(int i=0; i<prev_fts_d.size(); i++) {
			fts_tmp.push_back(cv::Point3f(prev_fts_d[i].x/pymd_scale, prev_fts_d[i].y/pymd_scale, prev_fts_d[i].z));
		}
		prev_fts_pymd.push_back(fts_tmp);

		pymd_scale *= 2.0;
	}

	cv::Mat test = next_img.clone();
	cv::cvtColor(test, test, cv::COLOR_GRAY2BGR);
	for(int i=0; i<next_fts.size(); i++) {
	cv::circle(test, next_fts[i], 1, cv::Scalar(0,0,255));
	}
	cv::imshow("before", test);
	// set next_fts to initial pyramid
	double div = 2.0; for(int i=0; i<maxLevel; i++) div /= 2.0;
	for(int i=0; i<next_fts.size(); i++) {
		next_fts[i] = div*next_fts[i];
		status.push_back(1);
		err.push_back(0);
	}
	for(int i=maxLevel-1; i>=0; i--)
	{
		compute_pymd(prev_img_pymd[i], next_img_pymd[i], prev_fts_pymd[i], next_fts, K_pymd[i], R, t, status, err);
		if(i!=0) for(int j=0; j<next_fts.size(); j++) next_fts[j] = 2.0*next_fts[j];	// bring to higher pyramid level
	}


cv::Mat test1 = next_img.clone();
cv::cvtColor(test1, test1, cv::COLOR_GRAY2BGR);
for(int i=0; i<next_fts.size(); i++) {
	cv::circle(test1, next_fts[i], 1, cv::Scalar(0,0,255));
}
cv::imshow("after", test1);
cv::waitKey(1);
}
