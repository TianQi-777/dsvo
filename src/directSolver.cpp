#include "directSolver.h"
#include <iostream>

#define PYMD 1

DirectSolver::DirectSolver()
{
	DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
	DirectBlock* solver_ptr = new DirectBlock(linearSolver);
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

	optimizer.setAlgorithm(solver);
	optimizer.setVerbose(true);
}

void DirectSolver::poseEstimate(const vector<Gray3DPoint>& points, const cv::Mat& gray_img, Eigen::Matrix3d K, Eigen::Isometry3d& Tcw, int iter_num)
{
	cv::Mat tmp = gray_img.clone();
	vector<cv::Mat> img_pymd;
	vector<Eigen::Matrix3d> K_pymd;
	int scale = 2;
	for(int i=0; i<PYMD; i++)
	{
		img_pymd.push_back(tmp);
		K_pymd.push_back(K);

		cv::pyrDown(tmp, tmp, cv::Size(tmp.cols/2, tmp.rows/2));
		K = K/double(scale);
		K(2,2) = 1.0;
		scale *= 2;
	}
	for(int p=PYMD-1; p>=0; p--)
	{
		optimizer.clear();
		g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
		pose->setEstimate(g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
		pose->setId(0);
		optimizer.addVertex(pose);

		for(int i=0; i<points.size(); i++) {
			g2o::DirectEdge* edge = new g2o::DirectEdge(points[i].pos_world, K_pymd[p], img_pymd[p]);
			edge->setVertex(0, pose);
			edge->setMeasurement(points[i].grayscale);
			edge->setInformation(Eigen::Matrix<double,1,1>::Identity());
	        edge->setRobustKernel( new g2o::RobustKernelHuber() );
			edge->setId(i+1);

			optimizer.addEdge(edge);
		}
		
		optimizer.initializeOptimization();
		optimizer.optimize(iter_num);

		Tcw = pose->estimate();

		// iter_num /= 2;
		// if(iter_num == 0) iter_num=1;
	}
}