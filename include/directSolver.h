#include "directEdge.h"

#include "opencv2/imgproc/imgproc.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/core/robust_kernel_impl.h>
using namespace std;

struct Gray3DPoint
{
	Gray3DPoint( Eigen::Vector3d p, float g): pos_world(p), grayscale(g) {}
	Eigen::Vector3d pos_world;
	float grayscale;
};

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;

class DirectSolver
{
public:
	DirectSolver();

	void poseEstimate(const vector<Gray3DPoint>& points, const cv::Mat& gray_img, Eigen::Matrix3d K, Eigen::Isometry3d& Tcw, int iter_num);

private:
	g2o::SparseOptimizer optimizer;
};