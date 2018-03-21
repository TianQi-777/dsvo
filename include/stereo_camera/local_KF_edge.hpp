#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/base_unary_edge.h>
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include "helper.hpp"

class EdgeLocalKF:public g2o::BaseUnaryEdge<1, ScaleBatch, g2o::VertexSE3Expmap>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EdgeLocalKF(const Eigen::Vector3d& point, const Eigen::Matrix3d& K, const cv::Mat& img)
	: start_point(point), fx(K(0,0)), fy(K(1,1)), cx(K(0,2)), cy(K(1,2)), dest_img(img)
	{}

	void computeError();

	void linearizeOplus();

    virtual bool read(std::istream& /*is*/)
    {
      std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
      return false;
    }

private:

	Eigen::Vector3d start_point;
	double fx=0, fy=0, cx=0, cy=0;
	const cv::Mat& dest_img;
};

