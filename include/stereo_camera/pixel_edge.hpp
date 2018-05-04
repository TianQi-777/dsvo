#ifndef PIXEL_EDGE_HPP
#define PIXEL_EDGE_HPP

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>

#include "data.hpp"
#include "helper.hpp"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<1, 1> >  PixelBlockSolver;
typedef g2o::LinearSolverCSparse<PixelBlockSolver::PoseMatrixType> PixelLinearSolver;

class VertexPixel : public g2o::BaseVertex<1, double>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPixel()
    {
    }

    virtual bool read(std::istream& /*is*/)
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
      return false;
    }

    virtual void setToOriginImpl()
    {
      cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }

    virtual void oplusImpl(const double* update)
    {
      // cout<<_estimate<<" + "<<*update<<endl;                
      _estimate += *update;
    }
};

class PixelEdge:public g2o::BaseUnaryEdge<1, ScaleBatch, VertexPixel>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PixelEdge(double _v, const cv::Mat& img)
	: v(_v), dest_img(img)
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
  double v;
  const cv::Mat& dest_img;

};

#endif