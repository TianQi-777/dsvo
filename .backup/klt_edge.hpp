#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>

#include "data.hpp"
#include "../helper.hpp"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<2, 1> >  KLTBlockSolver;
typedef g2o::LinearSolverCSparse<KLTBlockSolver::PoseMatrixType> KLTLinearSolver;

class VertexKLT : public g2o::BaseVertex<2, Eigen::Vector2d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexKLT()
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
      Eigen::Vector2d::ConstMapType v(update);
      _estimate += v;
    }
};

class EdgeKLT : public g2o::BaseUnaryEdge<1, Eigen::VectorXd, VertexKLT>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeKLT(const cv::Mat& _img1, const cv::Mat& _Ix, const cv::Mat& _Iy)
    :img1(_img1), Ix(_Ix), Iy(_Iy){}

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

    void computeError();
    // void linearizeOplus();

  private:
    const cv::Mat& img1;
    const cv::Mat& Ix;
    const cv::Mat& Iy;
};
