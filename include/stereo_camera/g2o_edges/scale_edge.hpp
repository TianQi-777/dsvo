#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>

#include "../data.hpp"
#include "../helper.hpp"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<1, 1> >  ScaleBlockSolver;
typedef g2o::LinearSolverCSparse<ScaleBlockSolver::PoseMatrixType> ScaleLinearSolver;

class VertexScale : public g2o::BaseVertex<1, double>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexScale()
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

class EdgeScaleDirect : public g2o::BaseUnaryEdge<1, ScaleBatch, VertexScale>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeScaleDirect(const Eigen::Vector3d& point3d, const Eigen::Matrix3d& K, double tx, const cv::Mat& _img1)
    : img1(_img1)
    {
      Eigen::Vector3d KX = K * point3d;

      u_base = KX(0)/KX(2);
      v = KX(1)/KX(2);
      u_inc = tx*K(0,0)/point3d(2);
      au_as << u_inc, 0;
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

    void computeError();
    void linearizeOplus();

  private:

    double u_base;
    double v;
    double u_inc;
    Eigen::Matrix<double,2,1> au_as;
    const cv::Mat& img1;
};
