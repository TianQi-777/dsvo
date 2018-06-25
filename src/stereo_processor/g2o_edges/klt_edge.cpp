#include "stereo_processor/g2o_edges/klt_edge.hpp"
#include <g2o/core/factory.h>

#include <iostream>
using namespace std;

void EdgeKLT::computeError()
{
	const VertexKLT* vtx = static_cast<const VertexKLT*> (_vertices[0]);
	const double& u = vtx->estimate()(0);
	const double& v = vtx->estimate()(1);
std::cout<<"computeError "<<u<<" - "<<v<<std::endl;
	if(( u-KLT_BATCH_SIZE/2-4 ) <0 || ( u+KLT_BATCH_SIZE/2+4 ) >img1.cols || ( v-KLT_BATCH_SIZE/2-4 ) <0 || ( v+KLT_BATCH_SIZE/2+4 ) >img1.rows)
	{
		_error(0,0) = 1.0;			//TODO
		this->setLevel ( 1 );
		return;
	}

	Eigen::VectorXd batch;
	helper::getBatchAround(img1,u,v,batch,KLT_BATCH_SIZE);
	// cout<<"batch= "<<batch<<endl<<" _measurement= "<<_measurement<<endl;
	_error = batch - _measurement;
	// cout<<"error "<<_error(0)<<endl;

	std::cout<<"computeError done"<<std::endl;
}

void EdgeKLT::linearizeOplus()
{
	if ( level() == 1 )
  {
      _jacobianOplusXi = Eigen::Matrix<double, 1, 2>::Zero();
      return;
  }

	const VertexKLT* vtx = static_cast<const VertexKLT*> (_vertices[0]);
	const double& u = vtx->estimate()(0);
	const double& v = vtx->estimate()(1);
std::cout<<"linearizeOplus "<<u<<" - "<<v<<std::endl;

  // _jacobianOplusXi ( 0,0 ) = helper::getPixelValue (Ix, u, v );
  // _jacobianOplusXi ( 0,1 ) = helper::getPixelValue (Iy, u, v );

	Eigen::VectorXd Ix_batch, Iy_batch;
	helper::getBatchAround(Ix,u,v,Ix_batch,KLT_BATCH_SIZE);
	helper::getBatchAround(Iy,u,v,Iy_batch,KLT_BATCH_SIZE);

	_jacobianOplusXi << Ix_batch, Iy_batch;
	std::cout<<"linearizeOplus done"<<std::endl;
}
