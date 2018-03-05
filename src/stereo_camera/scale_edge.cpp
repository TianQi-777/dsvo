#include "stereo_camera/scale_edge.hpp"
#include <g2o/core/factory.h>

#include <iostream>
using namespace std;

void EdgeScaleDirect::computeError()
{
	// if(( v-BATCH_SIZE/2-4 ) <0 || ( v+BATCH_SIZE/2+4 ) >img1.rows) 
	// {
	// 	_error(0,0) = 99999.0;			//TODO
	// 	// this->setLevel ( 1 );
	// 	return;
	// }

	const VertexScale* vtx = static_cast<const VertexScale*> (_vertices[0]);
	const double& scale_inv = vtx->estimate();

	double u = u_base + scale_inv * u_inc;

	// if(( u-BATCH_SIZE/2-4 ) <0 || ( u+BATCH_SIZE/2+4 ) >img1.cols) 
	// {
	// 	_error(0,0) = 99999.0;			//TODO
	// 	// this->setLevel ( 1 );
	// 	return;
	// }

	ScaleBatch batch;
	getBatchAround(img1,u,v,batch);
	_error(0,0) = (batch - _measurement).sum();
	// cout<<"error "<<_error(0)<<endl;
	
}

void EdgeScaleDirect::linearizeOplus()
{
	// if ( level() == 1 )
 //    {
 //        _jacobianOplusXi = Eigen::Matrix<double, 1, 1>::Zero();
 //        return;
 //    }

	const VertexScale* vtx = static_cast<const VertexScale*> (_vertices[0]);
	const double& scale_inv = vtx->estimate();

	double u = u_base + scale_inv * u_inc;

	Eigen::MatrixXd aI_au(1,2);
	aI_au << (getPixelValue(img1, u+1.0, v)-getPixelValue(img1, u, v)), 
			 (getPixelValue(img1, u, v+1.0)-getPixelValue(img1, u, v));

	_jacobianOplusXi = aI_au * au_as;

    // if(_jacobianOplusXi.norm()<1000) 
    // {
    //     _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
    //     this->setLevel ( 1 );
    // }
}
