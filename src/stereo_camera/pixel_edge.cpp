#include "stereo_camera/pixel_edge.hpp"
#include <g2o/core/factory.h>

#include <iostream>
#include <stdlib.h> 


void PixelEdge::computeError()
{
	const VertexPixel* vtx = static_cast<const VertexPixel*> (_vertices[0]);
	double u = vtx->estimate();


    if(( u-BATCH_SIZE/2-4 ) <0 || ( u+BATCH_SIZE/2+4 ) >dest_img.cols || (v-BATCH_SIZE/2-4 ) <0 || ( v+BATCH_SIZE/2+4 ) >dest_img.rows )
    {
        _error(0,0) = 999.0;            //TODO
        this->setLevel ( 1 );
        return;
    }
	else 
	{
        ScaleBatch dest_batch;
        getBatchAround(dest_img, u, v, dest_batch);
		_error(0,0) = (dest_batch - _measurement).sum() / BATCH_SIZE / BATCH_SIZE;
        // std::cout<<_error(0,0)<<std::endl;
        // cout<<"start_point"<<endl<<start_point<<endl<<"proj_point"<<endl<<proj_point<<endl<<"u"<<u<<" v "<<v<<endl<<"batch\t"<<batch<<endl<<"_meas\t"<<_measurement<<endl<<endl;
	}
}

void PixelEdge::linearizeOplus()
{
	if ( level() == 1 )
    {
        _error(0,0) = 0.0;
        _jacobianOplusXi = (double(std::rand()) / RAND_MAX - 0.5) * Eigen::Matrix<double, 1, 1>::Identity();
        return;
    }

    const VertexPixel* vtx = static_cast<const VertexPixel*> (_vertices[0]);
    double u = vtx->estimate();

    Eigen::Matrix<double, 1, 1> jacobian_pixel_u;

    jacobian_pixel_u ( 0,0 ) = ( getPixelValue (dest_img, u+1, v )-getPixelValue (dest_img, u, v ) );

    _jacobianOplusXi = jacobian_pixel_u;

    if(_jacobianOplusXi.norm() < 1.0e-10) 
    {
        _error(0,0) = 0.0;
        _jacobianOplusXi = (double(std::rand()) / RAND_MAX - 0.5) * Eigen::Matrix<double, 1, 1>::Identity();
        this->setLevel ( 1 );
    }
}
