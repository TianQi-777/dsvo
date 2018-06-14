#include "stereo_camera/g2o_edges/pose_edge.hpp"
#include <g2o/core/factory.h>

#include <iostream>
using namespace std;


void PoseEdge::computeError()
{
	const g2o::VertexSE3Expmap* vtx = static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);
	Eigen::Vector3d proj_point = vtx->estimate().map(start_point);

	double u = proj_point[0]/proj_point[2]*fx + cx;
	double v = proj_point[1]/proj_point[2]*fy + cy;


    if(( u-BATCH_SIZE/2-4 ) <0 || ( u+BATCH_SIZE/2+4 ) >dest_img.cols || (v-BATCH_SIZE/2-4 ) <0 || ( v+BATCH_SIZE/2+4 ) >dest_img.rows )
    {
        _error(0,0) = 999.0;            //TODO
        this->setLevel ( 1 );
        return;
    }
	else
	{
        ScaleBatch batch;
        helper::getBatchAround(dest_img, u, v, batch);
		_error(0,0) = (batch - _measurement).sum() / BATCH_SIZE / BATCH_SIZE;
        // cout<<"start_point"<<endl<<start_point<<endl<<"proj_point"<<endl<<proj_point<<endl<<"u"<<u<<" v "<<v<<endl<<"batch\t"<<batch<<endl<<"_meas\t"<<_measurement<<endl<<endl;
	}
}

void PoseEdge::linearizeOplus()
{
	if ( level() == 1 )
    {
        _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
        return;
    }

	const g2o::VertexSE3Expmap* vtx = static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);
	Eigen::Vector3d proj_point = vtx->estimate().map(start_point);

	double x = proj_point[0];
	double y = proj_point[1];
	double invz = 1.0 / proj_point[2];
	double invz_2 = invz*invz;

	double u = x*fx*invz + cx;
	double v = y*fy*invz + cy;

	Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

    jacobian_uv_ksai ( 0,0 ) = - x*y*invz_2 *fx;
    jacobian_uv_ksai ( 0,1 ) = ( 1+ ( x*x*invz_2 ) ) *fx;
    jacobian_uv_ksai ( 0,2 ) = - y*invz *fx;
    jacobian_uv_ksai ( 0,3 ) = invz *fx;
    jacobian_uv_ksai ( 0,4 ) = 0;
    jacobian_uv_ksai ( 0,5 ) = -x*invz_2 *fx;

    jacobian_uv_ksai ( 1,0 ) = -( 1+y*y*invz_2 ) *fy;
    jacobian_uv_ksai ( 1,1 ) = x*y*invz_2 *fy;
    jacobian_uv_ksai ( 1,2 ) = x*invz *fy;
    jacobian_uv_ksai ( 1,3 ) = 0;
    jacobian_uv_ksai ( 1,4 ) = invz *fy;
    jacobian_uv_ksai ( 1,5 ) = -y*invz_2 *fy;

    Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

    jacobian_pixel_uv ( 0,0 ) = ( helper::getPixelValue (dest_img, u+1, v )-helper::getPixelValue (dest_img, u, v ) );
    jacobian_pixel_uv ( 0,1 ) = ( helper::getPixelValue (dest_img, u, v+1 )-helper::getPixelValue (dest_img, u, v ) );

    _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;

    // if(_jacobianOplusXi.norm()<1000)
    // {
    //     _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
    //     this->setLevel ( 1 );
    // }
}
