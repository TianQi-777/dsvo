#include "directEdge.h"
#include <g2o/core/factory.h>

#include <iostream>
using namespace std;

namespace g2o
{

void DirectEdge::computeError()
{
	const VertexSE3Expmap* v = static_cast<const VertexSE3Expmap*> (_vertices[0]);
	Eigen::Vector3d proj_point = v->estimate().map(start_point);

	float x = proj_point[0]/proj_point[2]*fx + cx;
	float y = proj_point[1]/proj_point[2]*fy + cy;


	if(x-4<0 || ( x+4 ) >dest_img.cols || ( y-4 ) <0 || ( y+4 ) >dest_img.rows)
	{
		_error(0,0) = 0.0;
		this->setLevel ( 1 );
	}
	else 
	{
		_error(0,0) = getPixelValue(x,y) - _measurement;
	}
}

void DirectEdge::linearizeOplus()
{
	if ( level() == 1 )
    {
        _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
        return;
    }

	const VertexSE3Expmap* vtx = static_cast<const VertexSE3Expmap*> (_vertices[0]);
	Eigen::Vector3d proj_point = vtx->estimate().map(start_point);

	float x = proj_point[0];
	float y = proj_point[1];
	float invz = 1.0 / proj_point[2];
	float invz_2 = invz*invz;

	float u = x*fx*invz + cx;
	float v = y*fy*invz + cy;

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

    jacobian_pixel_uv ( 0,0 ) = ( getPixelValue ( u+1,v )-getPixelValue ( u-1,v ) ) /2;
    jacobian_pixel_uv ( 0,1 ) = ( getPixelValue ( u,v+1 )-getPixelValue ( u,v-1 ) ) /2;

    _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;

    if(_jacobianOplusXi.norm()<1000) 
    {
        _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
        this->setLevel ( 1 );
    }
}

float DirectEdge::getPixelValue(float x, float y)
{
	int xf=int(x), yf=int(y), xc=xf+1, yc=yf+1;
	float xx = x - float(xf);
	float yy = y - float(yf);

	float res=0.0f;
	res += (1.0f-xx)*(1.0f-yy)*float(dest_img.at<uchar>(yf,xf));
	res += (1.0f-xx)*yy*float(dest_img.at<uchar>(yc,xf));
	res += xx*(1.0f-yy)*float(dest_img.at<uchar>(yf,xc));
	res += xx*yy*float(dest_img.at<uchar>(yc,xc));

	return res;
}

bool DirectEdge::read(std::istream& in)
{
	return true;
}

bool DirectEdge::write(std::ostream& out) const
{
	return true;
}

}