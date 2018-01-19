#include <Eigen/Geometry>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/core.hpp>

namespace g2o
{
class DirectEdge:public BaseUnaryEdge<1, double, VertexSE3Expmap>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DirectEdge(Eigen::Vector3d point, Eigen::Matrix3d K, const cv::Mat& img)
	: start_point(point), fx(K(0,0)), fy(K(1,1)), cx(K(0,2)), cy(K(1,2)), dest_img(img)
	{}

	void computeError();

	void linearizeOplus();

	bool read(std::istream& in);

	bool write(std::ostream& out) const;

private:
	float getPixelValue(float x, float y);

	Eigen::Vector3d start_point;
	float fx=0, fy=0, cx=0, cy=0;
	const cv::Mat& dest_img;
};

}