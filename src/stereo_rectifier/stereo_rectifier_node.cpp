#include "stereo_rectifier/stereo_rectifier.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "StereoRectifier");
	StereoRectifier myStereoRectifier;
	ros::spin();
	return 0;
}
