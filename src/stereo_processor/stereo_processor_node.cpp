#include "stereo_processor/stereo_processor.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "StereoProcessor");
	StereoProcessor myStereoProcessor;
	ros::spin();
	return 0;
}
