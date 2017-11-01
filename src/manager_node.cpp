#include "manager.hpp"

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "Manager");
	Manager myManager;
	ros::spin();
	return 0;
}