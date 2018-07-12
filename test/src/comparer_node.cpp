#include "comparer.hpp"
#include "odom_comparer.hpp"
#include "point_comparer.hpp"
#include "trans_comparer.hpp"
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Comparer");
	Comparer* comparer;
	std::string type = argv[1];
	if(type == "odom")
	{
		comparer = new OdomComparer();
	} else if(type == "point") {
		comparer = new PointComparer();
	} else {
		comparer = new TransComparer();
	}
	ros::spin();
	delete comparer;
	return 0;
}
