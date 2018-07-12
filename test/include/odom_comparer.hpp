#include "comparer.hpp"
#include "nav_msgs/Odometry.h"

class OdomComparer : public Comparer{
private:
	void gt_Callback(const nav_msgs::Odometry::ConstPtr& msg);
public:
	OdomComparer();
};
