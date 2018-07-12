#include "comparer.hpp"
#include "geometry_msgs/PointStamped.h"

class PointComparer : public Comparer{
private:
	void gt_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);
public:
	PointComparer();
};
