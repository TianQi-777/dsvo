#include "comparer.hpp"
#include "geometry_msgs/PointStamped.h"

class PointComparer : public Comparer{
private:
	ros::Subscriber gt_sub;

	void gt_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);
	void write_vo(const Pose& pose, double p_time);

public:
	PointComparer();
};