#include "comparer.hpp"
#include "nav_msgs/Odometry.h"

class OdomComparer : public Comparer{
private:
	ros::Subscriber gt_sub;

	void gt_Callback(const nav_msgs::Odometry::ConstPtr& msg);
	void write_vo(const Pose& pose, double p_time);

public:
	OdomComparer();
};