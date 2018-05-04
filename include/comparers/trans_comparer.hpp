#include "comparer.hpp"
#include "geometry_msgs/TransformStamped.h"

class TransComparer : public Comparer{
private:
	ros::Subscriber gt_sub;

	void gt_Callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
	void write_vo(const Pose& pose, double p_time, bool stereo_match_flag);

public:
	TransComparer();
};
