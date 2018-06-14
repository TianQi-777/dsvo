#include "comparer.hpp"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

class PointComparer : public Comparer{
private:
	ros::Subscriber gt_sub;
	ros::Subscriber sptam_sub;
	std::ofstream sptam_ofs;

	void gt_Callback(const geometry_msgs::PointStamped::ConstPtr& msg);
	void sptam_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void write_vo(const Pose& pose, double p_time, bool stereo_match_flag);

public:
	PointComparer();
};
