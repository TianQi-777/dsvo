#include "comparer.hpp"
#include "geometry_msgs/TransformStamped.h"

class TransComparer : public Comparer{
private:
	void gt_Callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
public:
	TransComparer();
};
