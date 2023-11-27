#include <ros/ros.h>
#include "augment_gridmap_online/AugmentedGridMap.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "augment_gridmap_online_node");
	ros::NodeHandle nodeHandle;

	ros_augmented_gridmaps::AugmentedGridMap rosAgmentedGridmap(nodeHandle);

	ros::spin();
	return 0;
	}
