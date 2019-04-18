#include "../../include/PX4_realsense_bridge/PX4_realsense_bridge.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace bridge {

PX4_Realsense_Bridge::PX4_Realsense_Bridge(const ros::NodeHandle& nh) :
		nh_(nh) {

	readParams();
	tf_listener_ = new tf::TransformListener(
			ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), true);

	// initialize standard subscribers
	pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>(
			"/mavros/local_position/pose", 1,
			&PX4_Realsense_Bridge::positionCallback, this);

}
;

PX4_Realsense_Bridge::~PX4_Realsense_Bridge() {

	delete tf_listener_;
}

void PX4_Realsense_Bridge::readParams() {
	nh_.getParam("input_topic", input_topic);
}

void PX4_Realsense_Bridge::positionCallback(
		const geometry_msgs::PoseStamped& msg) {
	newest_pose_ = msg;
}

}
