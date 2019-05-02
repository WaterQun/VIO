#include "../../include/PX4_realsense_bridge/PX4_realsense_bridge.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace bridge {

PX4_Realsense_Bridge::PX4_Realsense_Bridge(const ros::NodeHandle& nh)
    : nh_(nh) {
  tf_listener_ = new tf::TransformListener(
      ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), true);

  // initialize subscribers
  odom_sub_ = nh_.subscribe<const nav_msgs::Odometry&>(
      "/camera/odom/sample", 1, &PX4_Realsense_Bridge::odomCallback, this);
  // publishers
  mavros_odom_pub_ =
      nh_.advertise<nav_msgs::Odometry>("/mavros/odometry/out", 10);
};

PX4_Realsense_Bridge::~PX4_Realsense_Bridge() { delete tf_listener_; }

void PX4_Realsense_Bridge::publishTF() {
  tf::StampedTransform original_transform;
  try {
    tf_listener_->waitForTransform("/camera_odom_frame", "/camera_pose_frame",
                                   ros::Time(0), ros::Duration(2.0));
    tf_listener_->lookupTransform("/camera_odom_frame", "/camera_pose_frame",
                                  ros::Time(0), original_transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("Received an exception in TF: %s", ex.what());
  }

  tf_broadcaster_.sendTransform(tf::StampedTransform(
      original_transform.inverse(), original_transform.stamp_,
      "new_cam_pose_frame", "camera_odom_frame"));
}

void PX4_Realsense_Bridge::odomCallback(const nav_msgs::Odometry& msg) {
  mavros_odom_pub_.publish(msg);
}
}
