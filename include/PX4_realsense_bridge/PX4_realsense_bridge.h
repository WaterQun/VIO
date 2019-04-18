#ifndef PX4_REALSENSE_BRIDGE
#define PX4_REALSENSE_BRIDGE


#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/Param.h>
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Trajectory.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>  // transformPointCloud
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <boost/bind.hpp>

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace bridge {



class PX4_Realsense_Bridge {
 public:
  PX4_Realsense_Bridge(const ros::NodeHandle& nh);
  ~PX4_Realsense_Bridge();

 private:
  ros::NodeHandle nh_;

  geometry_msgs::PoseStamped newest_pose_;
  std::vector<std::string> input_topic;


  tf::TransformListener* tf_listener_;

  // Subscribers
    ros::Subscriber pose_sub_;

  void positionCallback(const geometry_msgs::PoseStamped& msg);
  void readParams();

};
}
#endif  // PX4_REALSENSE_BRIDGE
