#include "../../include/PX4_realsense_bridge/PX4_realsense_bridge.h"

using namespace bridge;

int main(int argc, char** argv) {
  ros::init(argc, argv, "PX4_realsense_bridge_node");


	ros::init(argc, argv, "PX4_realsense_bridge_node");
	ros::Rate loop_rate(10);
	ros::NodeHandle nh("~");
	PX4_Realsense_Bridge Brigde(nh);




		while (ros::ok()){

			ros::spinOnce();


		}


  return 0;
}
