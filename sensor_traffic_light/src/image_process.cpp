#include "TrafficState.h"

const std::string NODE_NAME = "traffic_state";

int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Image_process image_process;
  
  
  //ROS_INFO("%s is ONLINE", NODE_NAME.c_str());

  while (ros::ok()) {
    ros::spinOnce();
    image_process.process_image();
  }

  return 0;
}