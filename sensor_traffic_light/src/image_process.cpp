#include "TrafficState.h"



int main(int argc, char** argv) {
  // Init
  ros::init(argc, argv, NODE_NAME);
  Image_porcess image_porcess;
  
  
  //ROS_INFO("%s is ONLINE", NODE_NAME.c_str());

  controller.start();
  while (ros::ok()) {
    ros::spinOnce();
    image_porcess.process_image();
  }

  return 0;
}