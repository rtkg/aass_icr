#include <ros/ros.h>
#include "../include/icr_server.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "icr_srv");

  IcrServer server;
  ROS_INFO("ICR server ready");
  ros::spin();

  return 0;
}
//----------------------------------------------------------------------------------------
