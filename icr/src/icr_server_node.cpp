/** \file icr_server_node.cpp 
 * \author Robert Krug 
 * \brief Just an implementation of icr_server_node. 
 * Read \ref IcrServer for more details.
 */
#include <ros/ros.h>
#include "../include/icr_server.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////
//---------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "icr_srv");

  IcrServer server;
  ROS_INFO("ICR server ready");
  // while(ros::ok()) {
  //   server.publishCloud();
  //   ros::spinOnce();
  // }
  ros::spin();
  return 0;
}
//---------------------------------------------------------------------
