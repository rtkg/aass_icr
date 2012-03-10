#include <ros/ros.h>
#include "../include/grasp_server.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_server");

  GraspServer grasp_server;
  ROS_INFO("Grasp server ready");
  ros::spin();

  return 0;
}
//----------------------------------------------------------------------------------------
