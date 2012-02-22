#include <ros/ros.h>
#include "../include/model_server.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "model_server");

  ModelServer model_server;
  ROS_INFO("Model server ready");
  ros::spin();

  return 0;
}
//----------------------------------------------------------------------------------------
