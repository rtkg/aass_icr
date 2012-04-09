#include <ros/ros.h>
#include "dummy_server/dummy_server.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_server");

  DummyServer dummy_server;
  ROS_INFO("Dummy server ready");
  ros::spin();

  return 0;
}
//----------------------------------------------------------------------------------------
