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
  ros::init(argc, argv, "icr_server");

  ICR::IcrServer icr_server;
  ROS_INFO("ICR server ready");
  while(ros::ok()) 
  {
    if(!strcmp(icr_server.getComputationMode().c_str(),"continuous"))
      {
	icr_server.computeSearchZones();
	icr_server.computeICR();
        icr_server.publish();
      }
    else if(!strcmp(icr_server.getComputationMode().c_str(),"step_wise"))
      {
	ROS_WARN("The service calls for step wise computation (compute_search_zones & compute_icr) are not implemented yet");
      }  
    else
      ROS_ERROR("%s is an invalid computation mode - ICR computation not possible",icr_server.getComputationMode().c_str());

    ros::spinOnce();
  }

  return 0;
}
//---------------------------------------------------------------------
