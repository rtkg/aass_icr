/**
 * @author Robert Krug
 * @date   Sat Apr 7 2012
 *
 */

#include <ros/ros.h>
#include "grasp_affordances/grasp_affordances.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////

//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_affordances");

  GraspAffordances grasp_affordances;
  ROS_INFO("Grasp Affordances node ready");

  while(ros::ok())
    {
      grasp_affordances.publish();
      ros::spinOnce();
    }

  return 0;
}
//----------------------------------------------------------------------------------------
