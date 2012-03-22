/**
 *
 * @brief Remaps various contact sensor messages to a icr/ContactState message
 *
 *
 */

#include <ros/ros.h>
#include "sensor_remapper/sensor_remapper.h"

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_remapper");
  SensorRemapper sensor_remapper;

  ROS_INFO("Contact sensor remapper started.");
  ros::spin();

  return 0;
}

