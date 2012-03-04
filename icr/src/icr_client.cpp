/** \file icr_client.cpp
 * \author Krzysztof Charusta
 * \date 26 Deb 2012
 * \brief This node is testgin the icr_server node.
 */
#include "ros/ros.h"
//#include "std_msgs/builtin_uint8.h"
#include "icr/set_finger_number.h"
#include "icr/load_object.h"
#include "icr/compute_icr.h"
#include <cstdlib>
#include <string>

using std::cout;
using std::endl;
using std::string;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "icr_client");
  if (argc != 3)
  {
    ROS_INFO("usage: $ icr_client <no_fingers> <path_obj> ");
    return 1;
  }

  ros::NodeHandle n;
  std::string prefix;
  n.searchParam("icr_prefix", prefix);
  string tmp_name;
  ///sets number of fingers 
  tmp_name = prefix + "/icr_server/set_finger_number";
  cout << tmp_name << endl;
  ros::ServiceClient clientFingerNoSet = 
    n.serviceClient<icr::set_finger_number>(tmp_name);
  ///loads an object
  tmp_name = prefix + "/icr_server/load_wfront_obj";
  ros::ServiceClient clientLoadObject = 
    n.serviceClient<icr::load_object>(tmp_name);
  ///computes icrs
  tmp_name = prefix + "/icr_server/compute_icr";
  ros::ServiceClient clientComputeICR = 
    n.serviceClient<icr::compute_icr>(tmp_name);

  icr::set_finger_number set_finger_number_srv;
  icr::load_object load_object_srv;
  icr::compute_icr compute_icr_srv;

  string tmpName("example_object");
  uint16_t myints[] = {1, 2, 3, 4, 5};
  std::vector<uint16_t> 
    tmpPts(myints, myints + sizeof(myints) / sizeof(uint16_t) );

  //defining srvs
  cout <<"patrzcie niedowiarki "
       << sizeof(myints) << " " 
       << sizeof(uint16_t) << " " << tmpPts.size() <<  endl;
  set_finger_number_srv.request.number = atoll(argv[1]);
  load_object_srv.request.path = (std::string)argv[2];
  load_object_srv.request.name = tmpName;

  compute_icr_srv.request.centerpoint_ids = tmpPts;

  ROS_INFO("Path where the *.obj file will be searched:"); 
  ROS_INFO("%s", load_object_srv.request.path.c_str());
  ROS_INFO("Object will have default name: %s",load_object_srv.request.name.c_str());
  ROS_INFO("Nuber of loaded center-points: %d",compute_icr_srv.request.centerpoint_ids.size());

  // call all servers and get responses
  //
  if (clientFingerNoSet.call(set_finger_number_srv)) {
    ROS_INFO("Response: %ld", (long int)set_finger_number_srv.response.success);
  } else {
    ROS_ERROR("Failed to call service /icr_server/set_finger_number");
  }
  //
  if (clientLoadObject.call(load_object_srv)) {
    ROS_INFO("Response: %ld", (long int)load_object_srv.response.success);
  } else {
    ROS_ERROR("Failed to call service /icr_server/load_object");
  }
 
  if (set_finger_number_srv.response.success && load_object_srv.response.success) {
    if (clientComputeICR.call(compute_icr_srv)) {
      ROS_INFO("Response: %ld", (long int)compute_icr_srv.response.success);
      ROS_INFO("ICR server responded somehow. Be happy.");
    } else {
      ROS_ERROR("Failed to call service /icr_server/compute_icr");
    }
  } else {
    ROS_ERROR("ICR_Server don't know about the loaded object and/or added fingers ");
  }
  
  return 0;
}
