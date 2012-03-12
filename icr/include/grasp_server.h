/**
 * @author Robert Krug
 * @date   Fri, Mar 9, 2012
 *
 */


#ifndef grasp_server_h___
#define grasp_server_h___

#include "ros/ros.h"
#include "../msg_gen/cpp/include/icr/ContactPoints.h"
#include <boost/thread/mutex.hpp>

//#include <tf/transform_broadcaster.h>
//#include <gazebo_msgs/ContactsState.h>
#include "../srv_gen/cpp/include/icr/SetObject.h"
#include <vector>
#include "phalange.h"
#include "model_server.h"

class GraspServer
{
 public:

  GraspServer();
  ~GraspServer();

  void spin();

 private:

  ros::NodeHandle nh_, nh_private_;
  ros::ServiceServer set_target_obj_srv_;
  ros::Publisher contact_points_pub_;  

  boost::mutex lock_;
  Model target_obj_;
  std::vector<Phalange*> phalanges_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool setObject(icr::SetObject::Request  &req, icr::SetObject::Response &res);
};



#endif
