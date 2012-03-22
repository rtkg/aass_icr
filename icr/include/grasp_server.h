/**
 * @author Robert Krug
 * @date   Fri, Mar 9, 2012
 *
 */


#ifndef grasp_server_h___
#define grasp_server_h___

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include "../srv_gen/cpp/include/icr/SetObject.h"
#include <vector>
#include "phalange.h"
#include "model_server.h"

/**
 *@brief Contains a list of Phalanges and publishes corresponding icr/ContactPoints messages 
 */
class GraspServer
{
 public:

  GraspServer();
  ~GraspServer();

/**
 *@brief Collects the current contact poses from the Phalanges and publishes icr/ContactPoints messages
 */
  void spin();

 private:

  ros::NodeHandle nh_, nh_private_;
  ros::ServiceServer set_target_obj_srv_;
  ros::Publisher contact_points_pub_;  
  //  ros::Publisher debug_pub_;//REMOVE  
  boost::mutex lock_;
/**
 *@brief The current target object - needed by the Phalanges to determine whether they are in
 *conatct or not
 */
  boost::shared_ptr<Model> target_obj_;
/**
 *@brief Vector containing the phalanges of the hand - the order is important
 */
  std::vector<Phalange*> phalanges_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool setObject(icr::SetObject::Request  &req, icr::SetObject::Response &res);
};
#endif
