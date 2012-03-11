/**
 * @author Robert Krug
 * @date   Fri, Mar 9, 2012
 *
 */


#ifndef grasp_server_h___
#define grasp_server_h___

#include "ros/ros.h"
//#include "../srv_gen/cpp/include/icr/load_model.h"
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
//#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ContactsState.h>
#include "../srv_gen/cpp/include/icr/SetObject.h"
#include <vector>
#include "phalange.h"
#include "model_server.h"

class GraspServer
{
 public:

  GraspServer();
  ~GraspServer();

 private:

  ros::NodeHandle nh_, nh_private_;
  ros::V_Subscriber phalange_contacts_subs_;

  Model model_;
  std::vector<Phalange*> phalanges_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  void listenContacts(const gazebo_msgs::ContactsState::ConstPtr& cts_st, unsigned int i)
	  {
	    ROS_INFO("I heard: with user string ");
	  }

};



#endif
