/**
 * @author Robert Krug
 * @date   Fri, Mar 9, 2012
 *
 */


#ifndef grasp_server_h___
#define grasp_server_h___

#include "ros/ros.h"
#include "../msg_gen/cpp/include/icr/StampedContactPoint.h"
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
//#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include "../srv_gen/cpp/include/icr/SetObject.h"
#include <vector>
#include "phalange.h"
#include "model_server.h"
//#include <geometry_msgs/PoseStamped.h>

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
  boost::shared_ptr<Model> target_obj_;
  std::vector<Phalange*> phalanges_;


  tf::TransformListener tf_list_;
  tf::MessageFilter<icr::StampedContactPoint>* tf_filter_;



  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool setObject(icr::SetObject::Request  &req, icr::SetObject::Response &res);
};



#endif
