/**
 * @author Robert Krug
 * @date   Sat, Mar 10, 2012
 *
 *@brief Subscribes to gazebo_msgs/ContactsState outputted by the sensor_bumpers and computes the
 *average contact position. If the phalange is not in contact it publishes a predefined reference
 *contact point. 
 *
 */

#ifndef phalange_h___
#define phalange_h___

#include "ros/ros.h"
#include "../srv_gen/cpp/include/icr/SetPose.h"
#include "../msg_gen/cpp/include/icr/StampedContactPose.h"
#include "../msg_gen/cpp/include/icr/StampedContactPoint.h"
#include "../msg_gen/cpp/include/icr/ContactPose.h"
#include "../msg_gen/cpp/include/icr/ContactPoint.h"
#include <gazebo_msgs/ContactsState.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <string>
#include "model_server.h"
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

/**
 *@brief This class represents a phalange of the hand. It subscribes to gazebo_msgs/ContactsState
 *outputted by the sensor_bumpers and computes the average contact position and normal direction for
 *a given target object. If the phalange is not in contact it publishes a predefined reference
 *contact pose. The get functions return boost::shared_ptr in order to be thread safe. Note, that
 *presently the published geometry_msgs/StampedPose message can create problems when visualizing in
 *Rviz. This is due to the fact that Rviz only buffers 5 geometry_msgs/StampedPose msgs and proper
 *tf transformations might not be available at the time the contact pose is published. This can be
 *fixed by increasing the buffer size in rviz/src/rviz/default_plugin/pose_display.cpp and
 *recompiling Rviz. Currently (in ROS Electric) there is no way to increase Rviz's tf::MessageFilter
 *queue at runtime. Also using a topic_tools/throttle should fix the problem (did not test it
 *though...)
 */
class Phalange
{
 public:

 
  Phalange(tf::Transform const & L_T_Cref,Model const & model,std::string const & sensor_topic);
  ~Phalange(){};

  boost::shared_ptr<icr::StampedContactPose> getStampedContactPose();
  icr::StampedContactPoint::ConstPtr getStampedContactPoint();
  boost::shared_ptr<icr::ContactPose> getContactPose();
  boost::shared_ptr<icr::ContactPoint> getContactPoint();

  boost::shared_ptr<Model> getPhalangeModel();
  bool setTargetObjGeom(std::string const & obj_geom);  
 
 private:

/**
 *@brief Using the default constructor is not allowed since the class needs to be initialized
 *properly. This could be changed by implementing corresponding setXYZ methods
 */
  Phalange();
  
  ros::NodeHandle nh_, nh_private_;
/**
 *@brief Publishes the average pose if in contact, otherwise a predefined reference pose is published
 */
  ros::Publisher ct_pose_pub_;
/**
 *@brief Subscribes to a gazebo_msgs/ContactsState message 
 */
  ros::Subscriber contacts_subs_;
/**
 *@brief Allows to set the pose of the reference contact w.r.t the phalange link frame id (mainly intended for debugging)
 */
  ros::ServiceServer set_pose_srv_;
/**
 *@brief Pose of the reference contact expressed in the link frame id of the phalange
 */
  tf::Transform L_T_Cref_;
/**
 *@brief Current contact pose expressed in the link frame id of the phalange. The pose is stored for
 *convenience as a pointer to a  icr/StampedContactPose  in order to simplify tf::lookupTransform
 *calls and tf::message_filter calls which need a timestamped pose msg.
 */
  boost::shared_ptr<icr::StampedContactPose> C_T_L_;
/**
 *@brief Represents the model parameters of the phalange - holds name, link frame id and the name of
 *the geometry of the link
 */
  boost::shared_ptr<Model> model_;
/**
 *@brief Defines the name of the target object geometry - only contacts between the link geometry
 *and this geometry are considered
 */
  std::string obj_geom_;
  boost::mutex lock_;

  std::string tf_prefix_;
/**
 *@brief Helper function to compute the average contact position/normal (The
 *gazebo_msgs/ContactsState hold all points in contact)
 */
  tf::Vector3 averageVectors(std::vector<geometry_msgs::Vector3> const & vecs);
/**
 *@brief Used to generate the contact pose if Phalange::obj_geom_ and Phalange::model_->geom_ are
 *touching. The z-axis of the pose is given as argument. The x & y axis are computed via projecting
 *the x & y axis of the contact reference frame on the nullspace of the new z axis (this is not very
 *stable when axis of the link frame are (nearly) aligned with the z axis of the contact pose since
 *the norms of the projections of those axis will be (nearly) 0)
 */
  tf::Quaternion projectPose(tf::Vector3 const & z);

  /////////////////
  //  CALLBACKS  //
  /////////////////

/**
 *@brief Callback listening to ContactsState msgs (which are always published, even if they are
 *empty). Computes the current contact pose.
 */
  void listenContacts(const gazebo_msgs::ContactsState::ConstPtr& cts_st);
  bool setPose(icr::SetPose::Request  &req, icr::SetPose::Response &res);

};
#endif
