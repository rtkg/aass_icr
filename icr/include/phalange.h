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
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <string>
#include "model_server.h"
#include <boost/thread/mutex.hpp>

/**
 *@brief This class represents a phalange of the hand. It subscribes to gazebo_msgs/ContactsState
 *outputted by the sensor_bumpers and computes the average contact position and normal direction for
 *a given target object. If the phalange is not in contact it publishes a predefined reference
 *contact pose. Note, that presently the published geometry_msgs/StampedPose message can create
 *problems when visualizing in Rviz. This is due to the fact that Rviz only buffers 5
 *geometry_msgs/StampedPose msgs and proper tf transformations might not be available at the time
 *the contact pose is published. This can be fixed by increasing the buffer size in
 *rviz/src/rviz/default_plugin/pose_display.cpp and recompiling Rviz. Currently (in ROS Electric)
 *there is no way to increase Rviz's tf::MessageFilter queue at runtime. Also using a
 *topic_tools/throttle should fix the problem (did not test it though...)
 */
class Phalange
{
 public:

 
  Phalange(tf::Transform const & L_T_Cref,Model const & model,std::string const & sensor_topic);
  ~Phalange();

  tf::Vector3* getContactPositon();
  tf::Quaternion* getContactOrientation();  
  Model* getPhalangeModel();
  bool setTargetObjGeom(std::string const & obj_geom);  
  bool touching();

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
 *@brief Current contact position expressed in the link frame id of the phalange
 */
  tf::Vector3* ct_pos_;
/**
 *@brief Current contact orientation expressed in the link frame id of the phalange
 */
  tf::Quaternion* ct_ori_;
/**
 *@brief Represents the model parameters of the phalange - holds name, link frame id and the name of
 *the geometry of the link
 */
  Model* model_;
/**
 *@brief Defines the name of the target object geometry - only contacts between the link geometry
 *and this geometry are considered
 */
  std::string obj_geom_;
  boost::mutex lock_;
/**
 *@brief Flag indicating whether the link geometry is in contact with the target object geometry
 *specified in Phalange::obj_geom_
 */
  bool touching_;
  std::string tf_prefix_;
/**
 *@brief Helper function to compute the average contact position/normal (The
 *gazebo_msgs/ContactsState hold all points in contact)
 */
  tf::Vector3 averageVectors(std::vector<geometry_msgs::Vector3> const & vecs);
/**
 *@brief Used to generate the contact pose if Phalange::obj_geom_ and Phalange::model_->geom_ are
 *touching. The z-axis of the pose is the normal vector given as argument. The x & y axis are
 *computed via projecting the x & y axis of the contact reference frame on the nullspace of the new
 *z axis (this is not very stable when axis of the link frame are (nearly) aligned with the z axis
 *of the contact pose since the norms of the projections of those axis will be (nearly) 0)
 */
  tf::Quaternion projectPose(tf::Vector3 const & normal);

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
