/**
 * @author Robert Krug
 * @date   Sat, Mar 10, 2012
 *
 */


#ifndef phalange_h___
#define phalange_h___

#include "ros/ros.h"
#include "../srv_gen/cpp/include/icr/SetPose.h"
//#include <boost/thread/mutex.hpp>
//#include "std_msgs/String.h"
//#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <string>
#include "model_server.h"
#include <boost/thread/mutex.hpp>

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
  Phalange();
  

  ros::NodeHandle nh_;
  ros::Publisher ct_pose_pub_;
  ros::Subscriber contacts_subs_;
  ros::ServiceServer set_pose_srv_;

  tf::Transform L_T_Cref_;
  tf::Vector3* ct_pos_;
  tf::Quaternion* ct_ori_;
  Model* model_;
  std::string obj_geom_;
  boost::mutex lock_;
  bool touching_;

  tf::Vector3 averageVectors(std::vector<geometry_msgs::Vector3> const & vecs);
  tf::Quaternion projectPose(tf::Vector3 const & normal);

  /////////////////
  //  CALLBACKS  //
  /////////////////

  void listenContacts(const gazebo_msgs::ContactsState::ConstPtr& cts_st);
  bool setPose(icr::SetPose::Request  &req, icr::SetPose::Response &res);

};



#endif
