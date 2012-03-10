/**
 * @author Robert Krug
 * @date   Sat, Mar 10, 2012
 *
 */


#ifndef phalange_h___
#define phalange_h___

#include "ros/ros.h"
//#include "../srv_gen/cpp/include/icr/load_model.h"
//#include <boost/thread/mutex.hpp>
//#include "std_msgs/String.h"
//#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ContactsState.h>
#include <tf/tf.h>
#include <string>

class Phalange
{
 public:

 
  Phalange(tf::Transform const & L_T_Cref,std::string const & link_geom);
  ~Phalange();

  void updateContact(gazebo_msgs::ContactsState::ConstPtr& cts_st, std::string const & obj_geom);
  tf::Vector3* getContactPositon();
  tf::Quaternion* getContactOrientation();  
  std::string* getPhalangeGeom();

 private:
 Phalange();
  

  ros::NodeHandle nh_;
  ros::Publisher ct_pose_pub_;

tf::Transform L_T_Cref_;
   tf::Vector3* ct_pos_;
  tf::Quaternion* ct_ori_;
 std::string* link_geom_;

  


};



#endif
