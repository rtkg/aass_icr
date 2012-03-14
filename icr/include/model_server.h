/**
 * @author Robert Krug
 * @date   Wed Feb 22 2012
 *
 *
 *Implements a model server which spawns object models used fot the icr computation in gazebo and
 *pushes the according urdf files on the parameter server for RVIZ - visualization. Also the pose of
 *the object in the gazebo world frame is broadcasted via tf in order to track the object in RVIZ.
 *
 */


#ifndef model_server_h___
#define model_server_h___

#include "ros/ros.h"
#include "../srv_gen/cpp/include/icr/load_model.h"
#include "../srv_gen/cpp/include/icr/SetObject.h"
#include <boost/thread/mutex.hpp>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <iostream>

//----------------------------------------------------------------------------
struct Model
{
  std::string name_;
  std::string frame_id_;
  std::string geom_;

  Model() : name_("default"),frame_id_("default"), geom_("default"){}
  Model(std::string const & name, std::string const & frame_id,std::string const & geom) : name_(name),frame_id_(frame_id), geom_(geom){}

  friend std::ostream& operator<<(std::ostream &stream,Model const& model);
};
//----------------------------------------------------------------------------
class ModelServer
{
 public:

  ModelServer();
  ~ModelServer(){};

 private:

  ros::NodeHandle nh_, nh_private_;
  boost::mutex data_mutex_;
  tf::TransformBroadcaster tf_brc_;
  std::string model_dir_;
  std::string ref_frame_id_;
  Model icr_object_;

  ros::ServiceServer load_object_srv_;
  ros::ServiceClient gazebo_spawn_clt_; 
  ros::ServiceClient gazebo_delete_clt_; 
  ros::ServiceClient gazebo_pause_clt_; 
  ros::ServiceClient gazebo_unpause_clt_; 
  ros::Subscriber gazebo_modstat_sub_;
  ros::ServiceClient set_object_clt_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

 /**
   *  Spawns the given urdf model as "icr_object" in gazebo, if such an object already exists it is
   *  deleted. Also, the urdf file is pushed onto the parameter server.
   */
  bool loadModel(icr::load_model::Request  &req, icr::load_model::Response &res);
  void getModelStates(gazebo_msgs::ModelStates const & states);

};



#endif
