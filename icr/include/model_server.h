/**
 * @author Robert Krug
 * @date   Wed Feb 22 2012
 *
 *
 *Implements a model server which spawns object models used fot the
 *icr computation in gazebo and pushes the according urdf files on the
 *parameter server for RVIZ - visualization. Also a proper tf
 *transform is published to track the object in RVIZ.
 *
 */


#ifndef model_server_h___
#define model_server_h___

#include "ros/ros.h"
#include "../srv_gen/cpp/include/icr/load_model.h"
#include <boost/thread/mutex.hpp>
#include <gazebo_msgs/ModelStates.h>

class ModelServer
{
 public:

  ModelServer();
  ~ModelServer(){};

 private:

  ros::NodeHandle nh_, nh_private_;
  boost::mutex data_mutex_;
  std::string root_link_name_;
  ros::ServiceServer load_object_srv_;
  ros::ServiceClient gazebo_spawn_clt_; 
  ros::ServiceClient gazebo_delete_clt_; 
  ros::ServiceClient gazebo_pause_clt_; 
  ros::ServiceClient gazebo_unpause_clt_; 
  ros::Subscriber gazebo_modstat_sub_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

 /** \brief Spawns the given urdf model as "icr_object" in gazebo, if such
   *  an object already exists it is deleted. Also, the urdf file is
   *  pushed onto the parameter server.
   */
  bool loadModel(icr::load_model::Request  &req, icr::load_model::Response &res);
  void getModelStates(gazebo_msgs::ModelStates const & states);

};



#endif
