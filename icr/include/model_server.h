#ifndef model_server_h___
#define model_server_h___

#include "ros/ros.h"
#include "../srv_gen/cpp/include/icr/load_model.h"

#include <boost/thread/mutex.hpp>

class ModelServer
{
public:

  ModelServer();
   ~ModelServer(){};

private:

   ros::NodeHandle nh_, nh_private_;

   boost::mutex data_mutex_;
   std::string urdf_file_;
   ros::ServiceServer load_object_srv_;
   ros::ServiceClient gazebo_spawn_clt_; 
   ros::ServiceClient gazebo_delete_clt_; 
  /////////////////
  //  CALLBACKS  //
  /////////////////

   bool loadModel(icr::load_model::Request  &req, icr::load_model::Response &res);

};



#endif
