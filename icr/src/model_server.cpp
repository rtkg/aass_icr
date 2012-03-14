#include "../include/model_server.h"
#include "../srv_gen/cpp/include/icr/load_model.h"
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <boost/function.hpp>
//#include <urdf_interface/link.h>
#include <urdf/model.h>
#include <urdf_interface/link.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>

//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream &stream,Model const& model)
{
  stream <<'\n'<<"MODEL: "<<'\n'
         <<"Name: "<<model.name_<<'\n'
         <<"Frame id: "<<model.frame_id_<<'\n'
	 <<"Geometry: "<<model.geom_<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------------
ModelServer::ModelServer() : nh_private_("~")
{
  ref_frame_id_="/world"; //initialize the reference frame 

  std::string icr_prefix;
  std::string gazebo_prefix;
  std::string param;

  nh_private_.searchParam("gazebo_prefix", param);
  nh_private_.getParam(param, gazebo_prefix);

  nh_private_.searchParam("model_directory", param);
  nh_private_.getParam(param, model_dir_);

  if(nh_private_.searchParam("reference_frame_id", param)) //try to read the reference frame from the parameter server
    nh_private_.getParam(param, ref_frame_id_);
  else
    ROS_WARN("No reference frame id found on the parameter server - using %s",ref_frame_id_.c_str());

  load_object_srv_ = nh_.advertiseService("load_object",&ModelServer::loadModel,this);
  gazebo_spawn_clt_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(gazebo_prefix + "/spawn_urdf_model");
  gazebo_delete_clt_ = nh_.serviceClient<gazebo_msgs::DeleteModel>(gazebo_prefix + "/delete_model");
  gazebo_pause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix + "/pause_physics");
  gazebo_unpause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix + "/unpause_physics");
  gazebo_modstat_sub_ = nh_.subscribe(gazebo_prefix + "/model_states", 10, &ModelServer::getModelStates, this);
  set_object_clt_ = nh_.serviceClient<icr::SetObject>("set_object");
}

void ModelServer::getModelStates(gazebo_msgs::ModelStates const & states)
{
  bool obj_spawned=false;
  unsigned int icr_object_id;
  //see if the icr object is spawned, return if not
  for(icr_object_id=0; icr_object_id < states.name.size(); icr_object_id++)
    if(states.name[icr_object_id]==icr_object_.name_)
      {
	obj_spawned=true;
	break;
      }

  if(!obj_spawned)
      return;

  //broadcast the icr object's pose to tf
  static 
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(states.pose[icr_object_id].position.x,states.pose[icr_object_id].position.y,states.pose[icr_object_id].position.z));
  transform.setRotation(tf::Quaternion(states.pose[icr_object_id].orientation.x,states.pose[icr_object_id].orientation.y,states.pose[icr_object_id].orientation.z,states.pose[icr_object_id].orientation.w ));
  tf_brc_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),ref_frame_id_, icr_object_.frame_id_));

}
bool ModelServer::loadModel(icr::load_model::Request  &req, icr::load_model::Response &res)
{
  data_mutex_.lock();
  res.success=false;

  gazebo_msgs::DeleteModel delete_model;
  delete_model.request.model_name=icr_object_.name_;

  //Delete an icr object in case its already spawned in Gazebo (would be nicer to add a check before
  //and only do this when an icr_object actually exists)
  if(!gazebo_delete_clt_.call(delete_model))
    ROS_WARN("Could not delete previous icr_object model %s",icr_object_.name_.c_str());

  std::string line;
  std::string model;
  std::ifstream file((model_dir_+req.local_file).c_str());

  if(!file.is_open()) 
    {
      ROS_ERROR("Could not open file %s",(model_dir_+req.local_file).c_str());
     data_mutex_.unlock();
     return res.success;
    }
  while(!file.eof()) // Parse the contents of the given urdf in a string
    {
      std::getline(file,line);
      model+=line;
    }
  file.close();

  //Parse the urdf in order to get the robot name and geometry
  urdf::Model urdf_model;
  if(!urdf_model.initString(model))
    {  
      ROS_ERROR("Could not parse urdf model %s",(model_dir_+req.local_file).c_str());
      data_mutex_.unlock();
      return res.success;
    }
  icr_object_.name_=urdf_model.getName();
  //Right now, the URDF parser does not support geometry names. Thus, it is implied that the contact
  //geometry is named after the root link of the object + "_geom"
  icr_object_.geom_=urdf_model.getRoot()->name + "_geom";
  icr_object_.frame_id_="/"+urdf_model.getRoot()->name;//set the frame id to the base link name

  gazebo_msgs::SpawnModel spawn_model;
  spawn_model.request.model_name=icr_object_.name_; //name the model after its base link
  spawn_model.request.model_xml=model;
  spawn_model.request.initial_pose=req.initial_pose;
  spawn_model.request.reference_frame="world"; //spawn the object in Gazebo's world frame

  std_srvs::Empty empty;
  gazebo_pause_clt_.call(empty);
  if (gazebo_spawn_clt_.call(spawn_model))
    ROS_INFO("Successfully spawned model %s in Gazebo",req.local_file.c_str());
  else
    {
      ROS_ERROR("Failed to spawn model %s in Gazebo - the model pose will not be broadcasted",req.local_file.c_str());
      data_mutex_.unlock();
      return res.success;
    }
  gazebo_unpause_clt_.call(empty);

  //Push the loaded file on the parameter server 
  nh_.setParam("icr_object",model);


  //Send the object to the grasp server
  icr::SetObject obj;
  obj.request.name=icr_object_.name_;
  obj.request.frame_id=icr_object_.frame_id_;
  obj.request.geom=icr_object_.geom_;
   
  set_object_clt_.call(obj); 
  if(!obj.response.success)
    ROS_WARN("Set object client call unsuccessful.");

  res.success=true;
  data_mutex_.unlock();
  return res.success;
}

