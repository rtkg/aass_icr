#include "../include/model_server.h"
#include "../srv_gen/cpp/include/icr/load_model.h"
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

ModelServer::ModelServer() : nh_private_("~")
{
  std::string icr_prefix;
  std::string gazebo_prefix;
  std::string param;
  nh_private_.searchParam("icr_prefix", param);
  nh_private_.param(param, icr_prefix, std::string());
  nh_private_.searchParam("gazebo_prefix", param);
  nh_private_.param(param, gazebo_prefix, std::string());

  load_object_srv_ = nh_.advertiseService(icr_prefix + "/model_server/load_object",&ModelServer::loadModel,this);
  gazebo_spawn_clt_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(gazebo_prefix + "/spawn_gazebo_model");
  gazebo_delete_clt_ = nh_.serviceClient<gazebo_msgs::DeleteModel>(gazebo_prefix + "/delete_model");
}

bool ModelServer::loadModel(icr::load_model::Request  &req, icr::load_model::Response &res)
{
  data_mutex_.lock();
  res.success=false;

  gazebo_msgs::DeleteModel delete_model;
  delete_model.request.model_name="icr_object";

  if(!gazebo_delete_clt_.call(delete_model))
    ROS_WARN("Could not delete previous icr_object model");

  gazebo_msgs::SpawnModel spawn_model;
  spawn_model.request.model_name="icr_object";
  spawn_model.request.model_xml=req.urdf_file;
  spawn_model.request.initial_pose=req.initial_pose;
  spawn_model.request.reference_frame="world";

  if (gazebo_spawn_clt_.call(spawn_model))
    ROS_INFO("Successfully spawned model %s in Gazebo",req.urdf_file.c_str());
  else
    {
    ROS_ERROR("Failed to spawn model %s in Gazebo",req.urdf_file.c_str());
    data_mutex_.unlock();
    return res.success;
    }

  urdf_file_=req.urdf_file;
  nh_.setParam("/model_server/icr_object",urdf_file_);
  
  res.success=true;
  data_mutex_.unlock();
  return res.success;
}

//    proj_matrix_.setIdentity(number_hand_joints_,number_hand_joints_);

//     joints_names_.resize(number_hand_joints_);
//     ShadowhandToCybergloveRemapper::initNames();

//     std::string param;
//     std::string abg_path;
//     std::string gains_path;
//     std::string espace_dir;
//     n_tilde_.searchParam("mapping_abg_path", param);
//     n_tilde_.param(param, abg_path, std::string());
//     n_tilde_.searchParam("mapping_gains_path", param);
//     n_tilde_.param(param, gains_path, std::string());
//     n_tilde_.searchParam("projection_espace_dir", param);
//     n_tilde_.param(param, espace_dir, std::string());

//     calibration_parser_ = new CalibrationParser(abg_path,gains_path);
//     eigenspace_parser_ = new EigenspaceParser(espace_dir);

//     ROS_INFO("Mapping files loaded for the Cyberglove: %s, %s", abg_path.c_str(),gains_path.c_str());
//     ROS_INFO("Eigenspace projection directory set: %s", espace_dir.c_str());

//     std::string prefix;
//     std::string searched_param;
//     n_tilde_.searchParam("cyberglove_prefix", searched_param);
//     n_tilde_.param(searched_param, prefix, std::string());

//     std::string full_topic = prefix + "/raw/joint_states";
//     cyberglove_jointstates_sub_ = node_.subscribe(full_topic, 10, &ShadowhandToCybergloveRemapper::jointStatesCallback, this);

//     n_tilde_.searchParam("remapper_prefix", searched_param);
//     n_tilde_.param(searched_param, prefix, std::string());
//     full_topic = prefix + "project_eigenspace";
//    
   
//     n_tilde_.searchParam("sendupdate_prefix", searched_param);
//     n_tilde_.param(searched_param, prefix, std::string());
//     full_topic = prefix + "sendupdate";
//     shadowhand_pub_ = node_.advertise<sr_robot_msgs::sendupdate> (full_topic, 5);
// }
