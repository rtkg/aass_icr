#include "../include/grasp_server.h"
#include <tf/tf.h>

GraspServer::GraspServer() : nh_private_("~")
{
  // std::string icr_prefix;
  // std::string gazebo_prefix;
  // std::string param;
  // nh_private_.searchParam("icr_prefix", param);
  // nh_private_.getParam(param, icr_prefix);
  std::string searched_param;

 
  XmlRpc::XmlRpcValue phalange_config;
  XmlRpc::XmlRpcValue L_T_Cref;
  Model phalange_model;
  if (nh_private_.searchParam("phalanges", searched_param))
    {
      nh_.getParam(searched_param,phalange_config);
      ROS_ASSERT(phalange_config.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    
      for (int32_t i = 0; i < phalange_config.size(); ++i) 
	{    
	  L_T_Cref=phalange_config[i]["L_T_Cref"];
	  ROS_ASSERT(phalange_config[i]["link_geom"].getType() == XmlRpc::XmlRpcValue::TypeString);
	  ROS_ASSERT(phalange_config[i]["link_frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
	  ROS_ASSERT(phalange_config[i]["link_name"].getType() == XmlRpc::XmlRpcValue::TypeString);
          ROS_ASSERT(phalange_config[i]["sensor_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
	  ROS_ASSERT(L_T_Cref.getType() == XmlRpc::XmlRpcValue::TypeArray);
	  ROS_ASSERT(L_T_Cref.size()==6);//a transformation is specified by an offset vector + RPY values
	  for (int32_t j = 0; j <L_T_Cref.size();j++) 
	    ROS_ASSERT(L_T_Cref[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);

	  tf::Transform tf;
          tf.setOrigin(tf::Vector3(L_T_Cref[0],L_T_Cref[1],L_T_Cref[2]));
	  tf.setRotation(tf::createQuaternionFromRPY(L_T_Cref[3],L_T_Cref[4],L_T_Cref[5]));
	 
	  phalange_model.name_=(std::string)phalange_config[i]["link_name"];
          phalange_model.geom_=(std::string)phalange_config[i]["link_geom"];
          phalange_model.frame_id_=(std::string)phalange_config[i]["link_frame_id"];

	  phalanges_.push_back(new Phalange(tf,phalange_model,(std::string)phalange_config[i]["sensor_topic"]));

         }
    }
  else
    {
      ROS_ERROR("The hand phalange configurations are not specified - cannot start the Grasp Server");
      exit(0);
    }


  set_object_srv_ = nh_.advertiseService("set_object",&GraspServer::setObject,this);
  // gazebo_spawn_clt_ = nh_.serviceClient<gazebo_msgs::SpawnGrasp>(gazebo_prefix + "/spawn_urdf_model");
  // gazebo_delete_clt_ = nh_.serviceClient<gazebo_msgs::DeleteGrasp>(gazebo_prefix + "/delete_model");
  // gazebo_pause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix + "/pause_physics");
  // gazebo_unpause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix + "/unpause_physics");

 
}
//-----------------------------------------------------------------------------------------------
GraspServer::~GraspServer()
{
  for (unsigned int i=0; i<phalanges_.size();i++)
    delete phalanges_[i];
}
//-----------------------------------------------------------------------------------------------
bool GraspServer::setObject(icr::SetObject::Request  &req, icr::SetObject::Response &res)
{
  res.success=false;
  lock_.lock();
  object_.name_=req.name;
  object_.frame_id_=req.frame_id;
  object_.geom_=req.geom;

  for (unsigned int i=0;i<phalanges_.size();i++)
    phalanges_[i]->setTargetObjGeom(object_.geom_);

  lock_.unlock();
  res.success=true;

  return res.success;
}
//-----------------------------------------------------------------------------------------------
void GraspServer::spin()
{


 ros::spinOnce();
}
//-----------------------------------------------------------------------------------------------
