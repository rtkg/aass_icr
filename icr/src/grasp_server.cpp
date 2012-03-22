#include "../include/grasp_server.h"
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include "../msg_gen/cpp/include/icr/ContactPoint.h"
#include "../msg_gen/cpp/include/icr/ContactPoints.h"
#include "../msg_gen/cpp/include/icr/StampedContactPose.h" //REMOVE
#include <geometry_msgs/PoseStamped.h>//REMOVE

GraspServer::GraspServer() : nh_private_("~")
{
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

  set_target_obj_srv_ = nh_.advertiseService("set_object",&GraspServer::setObject,this);
  contact_points_pub_=nh_.advertise<icr::ContactPoints>("contact_points",5); 
  // debug_pub_=nh_.advertise<geometry_msgs::PoseStamped>("debug_thumb",5);  
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

  target_obj_.reset(new Model(req.name,req.frame_id,req.geom));

  //Transmit the new target object to the Phalanges
  for (unsigned int i=0;i<phalanges_.size();i++)
    phalanges_[i]->setTargetObj(target_obj_);
   
  lock_.unlock();
  res.success=true;

  return res.success;
}
//-----------------------------------------------------------------------------------------------
void GraspServer::spin()
{
  icr::ContactPoints pts;
  icr::ContactPoint p;

  //Get the current contact points from the Phalanges and publish them
  for(unsigned int i = 0; i < phalanges_.size();i++)
     {
       p.position=phalanges_[i]->getStampedContactPose()->contact_pose.pose.position;
       p.touching=phalanges_[i]->getStampedContactPose()->contact_pose.touching;
       pts.points.push_back(p);
     }
  contact_points_pub_.publish(pts);

  //geometry_msgs::PoseStamped dbg;
  // dbg.header=phalanges_[0]->getStampedContactPose()->header;
  // dbg.pose=phalanges_[0]->getStampedContactPose()->contact_pose.pose;
  // debug_pub_.publish(dbg);
  ros::spinOnce();
}
//-----------------------------------------------------------------------------------------------
