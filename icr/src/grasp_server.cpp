#include "../include/grasp_server.h"
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include "icr_msgs/ContactPoint.h"
#include "icr_msgs/ContactPoints.h"
#include "icr_msgs/StampedContactPose.h" //REMOVE
#include <geometry_msgs/PoseStamped.h>//REMOVE

namespace ICR
{

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
	  ROS_ASSERT(phalange_config[i]["phl_frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
	  ROS_ASSERT(phalange_config[i]["phl_name"].getType() == XmlRpc::XmlRpcValue::TypeString);
          ROS_ASSERT(phalange_config[i]["sensor_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
	  ROS_ASSERT(L_T_Cref.getType() == XmlRpc::XmlRpcValue::TypeArray);
	  ROS_ASSERT(L_T_Cref.size()==6);//a transformation is specified by an offset vector + RPY values
	  for (int32_t j = 0; j <L_T_Cref.size();j++) 
	    ROS_ASSERT(L_T_Cref[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);

	  tf::Transform tf;
          tf.setOrigin(tf::Vector3(L_T_Cref[0],L_T_Cref[1],L_T_Cref[2]));
	  tf.setRotation(tf::createQuaternionFromRPY(L_T_Cref[3],L_T_Cref[4],L_T_Cref[5]));

	  phalanges_.push_back(new Phalange(tf,(std::string)phalange_config[i]["phl_name"],(std::string)phalange_config[i]["phl_frame_id"] ,(std::string)phalange_config[i]["sensor_topic"]));

         }
    }
  else
    {
      ROS_ERROR("The hand phalange configurations are not specified - cannot start the Grasp Server");
      exit(0);
    }

  set_obj_srv_ = nh_.advertiseService("set_object",&GraspServer::setObject,this);
  contact_points_pub_=nh_.advertise<icr_msgs::ContactPoints>("contact_points",5); 
  //debug_pub_=nh_.advertise<geometry_msgs::PoseStamped>("debug_ff",5);  
}
//-----------------------------------------------------------------------------------------------
GraspServer::~GraspServer()
{
  for (unsigned int i=0; i<phalanges_.size();i++)
    delete phalanges_[i];

}
//-----------------------------------------------------------------------------------------------
bool GraspServer::setObject(icr_msgs::SetObject::Request  &req, icr_msgs::SetObject::Response &res)
{
  res.success=false;
  lock_.lock();

  //Transmit the new target object to the Phalanges
  for (unsigned int i=0;i<phalanges_.size();i++)
    phalanges_[i]->setTargetObjFrameId(req.object.points.header.frame_id);
   
  lock_.unlock();
  res.success=true;

  return res.success;
}
//-----------------------------------------------------------------------------------------------
void GraspServer::spin()
{
  icr_msgs::ContactPoints pts;
  icr_msgs::ContactPoint p;

  //Get the current contact points from the Phalanges and publish them
  for(unsigned int i = 0; i < phalanges_.size();i++)
     {
       p.position=phalanges_[i]->getStampedContactPose()->contact_pose.pose.position;
       p.touching=phalanges_[i]->getStampedContactPose()->contact_pose.touching;
       p.phalange=phalanges_[i]->getPhalangeName();
       pts.points.push_back(p);
     }
  contact_points_pub_.publish(pts);

  // geometry_msgs::PoseStamped dbg;
  // dbg.header=phalanges_[1]->getStampedContactPose()->header;
  // dbg.pose=phalanges_[1]->getStampedContactPose()->contact_pose.pose;
  // debug_pub_.publish(dbg);
  ros::spinOnce();
}
//-----------------------------------------------------------------------------------------------
}//end namespace
