#include "../include/grasp_server.h"
#include <tf/tf.h>



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
  //  contact_points_pub_=nh_.advertise<icr::ContactPoints>("contact_points",5);


  tf_filter_ = new tf::MessageFilter<icr::StampedContactPoint>(tf_list_, "/default", 50);
}
//-----------------------------------------------------------------------------------------------
GraspServer::~GraspServer()
{
  for (unsigned int i=0; i<phalanges_.size();i++)
    delete phalanges_[i];

  delete tf_filter_;
}
//-----------------------------------------------------------------------------------------------
bool GraspServer::setObject(icr::SetObject::Request  &req, icr::SetObject::Response &res)
{
  res.success=false;
  lock_.lock();

  target_obj_.reset(new Model(req.name,req.frame_id,req.geom));

  for (unsigned int i=0;i<phalanges_.size();i++)
    phalanges_[i]->setTargetObjGeom(target_obj_->geom_);
 
  tf_filter_->setTargetFrame(target_obj_->frame_id_);
  
  lock_.unlock();
  res.success=true;

  return res.success;
}
//-----------------------------------------------------------------------------------------------
void GraspServer::spin()
{
  if (!target_obj_)
    {
      ros::spinOnce();
      return;
    }

  // icr::StampedContactPoints s_cpts;
  // icr::ContactPoints ct_p;
  // geometry_msgs::PoseStamped O_T_C;
  // geometry_msgs::Quaternion q;
  lock_.lock();


 icr::StampedContactPoint::ConstPtr scp(new icr::StampedContactPoint);

//   ros::MessageEvent (const ConstMessagePtr &message)
//  typedef boost::shared_ptr<ConstMessage> ConstMessagePtr;
// typedef typename boost::add_const<M>::type ConstMessage;


 ros::MessageEvent<icr::StampedContactPoint> ev(scp);


  for(unsigned int i = 0; i < phalanges_.size();i++)
     {
       //  tf_filter_.add(ev);


      //  boost::shared_ptr<geometry_msgs::PoseStamped> P_T_C = phalanges_[i]->getContactPose();
      // try
      // 	{

      // 	  q=phalanges_[i]->getContactPose()->pose.orientation;
      // 	  //          std::cout<<"q orig: "<<q.x<<" "<<q.y<<" "<<q.z<<" "<<q.w<<std::endl;
      // 	  tf_list_.waitForTransform(target_obj_->frame_id_, P_T_C->header.frame_id,P_T_C->header.stamp, ros::Duration(0.5));
      // 	  tf_list_.transformPose(target_obj_->frame_id_, *(P_T_C).get(), O_T_C);
      //     q=O_T_C.pose.orientation;
      // 	  //	  std::cout<<"q trans: "<<q.x<<" "<<q.y<<" "<<q.z<<" "<<q.w<<std::endl;
      // 	}
      // catch (tf::TransformException ex){
      // 	ROS_ERROR("%s",ex.what());
      // 	//	lock_.unlock(); 
      // }


      // ct_p.points.push_back(O_T_C.pose.position);
      // ct_p.touching.push_back(phalanges_[i]->touching());
    }

  lock_.unlock();

  // contact_points_pub_.publish(ct_p);

  ros::spinOnce();
}
//-----------------------------------------------------------------------------------------------
