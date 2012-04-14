/**
 * @author Robert Krug
 * @date  Tue Apr 10 2012
 *
 */

#include "../include/pose_broadcaster.h"
#include <tf/tf.h>
#include <gazebo_msgs/GetModelState.h>

namespace ICR
{
  PoseBroadcaster::PoseBroadcaster() : nh_private_("~"),ref_frame_id_("/fixed")
  {
    std::string param;
    std::string gazebo_prefix;
    std::string uc3m_objtrack_prefix;

    nh_private_.searchParam("pose_source", param);
    nh_private_.getParam(param, pose_source_);

    if(nh_private_.searchParam("reference_frame_id", param)) //try to read the reference frame from the parameter server
      nh_private_.getParam(param, ref_frame_id_);
    else
      ROS_WARN("No reference frame id found on the parameter server - using %s",ref_frame_id_.c_str());

    nh_private_.searchParam("gazebo_prefix", param);
    nh_private_.getParam(param, gazebo_prefix);

    nh_private_.searchParam("uc3m_objtrack_prefix", param);
    nh_private_.getParam(param, uc3m_objtrack_prefix);

    gazebo_get_ms_clt_ = nh_.serviceClient<gazebo_msgs::GetModelState>(gazebo_prefix + "/get_model_state");

    //If Gazebo is the intended pose source, wait for Gazebo services to be available 
    if(!strcmp(pose_source_.c_str(),"gazebo"))
      gazebo_get_ms_clt_.waitForExistence();  
    else if(!strcmp(pose_source_.c_str(),"uc3m_objtrack"))
      {
      ;//wait for the corresponding services here
      }

  }
  //-----------------------------------------------------------------------------------
  void PoseBroadcaster::setObject(pcl::PointCloud<pcl::PointNormal> const & obj_cloud, std::string const & obj_name)
  {
    lock_.lock();
    obj_cloud_=obj_cloud;
    obj_name_=obj_name;
    lock_.unlock();
  }
  //-----------------------------------------------------------------------------------
  void PoseBroadcaster::broadcastPose()
  {
    tf::Transform pose;
    lock_.lock();

    if(!strcmp(pose_source_.c_str(),"gazebo")) //Gazebo is defined as pose source
      {
	if(!getPoseGazebo(pose))
	  {
	    lock_.unlock();
	    return;
	  }
      }
    else if(!strcmp(pose_source_.c_str(),"uc3m_objtrack")) //The UC3M objecttracker is defined as pose source
      {
	if(!getPoseUC3MObjtrack(pose))
	  {
	    lock_.unlock();
	    return;
	  }
      }
    else
      {
	ROS_ERROR("%s is an invalid pose source. Implemented are 'gazebo' and 'uc3m_objtrack'. Cannot broadcast object pose ... ",pose_source_.c_str());
	lock_.unlock();
	return;
      }
    
    //Broadcast the pose to tf
    tf_brc_.sendTransform(tf::StampedTransform(pose, ros::Time::now(),ref_frame_id_, obj_cloud_.header.frame_id));

    lock_.unlock();
  }
  //-----------------------------------------------------------------------------------
  bool PoseBroadcaster::getPoseGazebo(tf::Transform & pose)
  {
    gazebo_msgs::GetModelState get_ms;
    get_ms.request.model_name=obj_name_;

    if(!gazebo_get_ms_clt_.call(get_ms))
      {
	ROS_ERROR("Could not get state of object %s from Gazebo. Cannot determine pose...",obj_name_.c_str());
	return false;
      }

    pose.setOrigin(tf::Vector3(get_ms.response.pose.position.x,get_ms.response.pose.position.y,get_ms.response.pose.position.z));
    pose.setRotation(tf::Quaternion(get_ms.response.pose.orientation.x,get_ms.response.pose.orientation.y,get_ms.response.pose.orientation.z,get_ms.response.pose.orientation.w));
    return true;
  }
  //-----------------------------------------------------------------------------------
  bool PoseBroadcaster::getPoseUC3MObjtrack(tf::Transform & pose)
  {
    //TODO: at the beginning, fit the obj_cloud_ to the blob and compute a static offset pose, then call the pose service from the UC3M objecttracker to get the current pose
      

    return true;
  }
  //-----------------------------------------------------------------------------------


}//end namespace
