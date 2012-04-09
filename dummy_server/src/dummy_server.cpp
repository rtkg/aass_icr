#include "dummy_server/dummy_server.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include "rosbag/bag.h"
#include <rosbag/view.h>
#include <boost/foreach.hpp>

//--------------------------------------------------------------------------
DummyServer::DummyServer() : nh_private_("~"), model_dir_("default"), obj_(new icr_msgs::Object),icr_(new icr_msgs::ContactRegions)
{
  std::string param;

  nh_private_.searchParam("model_directory", param);
  nh_private_.getParam(param, model_dir_);

  send_obj_srv_ = nh_.advertiseService("get_object",&DummyServer::sendObject,this);
  send_icr_srv_ = nh_.advertiseService("get_contact_regions",&DummyServer::sendICR,this);

  loadObject();
  loadICR();

}
//-----------------------------------------------------------------------------------
void DummyServer::loadObject()
{
  //Load the urdf file of the model (necessary for visualiztion in Rviz)
  std::string line;
  std::string model;
  std::ifstream file((model_dir_+ "Sprayflask_5k.urdf").c_str());
  
  if(!file.is_open())
    {
      ROS_ERROR("Could not open file");
      ROS_BREAK();
    }
  while(!file.eof()) // Parse the contents of the given urdf in a string
    {
      std::getline(file,line);
      model+=line;
    }
  file.close();

  //Push the loaded file on the parameter server 
  nh_.setParam("icr_object",model);

  //load the object
  rosbag::Bag bag(model_dir_+ "Sprayflask_5k.bag");
  rosbag::View view(bag, rosbag::TopicQuery("icr_object"));
  
  //It's actually assumed that the bag only contains one message
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      obj_ = m.instantiate<icr_msgs::Object>();
      if (obj_ == NULL)
  	{
  	  ROS_ERROR("Invalid bag file");
  	  ROS_BREAK();
  	}
    }
  bag.close();

}
//-----------------------------------------------------------------------------------
void DummyServer::loadICR()
{

  //load the icr
  rosbag::Bag bag(model_dir_+ "Sprayflask_5k_tripod_icr.bag");
  rosbag::View view(bag, rosbag::TopicQuery("icr"));
  
  //It's actually assumed that the bag only contains one message
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      icr_ = m.instantiate<icr_msgs::ContactRegions>();
      if (icr_ == NULL)
  	{
  	  ROS_ERROR("Invalid bag file");
  	  ROS_BREAK();
  	}
    }
  bag.close();
}
//-------------------------------------------------------------------------------
bool DummyServer::sendObject(icr_msgs::GetObject::Request  &req, icr_msgs::GetObject::Response &res)
{
  res.success=false;
  lock_.lock();


  res.object=(*obj_);
  lock_.unlock();

  res.success=true;
  return res.success;
}
//-------------------------------------------------------------------------------
bool DummyServer::sendICR(icr_msgs::GetContactRegions::Request  &req, icr_msgs::GetContactRegions::Response &res)
{
  res.success=false;
  lock_.lock();

  res.contact_regions=(*(icr_.get()));
  lock_.unlock();
 
  res.success=true;
  return res.success;
}
//-------------------------------------------------------------------------------
