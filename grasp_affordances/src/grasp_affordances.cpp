#include "grasp_affordances/grasp_affordances.h"
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

//--------------------------------------------------------------------------
GraspAffordances::GraspAffordances() : nh_private_("~"), obj_(new   pcl::PointCloud<pcl::PointNormal>),
                                       input_icr_(new pcl::PointCloud<pcl::PointXYZRGB>)
{

  compute_aff_srv_ = nh_.advertiseService("compute_affordances",&GraspAffordances::computeAffordances,this);
  fetch_obj_srv_ = nh_.advertiseService("fetch_object",&GraspAffordances::fetchObject,this);
  fetch_icr_srv_ = nh_.advertiseService("fetch_contact_regions",&GraspAffordances::fetchICR,this);
  get_obj_clt_ = nh_.serviceClient<icr_msgs::GetObject>("get_object");
  get_icr_clt_ = nh_.serviceClient<icr_msgs::GetContactRegions>("get_contact_regions");
  pts_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("output_regions",1);

}
//-------------------------------------------------------------------------------
bool GraspAffordances::fetchObject(icr_msgs::GetObject::Request  &req, icr_msgs::GetObject::Response &res)
{
  res.success=false;
  
  icr_msgs::GetObject obj;
  obj.request=req;


  get_obj_clt_.call(obj); 
  if(!obj.response.success)
    {
      ROS_ERROR("Get object client call unsuccessful");
      return res.success;
    }

  res=obj.response;

  lock_.lock();
  obj_->clear();
  pcl::fromROSMsg(res.object.points,*obj_);
  lock_.unlock();

  return res.success;
}
//-------------------------------------------------------------------------------
bool GraspAffordances::fetchICR(icr_msgs::GetContactRegions::Request  &req, icr_msgs::GetContactRegions::Response &res)
{
  res.success=false;
  
  icr_msgs::GetContactRegions srv;
  srv.request=req;

  get_icr_clt_.call(srv); 
  if(!srv.response.success)
    {
      ROS_ERROR("Get contact regions client call unsuccessful");
      return res.success;
    }

  res=srv.response;
  pcl::PointCloud<pcl::PointXYZRGB> icr;
  pcl::PointCloud<pcl::PointXYZRGB> reg;
  icr.header.frame_id=res.contact_regions.regions[0].points.header.frame_id;

  lock_.lock();
  for (unsigned int i=0; i<res.contact_regions.regions.size();i++)
    {
      pcl::fromROSMsg(res.contact_regions.regions[i].points,reg);
      icr+=reg;
    }

  input_icr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(icr));
  lock_.unlock();

  return res.success;
}
//-------------------------------------------------------------------------------
void GraspAffordances::publish()
{
  if(input_icr_)
    {
      input_icr_->header.stamp=ros::Time(0);
      pts_pub_.publish(*input_icr_);
    }
}
//-------------------------------------------------------------------------------
bool computeAffordances(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{

  return true;
}
//-------------------------------------------------------------------------------
