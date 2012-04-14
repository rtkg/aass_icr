#include "grasp_affordances/grasp_affordances.h"
#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

//--------------------------------------------------------------------------
GraspAffordances::GraspAffordances() : nh_private_("~"), obj_(new   pcl::PointCloud<pcl::PointNormal>),
                                       input_icr_(new pcl::PointCloud<pcl::PointXYZRGB>), obj_set_(false),icr_set_(false)
{
  //An example of how to get stuff from the parameter server
  std::string search_param;

  nh_private_.searchParam("icr_database_directory", search_param);
  nh_private_.getParam(search_param,icr_dbase_dir_);

  //initialize Clients, Servers and Publishers
  compute_aff_srv_ = nh_.advertiseService("compute_affordances",&GraspAffordances::computeAffordances,this);
  set_obj_srv_ = nh_.advertiseService("set_object",&GraspAffordances::setObject,this);
  fetch_icr_srv_ = nh_.advertiseService("fetch_icr",&GraspAffordances::fetchIcr,this);
 
  get_icr_clt_ = nh_.serviceClient<icr_msgs::GetContactRegions>("get_icr");
  pts_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("output_regions",1);

}
//-------------------------------------------------------------------------------
bool GraspAffordances::setObject(icr_msgs::SetObject::Request  &req, icr_msgs::SetObject::Response &res)
{
  res.success=false;  
  lock_.lock();

  obj_->clear();
  pcl::fromROSMsg(req.object.points,*obj_);//Convert the obtained message to PointCloud format
  obj_set_=true;
  res.success=true;

  lock_.unlock();

  ROS_INFO("Object %s set in the grasp_affordances node",req.object.name.c_str());
  return res.success;
}
//-------------------------------------------------------------------------------
bool GraspAffordances::fetchIcr(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  icr_msgs::GetContactRegions get_icr;
  lock_.lock();
 
  get_icr_clt_.call(get_icr);
  if(!get_icr.response.success)
    {
      ROS_ERROR("Get contact regions client call unsuccessful");
      lock_.unlock();
      return false;
    }

  //concatenate the obtained regions to one point cloud (could probably be done more elegantly)
  pcl::PointCloud<pcl::PointXYZRGB> icr;
  pcl::PointCloud<pcl::PointXYZRGB> reg;
  icr.header.frame_id=get_icr.response.contact_regions.regions[0].points.header.frame_id;

  lock_.lock();
  for (unsigned int i=0; i<get_icr.response.contact_regions.regions.size();i++)
    {
      pcl::fromROSMsg(get_icr.response.contact_regions.regions[i].points,reg);
      icr+=reg;
    }

  input_icr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>(icr));
  icr_set_=true;
  lock_.unlock();

  return true;
}
//-------------------------------------------------------------------------------
void GraspAffordances::publish()
{
  //Should publish the fitted regions, not the input regions - this is just an example
  if(input_icr_)
    {
      input_icr_->header.stamp=ros::Time(0);
      input_icr_->header.frame_id="/Sprayflask_5k";
      pts_pub_.publish(*input_icr_);
    }
}
//-------------------------------------------------------------------------------
bool GraspAffordances::computeAffordances(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  if(fitInputIcr())
    return true;
  else
    return false;
}
//-------------------------------------------------------------------------------
bool GraspAffordances::fitInputIcr()
{
  //Do the fitting stuff here

  return true;
}
//-------------------------------------------------------------------------------
