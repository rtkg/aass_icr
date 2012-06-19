#include "grasp_affordances/grasp_affordances.h"


#include <pcl/ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <icr_msgs/ContactRegions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//--------------------------------------------------------------------------
GraspAffordances::GraspAffordances() : nh_private_("~"), obj_(new   pcl::PointCloud<pcl::PointNormal>),
                                       input_icr_(new pcl::PointCloud<pcl::PointNormal>), obj_set_(false),icr_set_(false)
{
  //An example of how to get stuff from the parameter server
  std::string search_param;

  nh_private_.searchParam("icr_database_directory", search_param);
  nh_private_.getParam(search_param,icr_dbase_dir_);

  //initialize Clients, Servers and Publishers
  load_icr_srv_ = nh_.advertiseService("load_icr",&GraspAffordances::loadIcr,this);
  compute_aff_srv_ = nh_.advertiseService("compute_affordances",&GraspAffordances::computeAffordances,this);
  set_obj_srv_ = nh_.advertiseService("set_object",&GraspAffordances::setObject,this);
  fetch_icr_srv_ = nh_.advertiseService("fetch_icr",&GraspAffordances::fetchIcr,this);
 
  get_icr_clt_ = nh_.serviceClient<icr_msgs::GetContactRegions>("get_icr");
  pts_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("output_regions",1);
  trans_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("transformed_regions",1);

}
//-------------------------------------------------------------------------------
bool GraspAffordances::setObject(icr_msgs::SetObject::Request &req, icr_msgs::SetObject::Response &res)
{
  res.success=false;  
  lock_.lock();

  obj_->clear();
  pcl::fromROSMsg(req.object.points,*obj_);//Convert the obtained message to PointCloud format
  obj_set_=true;
  res.success=true;

  //Making template of the obj_ and calc the features
  target_obj.loadInputCloud(obj_);

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

  input_icr_->clear();
  input_icr_->header.frame_id=get_icr.response.contact_regions.regions[0].points.header.frame_id;

  //concatenate the obtained regions to one point cloud (could probably be done more elegantly)
  pcl::PointCloud<pcl::PointNormal>::Ptr reg(new pcl::PointCloud<pcl::PointNormal>);
  for (unsigned int i=0; i<get_icr.response.contact_regions.regions.size();i++)
    {
      pcl::fromROSMsg(get_icr.response.contact_regions.regions[i].points,*reg);
      (*input_icr_)+=(*reg);
    }

  icr_set_=true;
  lock_.unlock();
  ROS_INFO("Fetched ICR on service: %s",get_icr_clt_.getService().c_str());
  return true;
}
//-------------------------------------------------------------------------------
void GraspAffordances::publish()
{
  lock_.lock();
  //Should publish the fitted regions, not the input regions - this is just an example
  if(input_icr_)
    {
      input_icr_->header.stamp=ros::Time(0);
      input_icr_->header.frame_id=input_icr_->header.frame_id;
      pts_pub_.publish(*input_icr_);
      //publish the regions from template_alignment
      transformed_cloud.header.stamp=ros::Time(0);
      transformed_cloud.header.frame_id=obj_->header.frame_id;
      trans_pub_.publish(transformed_cloud);
      
    }

  

  lock_.unlock();
}
//-------------------------------------------------------------------------------
bool GraspAffordances::computeAffordances(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
  lock_.lock();
  if(fitInputIcr()) {
    lock_.unlock();
    return true;
   }
   
   lock_.unlock();
   return false;

}
//-------------------------------------------------------------------------------
bool GraspAffordances::fitInputIcr()
{
  //Do the fitting stuff here
//  calcFeatures(input_icr_);

/* 	Call FeatureCloud Class
	FeatureCloud template_cloude;
	template_cloud.loadInputCloud(input_icr_); // Need to adjust the function
	*/
  
  //std::cerr << "PN ICR x: " << (*input_icr_).points[0].x << std::endl;
  
  //Calc local features
  //template_cloud.loadInputCloud(input_icr_);




 // Set template alignment input
  template_align.addTemplateCloud(template_cloud);
  template_align.setTargetCloud (target_obj);

  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = template_cloud;

// Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  //transformed_cloud = *best_template.getPointCloud ();
	std::cout << best_alignment.final_transformation << std::endl << std::endl;
  std::cerr << "transformed cloud: " << transformed_cloud << std::endl;
  std::cerr << "input icr #2: " << (*input_icr_) << std::endl;


  return true;
}
//-------------------------------------------------------------------------------
bool GraspAffordances::loadIcr(grasp_affordances::LoadIcr::Request  &req, grasp_affordances::LoadIcr::Response &res)
{
  res.success=false;  
  lock_.lock();
  ROS_INFO("Loading ICR from: %s",(icr_dbase_dir_ +req.file+ ".bag").c_str());
  //load the icr
  rosbag::Bag bag(icr_dbase_dir_+ req.file + ".bag");
  rosbag::View view(bag, rosbag::TopicQuery("contact_regions"));

  icr_msgs::ContactRegions::Ptr icr(new icr_msgs::ContactRegions);
  //It's actually assumed that the bag only contains one message
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      icr = m.instantiate<icr_msgs::ContactRegions>();
      if (input_icr_ == NULL)
  	{
  	  ROS_ERROR("Could not load %s",(icr_dbase_dir_ +req.file+ ".bag").c_str());
  	  lock_.unlock();
          return res.success;
  	}
    }
  bag.close();

  input_icr_->clear();
  input_icr_->header.frame_id=icr->regions[0].points.header.frame_id;

  //concatenate the obtained regions to one point cloud (could probably be done more elegantly)
  pcl::PointCloud<pcl::PointNormal>::Ptr reg(new pcl::PointCloud<pcl::PointNormal>);
  for (unsigned int i=0; i<icr->regions.size();i++)
    {
      pcl::fromROSMsg(icr->regions[i].points,*reg);
      (*input_icr_)+=(*reg);
    }

  icr_set_=true;

  //Calc local features
  template_cloud.loadInputCloud(input_icr_);
  std::cerr << "input icr #1: " << (*input_icr_) << std::endl;
  std::cerr << "header frame id: " << input_icr_->header.frame_id << std::endl;

  lock_.unlock();
  ROS_INFO("Loaded ICR from: %s",(icr_dbase_dir_ +req.file+ ".bag").c_str());
  res.success=true;
  return res.success;
}
//-------------------------------------------------------------------------------
