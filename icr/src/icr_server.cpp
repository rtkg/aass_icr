#include "../include/icr_server.h"
#include <sys/time.h>
#include <time.h>

#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//-------------------------------------------------------------------------
IcrServer::IcrServer() : nh_private_("~"), 
			 obj_loader_(new ICR::ObjectLoader()), 
			 finger_parameters_(new ICR::FParamList()), 
			 icr_(new ICR::IndependentContactRegions()), 
			 update_fingers_(true)
{
  std::string param;
  std::string prefix;
  nh_private_.searchParam("icr_prefix", param);
  nh_private_.param(param, prefix, std::string());

  compute_icr_service_=nh_.advertiseService(prefix + "/icr_server/compute_icr",&IcrServer::computeIcr,this);
  load_wfront_obj_service_=nh_.advertiseService(prefix + "/icr_server/load_wfront_obj",&IcrServer::loadWfrontObj,this);
  set_finger_number_service_=nh_.advertiseService(prefix + "/icr_server/set_finger_number",&IcrServer::setFingerNumber,this);
  set_finger_parameters_service_=nh_.advertiseService(prefix + "/icr_server/set_finger_parameters",&IcrServer::setFingerParameters,this);

  pub_icr_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> (prefix + "/icr_server/icr_cloud", 1, true);
}
//-------------------------------------------------------------------------
IcrServer::~IcrServer()
{
  delete obj_loader_;
  delete finger_parameters_;
  delete icr_;
}
//-------------------------------------------------------------------------
bool IcrServer::setFingerNumber(icr::set_finger_number::Request &req, 
				icr::set_finger_number::Response &res)
{
  data_mutex_.lock();
  
  if(req.number < 1)
    {
      ROS_INFO("At least one finger has to be added");
      res.success=false;
      data_mutex_.unlock();
      return res.success;  
    }
  
  ROS_INFO("Setting %d new fingers with default parameters.",req.number);

  finger_parameters_->clear();
  update_fingers_ = true;

  ICR::FingerParameters default_param;
  for(unsigned int i=0; i<req.number; i++) 
    {
      finger_parameters_->push_back(default_param);
    }
  
  ROS_INFO("Hand has now %d fingers",finger_parameters_->size());
  res.success=true;
  data_mutex_.unlock();
  return res.success;
}

//-------------------------------------------------------------------------
bool IcrServer::setFingerParameters(icr::set_finger_parameters::Request &req, icr::set_finger_parameters::Response &res)// from terminal call e.g: 
//rosservice call /icr_server/set_finger_parameters '{parameter_list: [1]}'
{
  data_mutex_.lock();
  if (req.parameter_list.size() > 9)
    {
      ROS_INFO("Maximal 8 parameters can be configurated for the given finger_id");
      res.success=false;
      data_mutex_.unlock();
      return res.success;
    }
  // char f_id=(char)req.parameter_list[0];
  //std::cout<<"finger id:  "<< f_id<<std::endl;
  std::basic_string<char> bs=req.parameter_list[0];
  std::cout<<"bs size: "<<bs.size()<<std::endl;
  char bs1=bs[0];
  unsigned int bs1i=(unsigned int)bs1;

  std::cout<<"bs1: "<<bs1<<" bs1i: "<<bs1i<<std::endl;
  class std::vector<std::basic_string<char> > rec=req.parameter_list;
  for (unsigned int i=0;i<req.parameter_list.size();i++)
    {
      std::cout<<"hello"<<req.parameter_list.size()<<std::endl;
   std::cout<<"rec: "<<req.parameter_list[i]<<std::endl;
    }

  update_fingers_ = true;

  res.success=true;
  data_mutex_.unlock();
  return res.success;

}

bool IcrServer::computeIcr(icr::compute_icr::Request  &req, 
			   icr::compute_icr::Response &res) 
{
  //Create a vector holding the centerpoint id's contained in the request
  ICR::VectorXui centerpoint_ids(req.centerpoint_ids.size());

  bool all_good = true;

  for(unsigned int i=0; i<req.centerpoint_ids.size();i++) 
    {
      if (req.used[i])
	{
	  //centerpoint_ids e.g. [1838, 4526, 4362, 1083, 793] for the cup
	  centerpoint_ids(i)=req.centerpoint_ids[i];  
	}
    }

  if (computeIcrCore(centerpoint_ids)) 
    {
      std::cout << "This is output" << *icr_ << std::endl;
      
      if(icr_->icrComputed() )
	{ 
	  // reserve memory in the output vector
	  uint16_t number = 0; // number of elements in all ICRs
	  for(uint i=0 ; i<icr_->getNumContactRegions() ; i++) 
	    {
	      number += icr_->getContactRegion(i)->size();
	    }
	  res.all_icrs.reserve(number);
	  // fill in ICRs into output vector
	  for(uint i=0 ; i<icr_->getNumContactRegions() ; i++)
	    { 
	      res.stx.push_back( res.all_icrs.size() );
	      res.len.push_back( icr_->getContactRegion(i)->size() );
	      for(uint j=0; j < icr_->getContactRegion(i)->size();j++)
		{
		  res.all_icrs.push_back(icr_->getContactRegion(i)->at(j)->patch_ids_.front());
		}
	    }
	  publishICRpc(res);
	}
    } 
  else 
    {
      all_good = false;
    }

  res.success = icr_->icrComputed();
  return all_good;
}



bool IcrServer::computeIcrCore(ICR::VectorXui &centerpoint_ids)
{
  bool all_good = true;

  data_mutex_.lock();
  if(!obj_loader_->objectLoaded())
    {
      ROS_INFO("A valid target object needs to be loaded prior to the ICR computation");
      all_good = false;
    }
  if(finger_parameters_->size() < 2)
    {
      ROS_INFO("Hand has to comprise at least 2 fingers in order to allow computing ICR");
      all_good = false;
    }
  if(finger_parameters_->size() != (uint) centerpoint_ids.size())
    {
      ROS_INFO("The number of given centerpoint id's has to equal the number of fingers on the hand in order to allow computing ICR");
      all_good =false;
    }
  
  if (all_good) {
    
    //  std::stringstream input;
    //std::copy(req.centerpoint_ids.begin(),req.centerpoint_ids.end(), std::ostream_iterator<unsigned int>(input," ")); 
    //    ROS_INFO("Computing ICR for object: %s with given centerpoint id's: %s",
    //	     (obj_loader_->getObject()->getName()).c_str(),input.str().c_str());

    //Modify some of the finger parameters of the prototype-grasp.
    //The third finger utilizies the Multi-Point contact model, patches only contain the center-point
    //and border points
    //(*finger_parameters_)[0].setSoftFingerContact(1,6,0.5,0.5);
    //(*finger_parameters_)[2].setContactModelType(ICR::Multi_Point);
    // (*finger_parameters_)[2].setInclusionRuleFilterPatch(true);

    //Create a prototype grasp and search zones, the parameter alpha
    //is the scale of the largest origin-centered ball contained by
    //the Grasp wrench space of the prototype grasp
    ICR::GraspPtr prototype_grasp(new ICR::Grasp());
    prototype_grasp->init(*finger_parameters_,obj_loader_->getObject(),centerpoint_ids);

    if(!prototype_grasp->getGWS()->containsOrigin() )
      {
	ROS_INFO("Given prototype grasp is not force closure - Not possible to compute ICR");
	all_good=false;
      } 
    else 
      {
	ICR::SearchZonesPtr search_zones(new ICR::SearchZones(prototype_grasp));
	double alpha=0.5;
	search_zones->computeShiftedSearchZones(alpha);

	//Create and plot the Independent Contact Regions
	icr_->setGrasp(prototype_grasp);
	icr_->setSearchZones(search_zones);

	icr_->computeICR();
	std::cout << *icr_;
	
      }
  }
  data_mutex_.unlock();

  return all_good;

  // Utility for timing selected parts of the code - uncomment below and put the code to be timed at the marked location
  // struct timeval start, end;
  // double c_time;
  // gettimeofday(&start,0);
  //
  //  Put code to be timed here...
  //  
  // gettimeofday(&end,0);
  // c_time = end.tv_sec - start.tv_sec + 0.000001 * (end.tv_usec - start.tv_usec);
  // std::cout<<"Computation time: "<<c_time<<" s"<<std::endl;
}


bool IcrServer::loadWfrontObj(icr::load_object::Request  &req, icr::load_object::Response &res) // call from terminal with e.g. following arguments: '{path: /home/rkg/ros/aass_icr/libicr/icrcpp/models/beer_can.obj, name: beer_can}'
{
   data_mutex_.lock();
   obj_loader_->loadObject(req.path,req.name);
   if((obj_loader_->objectLoaded()) & (obj_loader_->getObject()->getNumCp() > 0))
     res.success=true;
   else
     {
     ROS_INFO("The loaded target object must be valid. From a terminal type eg. rosservice call /icr_server/load_wfront_obj '{path: /home/username/models/beer_can.obj, name: beer_can}' ");
     res.success=false;
     }
  data_mutex_.unlock();
  return res.success;
}


void IcrServer::publishICRpc(icr::compute_icr::Response &res) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>() ); 
  
  double scale = 0.001;
  
  if ( icr_->icrComputed() ) {
    //clear pc
    cloud->clear();
    //fill in
    for(uint i=0 ; i<icr_->getNumContactRegions() ; i++) { 
      for(uint j=0; j < icr_->getContactRegion(i)->size();j++) {
	uint pt_id = icr_->getContactRegion(i)->at(j)->patch_ids_.front();
	const Eigen::Vector3d* pt_e = obj_loader_->getObject()->getContactPoint(pt_id)->getVertex();
       	std::cout << (float)pt_e->x() <<" "<< (float)pt_e->x() /1000.0 << std::endl;
	pcl::PointXYZ pt((float)pt_e->x()*scale,
			 (float)pt_e->y()*scale,
			 (float)pt_e->z()*scale);
	cloud->points.push_back(pt);
      }
    }
  }
  cloud->header.frame_id = "fixed";
  cloud->width = cloud->points.size();
  cloud->height = 1;
  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(*cloud,output_cloud);
  pub_icr_cloud_.publish(output_cloud);
}
