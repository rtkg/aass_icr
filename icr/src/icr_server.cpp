#include "../include/icr_server.h"
#include <sys/time.h>
#include <time.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <sensor_msgs/PointCloud2.h>
#include "icr/ContactPoints.h"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using std::cout;
using std::endl;

//-------------------------------------------------------------------------
IcrServer::IcrServer() : nh_private_("~"), 
			 obj_loader_(new ICR::ObjectLoader()), 
			 finger_parameters_(new ICR::FParamList()),
			 icr_(new ICR::IndependentContactRegions()), 
			 grasp_update_needed_(true),
			 alpha_shift_(1.0)
{
  std::string param;
  std::string prefix;
  nh_private_.searchParam("icr_prefix", param);
  nh_private_.param(param, prefix, std::string());

  compute_icr_service_=nh_.advertiseService(prefix + "/icr_server/compute_icr",&IcrServer::computeIcr,this);
  load_wfront_obj_service_=nh_.advertiseService(prefix + "/icr_server/load_wfront_obj",&IcrServer::loadWfrontObj,this);
  set_finger_number_service_=nh_.advertiseService(prefix + "/icr_server/set_finger_number",&IcrServer::setFingerNumber,this);
  set_finger_parameters_service_=nh_.advertiseService(prefix + "/icr_server/set_finger_parameters",&IcrServer::setFingerParameters,this);

  pub_icr_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> (prefix + "/icr_server/icr_cloud", 10);

  // ros::SubscribeOptions test;
  
  // test.init("/finger_tips", 100, fingerTipCallback);

  //  ros::Subscriber test1 
  sub_finger_tips_ = nh_.subscribe(prefix + "/finger_tips", 
				   100, &IcrServer::fingerTipCallback,this);

  //  sub_finger_tips_ = nh_.subscribe(test); 

  nh_private_.getParam("alpha_shift", alpha_shift_);

  loadFingerParameters();
}
//-------------------------------------------------------------------------
IcrServer::~IcrServer()
{
  delete obj_loader_;
  delete finger_parameters_;
  delete icr_;
}

void IcrServer::publishCloud() {
  pub_icr_cloud_.publish(output_cloud_);
}

void IcrServer::fingerTipCallback(const icr::ContactPoints& msg) {

  if(msg.points.size() != finger_parameters_->size()) {
    ROS_WARN("fingerTipCallback : number of parametrized fingers on icr_server is different than number of fingers in a message. ");
    return;
  }
    
  uint n_finger = finger_parameters_->size();
  //  uint* centerpoint_ids_tmp = new uint[n_finger];
  ICR::VectorXui centerpoint_ids(n_finger);


  ROS_INFO("Received fingertips positions:");
  for (uint i=0; i<n_finger; i++) {
    ROS_INFO("I've heard: [%f, %f, %f]", msg.points[i].position.x,
	     msg.points[i].position.y, msg.points[i].position.z);
    Eigen::Vector3d point_in(msg.points[i].position.x,
			     msg.points[i].position.y,
			     msg.points[i].position.z);
    centerpoint_ids[i] = findClosestStupid(&point_in);
    ROS_INFO("Found closest vertex on the object: %d.",centerpoint_ids[i]);
  }

  //calculate ICRs 
  if (computeIcrCore(centerpoint_ids) ) {
    publishICRpc();
  }
  //  delete [] centerpoint_ids_tmp;

}

//-----------------------------------------------------------------------
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
  grasp_update_needed_ = true;

  for(unsigned int i=0; i<req.number; i++) {
    finger_parameters_->push_back(default_finger_params_);
  }
  
  ROS_INFO("Hand has now %d fingers",finger_parameters_->size());
  res.success=true;
  data_mutex_.unlock();
  return res.success;
}

//----------------------------------------------------------------------
bool IcrServer::setFingerParameters(icr::set_finger_parameters::Request &req, icr::set_finger_parameters::Response &res)// from terminal call e.g: 
//rosservice call /icr_server/set_finger_parameters '{parameter_list: [1]}'
{


  // data_mutex_.lock();
  // if (req.parameter_list.size() > 9)
  //   {
  //     ROS_INFO("Maximal 8 parameters can be configurated for the given finger_id");
  //     res.success=false;
  //     data_mutex_.unlock();
  //     return res.success;
  //   }
  // // char f_id=(char)req.parameter_list[0];
  // //std::cout<<"finger id:  "<< f_id<<std::endl;
  // std::basic_string<char> bs=req.parameter_list[0];
  // std::cout<<"bs size: "<<bs.size()<<std::endl;
  // char bs1=bs[0];
  // unsigned int bs1i=(unsigned int)bs1;

  // std::cout<<"bs1: "<<bs1<<" bs1i: "<<bs1i<<std::endl;
  // class std::vector<std::basic_string<char> > rec=req.parameter_list;
  // for (unsigned int i=0;i<req.parameter_list.size();i++)
  //   {
  //     std::cout<<"hello"<<req.parameter_list.size()<<std::endl;
  //  std::cout<<"rec: "<<req.parameter_list[i]<<std::endl;
  //   }

  // update_icr_ = true;

  // res.success=true;
  // data_mutex_.unlock();
  // return res.success;
  return true;
}

bool IcrServer::computeIcr(icr::compute_icr::Request  &req, 
			   icr::compute_icr::Response &res) {
  //Create a vector holding the centerpoint id's contained in the request
  ICR::VectorXui centerpoint_ids(req.centerpoint_ids.size());

  bool all_good = true;

  for(unsigned int i=0; i<req.centerpoint_ids.size();i++) {
    if (req.used[i]){
      //centerpoint_ids e.g. [1838, 4526, 4362, 1083, 793] for the cup
      centerpoint_ids(i)=req.centerpoint_ids[i];  
    }
  }

  if (computeIcrCore(centerpoint_ids)) 
    {
      std::cout << "This is output" << *icr_ << std::endl;
      
      if(icr_->icrComputed() ) { 
	// reserve memory in the output vector
	uint16_t number = 0; // number of elements in all ICRs
	for(uint i=0 ; i<icr_->getNumContactRegions() ; i++) {
	  number += icr_->getContactRegion(i)->size();
	}
	res.all_icrs.reserve(number);
	
	// fill in ICRs into output vector
	for(uint i=0 ; i<icr_->getNumContactRegions() ; i++) { 
	  res.stx.push_back( res.all_icrs.size() );
	  res.len.push_back( icr_->getContactRegion(i)->size() );
	  for(uint j=0; j < icr_->getContactRegion(i)->size();j++) {
	    res.all_icrs.push_back(icr_->getContactRegion(i)->at(j)->patch_ids_.front());
	  }
	}
	publishICRpc();
      }
    } else {
    all_good = false;
  }

  res.success = icr_->icrComputed();
  return all_good;
}



bool IcrServer::computeIcrCore(ICR::VectorXui &centerpoint_ids)
{
  bool all_good = true;

  data_mutex_.lock();
  if(!obj_loader_->objectLoaded())    {
    ROS_INFO("A valid target object needs to be loaded prior to the ICR computation");
    all_good = false;
  }
  if(finger_parameters_->size() < 2)    {
    ROS_INFO("Hand has to comprise at least 2 fingers in order to allow computing ICR");
    all_good = false;
  }
  if(finger_parameters_->size() != (uint) centerpoint_ids.size())    {
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

    if (grasp_update_needed_) { 
      grasp_update_needed_ = false;
      ICR::GraspPtr prototype_grasp(new ICR::Grasp());
      prototype_grasp->init(*finger_parameters_,
			    obj_loader_->getObject(),
			    centerpoint_ids);
      icr_->setGrasp(prototype_grasp);    
    } else if ( icr_->hasInitializedGrasp() ) {
      //set new center points/select a subset of centerpoints
      icr_->getGrasp()->setCenterPointIds(centerpoint_ids);
    } else {
      ROS_ERROR("Something went terribly wrong. No grasp update needed and grasp is not initialized. ");
      exit(0);
    }

    if( !icr_->getGrasp()->getGWS()->containsOrigin() ) {
      ROS_INFO("Given prototype grasp is not force closure - Not possible to compute ICR");
    } else {
      ICR::SearchZonesPtr search_zones(new ICR::SearchZones(icr_->getGrasp()));
      search_zones->computeShiftedSearchZones(alpha_shift_);
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


void IcrServer::publishICRpc() {

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
	//       	std::cout << (float)pt_e->x() <<" "<< (float)pt_e->x() /1000.0 << std::endl;
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
  pcl::toROSMsg(*cloud,output_cloud_);

  publishCloud();
}

// when a marker is missing the default value for all fingers is set
bool IcrServer::loadFingerParameters() {

  bool all_good = true;
  XmlRpc::XmlRpcValue finger_config;
  
  nh_private_.getParam("icr_fingers_param", finger_config);

  ROS_INFO("Updating fingers...");
  assert(finger_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
  if (all_good) {
    for (int i = 0; i<finger_config.size(); ++i) {
      ICR::FingerParameters* finger = 
	new ICR::FingerParameters(default_finger_params_);
      updateOneFinger(finger_config[i],finger);
      cout << "Loaded the parametrization of a finger number [" 
	   << i << "]." << *finger  << endl;
      finger_parameters_->push_back(*finger);
      if (i==0) { //first finger in the config file
	default_finger_params_ = *finger;
      }
    }

  }
  ROS_INFO("Hand has now %d fingers",finger_parameters_->size());
  return true;
}


bool IcrServer::updateOneFinger(XmlRpc::XmlRpcValue &finger_config, 
				ICR::FingerParameters *finger_out) {
  bool all_good = true;
  assert(finger_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  if (finger_config.hasMember("finger_name") && 
      finger_config["finger_name"].getType() == XmlRpc::XmlRpcValue::TypeString) {
    finger_out->setName(finger_config["finger_name"]);
  }
  if (finger_config.hasMember("force_magnitude") && 
      finger_config["force_magnitude"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    finger_out->setForce(finger_config["force_magnitude"]);
  }
  if (finger_config.hasMember("fc_disc") && 
      finger_config["fc_disc"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
    finger_out->setDiscretization((int)finger_config["fc_disc"]);
  }
  if (finger_config.hasMember("mu_0") && 
      finger_config["mu_0"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    finger_out->setFriction(finger_config["mu_0"]);
  }
  if (finger_config.hasMember("mu_T") && 
      finger_config["mu_T"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    finger_out->setFrictionTorsional(finger_config["mu_T"]);
  }
  if (finger_config.hasMember("contact_type") && 
      finger_config["contact_type"].getType() == XmlRpc::XmlRpcValue::TypeString) {
    finger_out->setContactType(finger_config["contact_type"]);
  }
  if (finger_config.hasMember("model_type") && 
      finger_config["model_type"].getType() == XmlRpc::XmlRpcValue::TypeString) {
    finger_out->setContactModelType(finger_config["model_type"]);
  }
  if (finger_config.hasMember("filter_patch") && 
      finger_config["filter_patch"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    finger_out->setInclusionRuleFilterPatch(finger_config["filter_patch"]);
  }
  if (finger_config.hasMember("radius") && 
      finger_config["radius"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    finger_out->setInclusionRuleParameter((int)finger_config["radius"]);
  }
 
  return all_good;
}

uint IcrServer::findClosestStupid(Eigen::Vector3d* point_in) const {
  double min_dist = 100000;
  uint closest_idx_out = 0;
  const Eigen::Vector3d* point_on_object;
  Eigen::Vector3d pt_tmp;
  unsigned int size = obj_loader_->getObject()->getNumCp();

  for (uint i=0; i<size ; ++i) {
    point_on_object = obj_loader_->getObject()->getContactPoint(i)->getVertex();
    pt_tmp = *point_in - *point_on_object;
    double dist = pt_tmp.norm();
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx_out = i;
    }
  }
  return closest_idx_out;

}

//     for(uint i=0 ; i<icr_->getNumContactRegions() ; i++) { 
//       for(uint j=0; j < icr_->getContactRegion(i)->size();j++) {
// 	uint pt_id = icr_->getContactRegion(i)->at(j)->patch_ids_.front();
// 	const Eigen::Vector3d* pt_e = obj_loader_->getObject()->getContactPoint(pt_id)->getVertex();
// 	//       	std::cout << (float)pt_e->x() <<" "<< (float)pt_e->x() /1000.0 << std::endl;
// 	pcl::PointXYZ pt((float)pt_e->x()*scale,
// 			 (float)pt_e->y()*scale,
// 			 (float)pt_e->z()*scale);
// 	cloud->points.push_back(pt);
//       }
//     }


//   pt_out = this->getPoint(idx);
//   //  std::cout << "findClosestStupid: Found point:  ("<< idx 
//   //        << ") [" << pt_out->x() << " "  << pt_out->y() << " " << pt_out->z() 
//   //        << "] min dist: " << min_dist << std::endl;
//   return idx;
// }

