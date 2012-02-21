#include "../include/icr_server.h"
#include <sys/time.h>
#include <time.h>

//----------------------------------------------------------------------------------------
IcrServer::IcrServer() : obj_loader_(new ICR::ObjectLoader()), finger_parameters_(new ICR::FParamList())
  {
    compute_icr_service_=nh_.advertiseService("/icr_server/compute_icr",&IcrServer::computeIcr,this);
    load_wfront_obj_service_=nh_.advertiseService("/icr_server/load_wfront_obj",&IcrServer::loadWfrontObj,this);
    add_fingers_service_=nh_.advertiseService("/icr_server/add_fingers",&IcrServer::addFingers,this);
    set_finger_parameters_service_=nh_.advertiseService("/icr_server/set_finger_parameters",&IcrServer::setFingerParameters,this);
  }
//----------------------------------------------------------------------------------------
IcrServer::~IcrServer()
{
  delete obj_loader_;
  delete finger_parameters_;
}
//----------------------------------------------------------------------------------------
bool IcrServer::addFingers(icr::add_fingers::Request &req, icr::add_fingers::Response &res)
{
  data_mutex_.lock();
  
  if(req.number < 1)
    {
      ROS_INFO("At least one finger has to be added");
      res.success=false;
      data_mutex_.unlock();
      return res.success;  
    }

  ROS_INFO("Adding %d new fingers",req.number);

  ICR::FingerParameters default_param;
  for(unsigned int i=0; i<req.number; i++)
    finger_parameters_->push_back(default_param);

  ROS_INFO("Hand has now %d fingers",finger_parameters_->size());
  res.success=true;
  data_mutex_.unlock();
  return res.success;
}
//----------------------------------------------------------------------------------------
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
  res.success=true;
  data_mutex_.unlock();
  return res.success;

}
//----------------------------------------------------------------------------------------
bool IcrServer::computeIcr(icr::compute_icr::Request  &req, icr::compute_icr::Response &res)//call from terminal with e.g. the following arguments: "centerpoint_ids: [1,2,3,4,5]"
{
  data_mutex_.lock();
  if(!obj_loader_->objectLoaded())
    {
      ROS_INFO("A valid target object needs to be loaded prior to the ICR computation");
      res.success=false;
      data_mutex_.unlock();
      return res.success;
    }
  else if(finger_parameters_->size() < 2)
    {
      ROS_INFO("Hand has to comprise at least 2 fingers in order to allow computing ICR");
      res.success=false;
      data_mutex_.unlock();
      return res.success;
    }
  else if(finger_parameters_->size() != req.centerpoint_ids.size())
    {
      ROS_INFO("The number of given centerpoint id's has to equal the number of fingers on the hand in order to allow computing ICR");
      res.success=false;
      data_mutex_.unlock();
      return res.success;
    }

  //  std::stringstream input;
  //std::copy(req.centerpoint_ids.begin(),req.centerpoint_ids.end(), std::ostream_iterator<unsigned int>(input," ")); 
  //ROS_INFO("Computing ICR for object: %s with given centerpoint id's: %s",(obj_loader_->getObject()->getName()).c_str(),input.str().c_str());

   //Create a vector holding the centerpoint id's contained in the request
   ICR::VectorXui centerpoint_ids(req.centerpoint_ids.size());
   for(unsigned int i=0; i<req.centerpoint_ids.size();i++)
       centerpoint_ids(i)=req.centerpoint_ids[i];  //centerpoint_ids e.g. [1838, 4526, 4362, 1083, 793] for the cup

   //Modify some of the finger parameters of the prototype-grasp.
   //The third finger utilizies the Multi-Point contact model, patches only contain the center-point
   //and border points
   //(*finger_parameters_)[0].setSoftFingerContact(1,6,0.5,0.5);
   //(*finger_parameters_)[2].setContactModelType(ICR::Multi_Point);
   // (*finger_parameters_)[2].setInclusionRuleFilterPatch(true);

   //Create a prototype grasp and search zones, the parameter alpha is the scale of the largest
   //origin-centered ball contained by the Grasp wrench space of the prototype grasp
   ICR::GraspPtr prototype_grasp(new ICR::Grasp());
   prototype_grasp->init(*finger_parameters_,obj_loader_->getObject(),centerpoint_ids);

   if(!prototype_grasp->getGWS()->containsOrigin())
     {
       ROS_INFO("Given prototype grasp is not force closure - Not possible to compute ICR");
       res.success=false;
       data_mutex_.unlock();
       return res.success;
     }

   ICR::SearchZonesPtr search_zones(new ICR::SearchZones(prototype_grasp));
   double alpha=0.5;
   search_zones->computeShiftedSearchZones(alpha);

   //Create and plot the Independent Contact Regions
   ICR::IndependentContactRegions icr(search_zones,prototype_grasp);
   icr.computeICR();
   std::cout<<icr;

   res.success=true;
   data_mutex_.unlock();
   return res.success;

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
//----------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------

