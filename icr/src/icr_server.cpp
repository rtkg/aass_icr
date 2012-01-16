#include "ros/ros.h"
#include "std_msgs/String.h"
#include "icr.h"
#include <sstream>
#include <sys/time.h>
#include <time.h>
#include "icr/compute_icr.h"

bool compute(icr::compute_icr::Request  &req,
	     icr::compute_icr::Response &res )
{
  std::cout<<"In compute function"<<std::endl;


  ROS_INFO("sending back response: [%u]", (bool)res.success);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "icr_srv");
  ros::NodeHandle n;

 
  ros::ServiceServer service = n.advertiseService("compute_icr", compute);
  ROS_INFO("Ready to compute icr.");


  //Load a new target object 
  ICR::ObjectLoader obj_loader;
  obj_loader.loadObject("/home/rkg/ros/aass_icr/libicr/icrcpp/models/beer_can.obj","beer_can");
   
  //Create a list of 5 default finger parameters (default parameters defined in config.h) and a 
  //vector of centerpoint contact id's for the 5-fingered prototype grasp
  ICR::FParamList f_parameters;
  ICR::FingerParameters parameters;
   for (int i=0;i<5;i++)
     f_parameters.push_back(parameters);

   ICR::VectorXui centerpoint_ids(5);
   centerpoint_ids << 1838, 4526, 4362, 1083, 793;

   //Modify some of the finger parameters of the prototype-grasp.
   //The third finger utilizies the Multi-Point contact model, patches only contain the center-point
   //and border points
   f_parameters[0].setSoftFingerContact(1,6,0.5,0.5);
   f_parameters[2].setContactModelType(ICR::Multi_Point);
   f_parameters[2].setInclusionRuleFilterPatch(true);

   //Create a prototype grasp and search zones, the parameter alpha is the scale of the largest
   //origin-centered ball contained by the Grasp wrench space of the prototype grasp
   ICR::GraspPtr prototype_grasp(new ICR::Grasp());
   prototype_grasp->init(f_parameters,obj_loader.getObject(),centerpoint_ids);
   ICR::SearchZonesPtr search_zones(new ICR::SearchZones(prototype_grasp));
   double alpha=0.5;
   search_zones->computeShiftedSearchZones(alpha);

   //Create and plot the Independent Contact Regions
   ICR::IndependentContactRegions icr(search_zones,prototype_grasp);
   icr.computeICR();
   std::cout<<icr;

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

  ros::spin();

  return 0;
}
