#include "ros/ros.h"
#include "icr.h"
#include "icr/compute_icr.h"
#include "icr/load_object.h"
#include <sys/time.h>
#include <time.h>

bool compute(icr::compute_icr::Request  &req,
	     icr::compute_icr::Response &res)
{
  std::stringstream input;
  std::copy(req.centerpoint_ids.begin(),req.centerpoint_ids.end(), std::ostream_iterator<unsigned int>(input," ")); 
  ROS_INFO("Given centerpoint id's: %s", input.str().c_str());

  //Load a new target object 
  ICR::ObjectLoader obj_loader;
  obj_loader.loadObject("/home/rkg/ros/aass_icr/libicr/icrcpp/models/beer_can.obj","beer_can");
   
  //Create a list of 5 default finger parameters (default parameters defined in config.h) and a 
  //vector of centerpoint contact id's for the 5-fingered prototype grasp
  ICR::FParamList f_parameters;
  ICR::FingerParameters parameters;
   for (int i=0;i<5;i++)
     f_parameters.push_back(parameters);

   //Create a vector holding the centerpoint id's contained in the request
   ICR::VectorXui centerpoint_ids(req.centerpoint_ids.size());
   for(unsigned int i=0; i<req.centerpoint_ids.size();i++)
       centerpoint_ids(i)=req.centerpoint_ids[i];  //centerpoint_ids e.g. [1838, 4526, 4362, 1083, 793] for the cup

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

   if(!prototype_grasp->getGWS()->containsOrigin())
     {
       ROS_INFO("Given prototype grasp is not force closure - Not possible to compute ICR");
       res.success=false;
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



int main(int argc, char **argv)
{
  ros::init(argc, argv, "icr_srv");
  ros::NodeHandle n;

  ros::ServiceServer compute_icr_service = n.advertiseService("compute_icr", compute);
  ROS_INFO("Ready to compute ICR");
  ros::spin();

  return 0;
}
// include <PointCloudUtils.hh>
// #include <cstdio>
// #include <fstream>
// #include <Eigen/Eigen>

// #include "ros/ros.h"
// #include "pcl/point_cloud.h"
// #include "sensor_msgs/PointCloud2.h"
// #include "pcl/io/pcd_io.h"
// #include "message_filters/subscriber.h"
// #include "tf/message_filter.h"
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>

/// A shamelessly hardcoded logging node....

// class LoggerNode
// {
// protected:
//   // Our NodeHandle
//   ros::NodeHandle nh_;

//   ros::Subscriber kinect_;
//   ros::Subscriber sr_;
//   ros::Subscriber fotonic_;
//   ros::Subscriber laser_;
//   ros::Time lastLaser;

//   tf::TransformListener tfListener;
 
//   // Use the vector as a cyclic buffer (increment with std::rotate).
//   pcl::PointCloud<pcl::PointXYZ> sensor1_pc,sensor2_pc,sensor3_pc,sensor4_pc;

//   boost::mutex data_mutex;
//   bool s1_fresh, s2_fresh, s3_fresh, s4_fresh;
//   int dump_number;
//   std::string laserFrame, fotonicFrame, srFrame, kinectFrame;

// public:
//      // Constructor
//      LoggerNode() : s1_fresh(0), s2_fresh(0), s3_fresh(0), s4_fresh(0), dump_number(0)
//     {

// 	kinect_ = nh_.subscribe("/logger/kinect/points", 10, &LoggerNode::s1pts, this);
// 	sr_ = nh_.subscribe("/logger/sr/points", 10, &LoggerNode::s2pts, this);
// 	fotonic_ = nh_.subscribe("/logger/fotonic/points", 10, &LoggerNode::s3pts, this);
// 	laser_ = nh_.subscribe("/logger/laser/points", 10, &LoggerNode::s4pts, this);
	
// 	laserFrame = "/laser_frame";
// 	kinectFrame = "/openni_rgb_optical_frame";
// 	srFrame = "/camera";
// 	fotonicFrame = "/fotonic_frame";
// 	tfListener.setExtrapolationLimit(ros::Duration(1));
//     }
 
//   //dumps the fresh point clouds
//   void log();

//   // Callback
//   void s1pts(const sensor_msgs::PointCloud2::ConstPtr& msg_in);
//   void s2pts(const sensor_msgs::PointCloud2::ConstPtr& msg_in);
//   void s3pts(const sensor_msgs::PointCloud2::ConstPtr& msg_in);
//   void s4pts(const sensor_msgs::PointCloud2::ConstPtr& msg_in);

// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// };
