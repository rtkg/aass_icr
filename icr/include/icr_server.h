#ifndef icr_server_h___
#define icr_server_h___

#include "ros/ros.h"
#include "icr.h"
#include <string>

/* #include "../srv_gen/cpp/include/icr/compute_icr.h" */
#include "icr_msgs/SetObject.h"
#include "icr_msgs/ContactPoints.h"
#include "icr_msgs/SetSphericalQuality.h"
#include "icr_msgs/SetPhalangeParameters.h"
#include "icr_msgs/SetActivePhalanges.h"
#include <icr_msgs/ContactRegions.h>
#include <icr_msgs/ContactRegion.h>
#include <boost/thread/mutex.hpp>
/* #include <sensor_msgs/PointCloud2.h> */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

/** \class IcrServer icr_server.h 
 * \brief Server that computes Independent Contact Regions
 *
 * The icrServer can be parametrized in several ways:
 * \li A model of an object can be loaded \ref loadWfrontObj.  
 * \li The number of fingers can be specified \ref addFingers.
 * \li Each finger can be parametrized \ref setFingerParameters.
 * Finally, the icr can be computed using \ref computeIcr. 
 *
 * Assumption is that IcrServer, when properly parametrized, returns
 * indices of vertices that consist ICRs. Communication with IcrServer
 * is done through services.
 *
 * Example sequence of calling of services that leads to icr computation:
 * ....
 * 
 */

namespace ICR 
{
class IcrServer
{
 public:

  IcrServer();
  ~IcrServer(){};

  std::string getComputationMode();
  void computeSearchZones();
  void computeICR();
  void publish();

 private:

  //
  ros::NodeHandle nh_,nh_private_;
  
  
  XmlRpc::XmlRpcValue phalange_config_;
  TargetObjectPtr               obj_;
  GraspPtr                      pt_grasp_;
  SearchZonesPtr                sz_;
  IndependentContactRegionsPtr  icr_;
  
  
  bool obj_set_;
  bool pt_grasp_initialized_;
  bool gws_computed_;
  bool sz_computed_;
  bool icr_computed_;

  std::vector<std::string> active_phalanges_;
  
  std::string computation_mode_;
  double qs_;
  std::string obj_frame_id_;
  bool all_phl_touching_;

  ros::ServiceServer set_obj_srv_;
   ros::ServiceServer set_qs_srv_;
 ros::ServiceServer set_active_phl_srv_;
  ros::ServiceServer set_phl_param_srv_;
  ros::Subscriber ct_pts_sub_;
  ros::Publisher icr_cloud_pub_;
  ros::Publisher icr_pub_;

  boost::mutex lock_;
  icr_msgs::ContactRegions::Ptr icr_msg_;

  uint findClosestStupid(Eigen::Vector3d* point_in) const; 
  void getActivePhalangeParameters(FParamList & phl_param);
  unsigned int getPhalangeId(std::string const & name);
  void getFingerParameters(std::string const & name,FingerParameters & f_param);
  bool cpFromCptsMsg(icr_msgs::ContactPoints const & c_pts,const std::string & name,Eigen::Vector3d & contact_position,bool & touching);
  void initPtGrasp();  
  bool cloudFromContactRegion(unsigned int region_id,pcl::PointCloud<pcl::PointXYZRGB> & cloud, std::vector<unsigned int> & point_ids);
  

  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool setObject(icr_msgs::SetObject::Request  &req, icr_msgs::SetObject::Response &res);
 bool setSphericalQuality(icr_msgs::SetSphericalQuality::Request  &req, icr_msgs::SetSphericalQuality::Response &res);
 bool setActivePhalanges(icr_msgs::SetActivePhalanges::Request  &req, icr_msgs::SetActivePhalanges::Response &res);
 bool setPhalangeParameters(icr_msgs::SetPhalangeParameters::Request  &req, icr_msgs::SetPhalangeParameters::Response &res);
  void contactPointsCallback(icr_msgs::ContactPoints const & c_pts); 



};
}//end namespace



#endif
