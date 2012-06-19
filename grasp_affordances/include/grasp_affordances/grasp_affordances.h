/**
 * @author Robert Krug
 * @date   Sat Apr 7 2012
 *
 */

#ifndef grasp_affordances_h___
#define grasp_affordances_h___

#include "grasp_affordances/featurecloud.h"
#include "grasp_affordances/template_alignment.h"

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <icr_msgs/SetObject.h>
#include <grasp_affordances/LoadIcr.h>
#include <icr_msgs/GetContactRegions.h>
#include <std_srvs/Empty.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

/**
 * @brief Class fitting a point set (in form of Independent Contact Regions) to discrete object models 
 *
 */
class GraspAffordances
{
 public:

  GraspAffordances();
  ~GraspAffordances(){};

  void publish();

 private:

  //private node handle for accessing the parameter server, publich nh for other stuff
  ros::NodeHandle nh_, nh_private_;
  //ROS runs in threads -> necessary to maintain mutex locks
  boost::mutex lock_;

 /**
 * @brief Service for setting an object
 *
 */
  ros::ServiceServer set_obj_srv_;
 /**
 * @brief Service which wraps a client call to the connected icr server to get ICR's
 *
 */
  ros::ServiceServer fetch_icr_srv_;
 /**
 * @brief Service to fit the input regions to the object model
 *
 */
  ros::ServiceServer compute_aff_srv_;
 /**
 * @brief Service loading ICR's from the specified database directory
 *
 */
  ros::ServiceServer load_icr_srv_;
 /**
 * @brief Client which calls the connected icr server to get ICR's
 *
 */
  ros::ServiceClient get_icr_clt_;
 /**
 * @brief Point cloud publisher
 *
 */
  ros::Publisher     pts_pub_;
  ros::Publisher     trans_pub_;

 /**
 * @brief boost::shared_ptr to the object point cloud
 *
 */
  pcl::PointCloud<pcl::PointNormal>::Ptr obj_;
 /**
 * @brief boost::shared_ptr to the concatenated input icr cloud
 *
 */
  pcl::PointCloud<pcl::PointNormal>::Ptr input_icr_;

  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;

  bool obj_set_;  
  bool icr_set_;

  FeatureCloud target_obj;
  FeatureCloud template_cloud;
  TemplateAlignment template_align;

 /**
 * @brief string holding the path to the ICR database directory - loaded from the parameter server
 * in the constructor of GraspAffordances::GraspAffordances()
 *
 */
  std::string icr_dbase_dir_;

  bool fitInputIcr();

  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool loadIcr(grasp_affordances::LoadIcr::Request  &req, grasp_affordances::LoadIcr::Response &res);
  bool computeAffordances(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
  bool setObject(icr_msgs::SetObject::Request  &req, icr_msgs::SetObject::Response &res);
  bool fetchIcr(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

};
//----------------------------------------------------------------------------
#endif
