/**
 * @author Robert Krug
 *
 */

#ifndef dummy_server_h___
#define dummy_server_h___

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <string>
#include "icr_msgs/Object.h"
#include <sensor_msgs/PointCloud2.h>
#include <icr_msgs/ContactRegions.h>
#include <icr_msgs/GetObject.h>
#include <icr_msgs/GetContactRegions.h>
//----------------------------------------------------------------------------
class DummyServer
{
 public:

  DummyServer();
  ~DummyServer(){};

 private:

  ros::NodeHandle nh_, nh_private_;
  boost::mutex lock_;
  std::string model_dir_;
  icr_msgs::Object::Ptr obj_;
  icr_msgs::ContactRegions::Ptr icr_;

  ros::ServiceServer send_obj_srv_;
  ros::ServiceServer send_icr_srv_;

  void loadObject();
  void loadICR();
  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool sendObject(icr_msgs::GetObject::Request  &req, icr_msgs::GetObject::Response &res); 
  bool sendICR(icr_msgs::GetContactRegions::Request  &req, icr_msgs::GetContactRegions::Response &res);

};
#endif
