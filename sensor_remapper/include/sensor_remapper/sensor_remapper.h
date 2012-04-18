/**
 * @file   sensor_remapper.h
 * @author Robert Krug
 * @date   Wed Mar 21, 2012
* 
* Node remapping various contact sensor messages to a icr_msgs/ContactState message
*/

#ifndef   	sensor_remapper_h_
#define   	sensor_remapper_h_

#include <gazebo_msgs/ContactsState.h>
#include <kcl_msgs/KCL_ContactStateStamped.h>

/**
* @brief Node remapping various contact sensor messages to a icr_msgs/ContactState message. TODO: read
*the fully resolved frame ids from the parameter server and bind them to the callbacks 
*/

class SensorRemapper
{
 public:
 
  SensorRemapper();
  ~SensorRemapper(){};

 private:

  ros::NodeHandle nh_, nh_private_;
  ros::V_Subscriber sensor_subs_;
  ros::V_Publisher c_state_pubs_;
  ros::Subscriber createSubscriber(std::string const & name, std::string const &  type, unsigned int topic_id);


 /*  ///////////////// */
 /*  //  CALLBACKS  // */
 /*  ///////////////// */

  void remapGazeboMsgsContactsState(gazebo_msgs::ContactsState::ConstPtr msg, unsigned int topic_id);
  void remapKclMsgsKclContactStateStamped(kcl_msgs::KCL_ContactStateStamped::ConstPtr msg, unsigned int topic_id);
}; // end class


#endif 	
