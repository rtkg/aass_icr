/**
 * @file   sensor_remapper.h
 * @author Robert Krug
 * @date   Wed Mar 21, 2012
* 
* Node remapping various contact sensor messages to a icr/ContactState message
*/

#ifndef   	sensor_remapper_h_
#define   	sensor_remapper_h_

//#include "trajectory_parser.h"
//#include <Eigen/Core>
//#include <boost/thread/mutex.hpp>
//#include "../../icr/msg_gen/cpp/include/icr/ContactState.h"
#include <gazebo_msgs/ContactsState.h>
#include <kcl_msgs/KCL_ContactStateStamped.h>

class SensorRemapper
{
 public:
 
  SensorRemapper();
  ~SensorRemapper(){};

 private:

  ros::NodeHandle nh_, nh_private_;
  ros::V_Subscriber sensor_subs_;
  ros::V_Publisher c_state_pubs_;
 /*  static const unsigned int number_hand_joints_; */
 /*  TrajectoryParser* trajectory_parser_; */
 /*  boost::mutex data_mutex_; */

 /*  ros::ServiceServer replay_traj_srv_; */
 /*  ros::Publisher shadowhand_pub_; */

  ros::Subscriber createSubscriber(std::string const & name, std::string const &  type, unsigned int topic_id);

 /*  /\** */
 /*   * Returns a vector containing the joint angles of the hand at the given instance */
 /*   * */
 /*   * @param sample - between 1-N, where N is the number of samples for the trajectory */
 /*   *\/ */
 /*  Eigen::VectorXd getStateVector(unsigned int sample); */

 /* /\** */
 /*   * Creates a sendupdate message from the given joint state vector */
 /*   * */
 /*   * @param state_vec - vector containing the joint angles */
 /*   *\/ */
 /*  sr_robot_msgs::sendupdate generateMessage(Eigen::VectorXd & state_vec); */
  void initMessageTypes();

 /*  ///////////////// */
 /*  //  CALLBACKS  // */
 /*  ///////////////// */

 /*  bool replayTrajectory(sensor_remapper::replay_traj::Request &req, sensor_remapper::replay_traj::Response &res); */
  void remapGazeboMsgsContactsState(gazebo_msgs::ContactsState::ConstPtr msg, unsigned int topic_id);
  void remapKclMsgsKclContactStateStamped(kcl_msgs::KCL_ContactStateStamped::ConstPtr msg, unsigned int topic_id);
}; // end class


#endif 	
