/**
 * @file   sensor_remapper.cpp
 * @author Robert Krug
 * @date   Fri Feb 24, 2012
 *
 */
#include <ros/ros.h>
#include <string>
#include "sensor_remapper/sensor_remapper.h"
#include "../../icr/msg_gen/cpp/include/icr/ContactState.h"

//-------------------------------------------------------------------
SensorRemapper::SensorRemapper() : nh_private_("~")
{
 std::string searched_param;
 XmlRpc::XmlRpcValue sensor_topics;
 //  XmlRpc::XmlRpcValue L_T_Cref;
 //  Model phalange_model;
   if (nh_private_.searchParam("sensor_topics", searched_param))
     {
       nh_.getParam(searched_param,sensor_topics);
       ROS_ASSERT(sensor_topics.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    
       for (int32_t i = 0; i < sensor_topics.size(); ++i) 
  	{    
 // 	  L_T_Cref=sensor_topics[i]["L_T_Cref"];
  	 ROS_ASSERT(sensor_topics[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
         ROS_ASSERT(sensor_topics[i]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
         ROS_ASSERT(sensor_topics[i]["remap_name"].getType() == XmlRpc::XmlRpcValue::TypeString);

	 sensor_subs_.push_back(createSubscriber((std::string)sensor_topics[i]["name"],(std::string)sensor_topics[i]["type"],i));
	 c_state_pubs_.push_back(nh_.advertise<icr::ContactState> ((std::string)sensor_topics[i]["remap_name"], 1));

	 // sensor_subs_.push_back(nh_.subscribe((std::string)sensor_topics[i]["name"], 1, &SensorRemapper::remapGazeboMsgsContactsState, this));
          

 // 	  ROS_ASSERT(sensor_topics[i]["link_frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
 // 	  ROS_ASSERT(sensor_topics[i]["link_name"].getType() == XmlRpc::XmlRpcValue::TypeString);
 //          ROS_ASSERT(sensor_topics[i]["sensor_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
 // 	  ROS_ASSERT(L_T_Cref.getType() == XmlRpc::XmlRpcValue::TypeArray);
 // 	  ROS_ASSERT(L_T_Cref.size()==6);//a transformation is specified by an offset vector + RPY values
 // 	  for (int32_t j = 0; j <L_T_Cref.size();j++) 
 // 	    ROS_ASSERT(L_T_Cref[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);

 // 	  tf::Transform tf;
 //          tf.setOrigin(tf::Vector3(L_T_Cref[0],L_T_Cref[1],L_T_Cref[2]));
 // 	  tf.setRotation(tf::createQuaternionFromRPY(L_T_Cref[3],L_T_Cref[4],L_T_Cref[5]));
	 
 // 	  phalange_model.name_=(std::string)sensor_topics[i]["link_name"];
 //          phalange_model.geom_=(std::string)sensor_topics[i]["link_geom"];
 //          phalange_model.frame_id_=(std::string)sensor_topics[i]["link_frame_id"];

 // 	  phalanges_.push_back(new Phalange(tf,phalange_model,(std::string)sensor_topics[i]["sensor_topic"]));

       }
    }
  else
    {
      ROS_ERROR("The sensor remapping configurations are not specified - cannot start the Sensor remapper");
      exit(0);
    }
   exit(0);
 
  // initMessageTypes();
   //gazebo_modstat_sub_ = nh_.subscribe(gazebo_prefix + "/model_states", 10, &ModelServer::getModelStates, this);
  //  replay_traj_srv_ = nh_.advertiseService(server_prefix + "replay_traj",&SensorRemapper::replayTrajectory,this);
  // shadowhand_pub_ = nh_.advertise<sr_robot_msgs::sendupdate> (sendupdate_prefix + "sendupdate", 1); //queue size of only one - maybe change that
}
//-------------------------------------------------------------------
// SensorRemapper::~SensorRemapper(){delete trajectory_parser_;}
//-------------------------------------------------------------------
ros::Subscriber SensorRemapper::createSubscriber(std::string const & name, std::string const & type, unsigned int topic_id)
{
  ros::Subscriber sub;

  if (!strcmp(type.c_str(),"gazebo_msgs/ContactsState"))
    sub=nh_.subscribe<gazebo_msgs::ContactsState>(name, 1, boost::bind(&SensorRemapper::remapGazeboMsgsContactsState, this,_1,topic_id));
  else if (!strcmp(type.c_str(),"kcl_msgs/KCL_ContactStateStamped"))
    sub=nh_.subscribe<kcl_msgs::KCL_ContactStateStamped>(name, 1, boost::bind(&SensorRemapper::remapKclMsgsKclContactStateStamped, this,_1,topic_id));
  else
    ROS_WARN("%s is an unregistered message type - cannot remap topic %s.",type.c_str(),name.c_str());

  return sub;
}
//-------------------------------------------------------------------
void SensorRemapper::remapGazeboMsgsContactsState(gazebo_msgs::ContactsState::ConstPtr msg, unsigned int topic_id)
{
  icr::ContactState state;
  std::cout<<"ladida"<<std::endl;

  c_state_pubs_[topic_id].publish(state);
}
//-------------------------------------------------------------------
void SensorRemapper::remapKclMsgsKclContactStateStamped(kcl_msgs::KCL_ContactStateStamped::ConstPtr msg, unsigned int topic_id)
{
 icr::ContactState state;
  std::cout<<"ladida"<<std::endl;

  c_state_pubs_[topic_id].publish(state);
}
//-------------------------------------------------------------------
// sr_robot_msgs::sendupdate SensorRemapper::generateMessage(Eigen::VectorXd & state_vec)
// {
//   sr_robot_msgs::joint joint;
//   sr_robot_msgs::sendupdate msg;
   
//   //Generate sendupdate message
//   std::vector<sr_robot_msgs::joint> table(number_hand_joints_);
//   for(unsigned int i = 0; i < number_hand_joints_; ++i )
//     {
//       joint.joint_name = message_types_[i];
//       joint.joint_target = state_vec.coeff(i);
//       table[i] = joint;
//     }

//   msg.sendupdate_list = table;
//   msg.sendupdate_length = number_hand_joints_;

//   return msg;
// }
// //-------------------------------------------------------------------
// bool SensorRemapper::replayTrajectory(sensor_remapper::replay_traj::Request &req, sensor_remapper::replay_traj::Response &res)
// {
//   data_mutex_.lock();
//   res.success=false;
 
//   //Load the trajectory in the parser
//   if(!trajectory_parser_->parseFile(req.file))
//     {
//       ROS_ERROR("Could not parse file %s",(trajectory_parser_->getTrajDir()+req.file).c_str());
//       data_mutex_.unlock();
//       return res.success;
//     }

//   if(!trajectory_parser_->getNumTraj()==number_hand_joints_)
//     {
//       ROS_ERROR("%d number of joint trajectories need to be specified",number_hand_joints_);
//       data_mutex_.unlock();
//       return res.success;
//     }
  
//   Eigen::VectorXd state_vec(number_hand_joints_);
//   unsigned int sample_id=0;
//   ros::Rate r(1/trajectory_parser_->getTimestep()); 

//   unsigned int num_samples=trajectory_parser_->getNumSamples();

//   while (ros::ok() && (sample_id < num_samples))
//   {
//     state_vec=getStateVector(sample_id);
//     shadowhand_pub_.publish(generateMessage(state_vec));
//     sample_id+=1;
//     r.sleep();
//     //std::cout<<r.cycleTime()<<std::endl;
//   }

//   res.success=true;
//   data_mutex_.unlock();
//   return res.success;
// }
// //-------------------------------------------------------------------
// Eigen::VectorXd SensorRemapper::getStateVector(unsigned int sample)
// {
//   return trajectory_parser_->getTrajectories()->block<number_hand_joints_,1>(0,sample);
// }
// //-------------------------------------------------------------------



