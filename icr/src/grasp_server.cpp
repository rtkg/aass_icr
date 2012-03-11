#include "../include/grasp_server.h"
#include <tf/tf.h>

GraspServer::GraspServer() : nh_private_("~")
{
  // std::string icr_prefix;
  // std::string gazebo_prefix;
  // std::string param;
  // nh_private_.searchParam("icr_prefix", param);
  // nh_private_.getParam(param, icr_prefix);
std::string searched_param;

 
 XmlRpc::XmlRpcValue phalange_config;
 XmlRpc::XmlRpcValue L_T_Cref;
	 Model phalange_model;
 if (nh_private_.searchParam("phalanges", searched_param))
   {
     nh_.getParam(searched_param,phalange_config);
     ROS_ASSERT(phalange_config.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    
     for (int32_t i = 0; i < phalange_config.size(); ++i) 
       {    
         L_T_Cref=phalange_config[i]["L_T_Cref"];
	 ROS_ASSERT(phalange_config[i]["link_geom"].getType() == XmlRpc::XmlRpcValue::TypeString);
	 ROS_ASSERT(phalange_config[i]["link_frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
         ROS_ASSERT(phalange_config[i]["link_name"].getType() == XmlRpc::XmlRpcValue::TypeString);
	 ROS_ASSERT(L_T_Cref.getType() == XmlRpc::XmlRpcValue::TypeArray);
         ROS_ASSERT(L_T_Cref.size()==12);//a transformation is specified by a rotation matrix + offset vector
	 for (int32_t j = 0; j <L_T_Cref.size();j++) 
	    ROS_ASSERT(L_T_Cref[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);

	 tf::Transform tf;
	 tf.setBasis(btMatrix3x3(L_T_Cref[0],L_T_Cref[1],L_T_Cref[2],L_T_Cref[3],L_T_Cref[4],L_T_Cref[5],L_T_Cref[6],L_T_Cref[7],L_T_Cref[8]));
	 tf.setOrigin(tf::Vector3(L_T_Cref[9],L_T_Cref[10],L_T_Cref[11]));

	 phalange_model.name_=(std::string)phalange_config[i]["link_name"];
          phalange_model.geom_=(std::string)phalange_config[i]["link_geom"];
          phalange_model.frame_id_=(std::string)phalange_config[i]["link_frame_id"];

	 phalanges_.push_back(new Phalange(tf,phalange_model));
       }
   }
 else
   {
     ROS_ERROR("The hand phalange configurations are not specified - cannot start the Grasp Server");
     exit(0);
   }

  // for(int i=0; i<phalanges_.size();i++)
  //   std::cout<<"geom: "<<*(phalanges_[i]->getPhalangeGeom())<<std::endl;
  // load_object_srv_ = nh_.advertiseService(icr_prefix + "/load_object",&GraspServer::loadGrasp,this);
  // gazebo_spawn_clt_ = nh_.serviceClient<gazebo_msgs::SpawnGrasp>(gazebo_prefix + "/spawn_urdf_model");
  // gazebo_delete_clt_ = nh_.serviceClient<gazebo_msgs::DeleteGrasp>(gazebo_prefix + "/delete_model");
  // gazebo_pause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix + "/pause_physics");
  // gazebo_unpause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix + "/unpause_physics");

  phalange_contacts_subs_.push_back(nh_.subscribe<gazebo_msgs::ContactsState>("chatter", 5, boost::bind(&GraspServer::listenContacts, this, _1, 42)));
}
//-----------------------------------------------------------------------------------------------
GraspServer::~GraspServer()
{
  for (unsigned int i=0; i<phalanges_.size();i++)
    delete phalanges_[i];
}
//-----------------------------------------------------------------------------------------------
// 8	class Listener
// 39	{
// 40	public:
// 41	  ros::NodeHandle node_handle_;
// 42	  ros::V_Subscriber subs_;
// 43	
// 44	  Listener(const ros::NodeHandle& node_handle)
// 45	  : node_handle_(node_handle)
// 46	  {
// 47	  }
// 48	
// 49	  void init()
// 50	  {
// 51	    subs_.push_back
// 52	    subs_.push_back(node_handle_.subscribe<std_msgs::String>("chatter", 1000, boost::bind(&Listener::chatterCallback, this, _1, "User 2")));
// 53	    subs_.push_back(node_handle_.subscribe<std_msgs::String>("chatter", 1000, boost::bind(&Listener::chatterCallback, this, _1, "User 3")));
// 54	  }
// 55	
// 56	  void chatterCallback(const std_msgs::String::ConstPtr& msg, std::string user_string)
// 57	  {
// 58	    ROS_INFO("I heard: [%s] with user string [%s]", msg->data.c_str(), user_string.c_str());
// 59	  }
// 60	};
// 61	
// 62	int main(int argc, char **argv)
// 63	{
// 64	  ros::init(argc, argv, "listener_with_userdata");
// 65	  ros::NodeHandle n;
// 66	
// 67	  Listener l(n);
// 68	  l.init();
// 69	
// 70	  ros::spin();
// 71	
// 72	  return 0;
// 73	}
