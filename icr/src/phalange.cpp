#include "../include/phalange.h"
#include <geometry_msgs/PoseStamped.h>



Phalange::Phalange(tf::Transform const & L_T_Cref,std::string const & link_geom) : L_T_Cref_(L_T_Cref),ct_pos_(new tf::Vector3), ct_ori_(new tf::Quaternion),
                                                                                       link_geom_(new std::string(link_geom))
{
  ct_pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>(*link_geom_+"/contact_pose",1);
}
//---------------------------------------------------------------------
Phalange::~Phalange()
{
  delete ct_pos_;
  delete ct_ori_;
  delete link_geom_;
}
//---------------------------------------------------------------------
void Phalange::updateContact(gazebo_msgs::ContactsState::ConstPtr& cts_st, std::string const & obj_geom)
{
  if(cts_st->states.empty())
    return;
}
//---------------------------------------------------------------------
tf::Vector3* Phalange::getContactPositon(){return ct_pos_;}
//---------------------------------------------------------------------
tf::Quaternion* Phalange::getContactOrientation(){return ct_ori_;}
//---------------------------------------------------------------------
std::string* Phalange::getPhalangeGeom(){return link_geom_;}
//---------------------------------------------------------------------
