#include "../include/phalange.h"
#include <geometry_msgs/PoseStamped.h>



Phalange::Phalange(tf::Transform const & L_T_Cref,Model const & model) : L_T_Cref_(L_T_Cref),ct_pos_(new tf::Vector3), ct_ori_(new tf::Quaternion),
									 model_(new Model(model))
{
  ct_pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>(model_->name_ + "/contact_pose",1);
}
//---------------------------------------------------------------------
Phalange::~Phalange()
{
  delete ct_pos_;
  delete ct_ori_;
  delete model_;
}
//---------------------------------------------------------------------
void Phalange::updateContact(gazebo_msgs::ContactsState::ConstPtr& cts_st, std::string const & obj_geom)
{
  if(cts_st->states.empty())
    return;
}
//---------------------------------------------------------------------
Model* Phalange::getModel(){return model_;}
//---------------------------------------------------------------------
tf::Vector3* Phalange::getContactPositon(){return ct_pos_;}
//---------------------------------------------------------------------
tf::Quaternion* Phalange::getContactOrientation(){return ct_ori_;}
//---------------------------------------------------------------------
