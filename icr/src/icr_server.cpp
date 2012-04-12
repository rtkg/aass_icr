#include "../include/icr_server.h"
#include <sys/time.h>
#include <time.h>
// #include <ros/ros.h>
// #include <ros/subscribe_options.h>

#include <sensor_msgs/PointCloud2.h>
// #include "icr/ContactPoints.h"
// // PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <algorithm>
#include <Eigen/Core>

namespace ICR
{
//-------------------------------------------------------------------------
  IcrServer::IcrServer() : nh_private_("~"), obj_set_(false), pt_grasp_initialized_(false), gws_computed_(false),
                                             sz_computed_(false), icr_computed_(false), computation_mode_("continuous"), 
			   qs_(0.5),obj_frame_id_("/default"), all_phl_touching_(false), icr_msg_(new icr_msgs::ContactRegions)
{
  std::string searched_param;

  if(nh_private_.searchParam("computation_mode", searched_param))
    {
       nh_private_.getParam(searched_param, computation_mode_);
       if(!(!strcmp(computation_mode_.c_str(),"continuous") || !strcmp(computation_mode_.c_str(),"step_wise")))
	 {
	   ROS_WARN("%s is an invalid computation mode which has to be either 'continuous' or 'step_wise'. Using mode 'continuous' instead. ",computation_mode_.c_str());
	   computation_mode_="continuous";
         }
    }

  if (nh_private_.searchParam("phalanges", searched_param))
    {
      nh_.getParam(searched_param,phalange_config_);
      ROS_ASSERT(phalange_config_.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    
      for (int32_t i = 0; i < phalange_config_.size(); ++i) 
	{    
           ROS_ASSERT(phalange_config_[i]["phl_name"].getType() == XmlRpc::XmlRpcValue::TypeString); 
	   ROS_ASSERT(phalange_config_[i]["force_magnitude"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	   ROS_ASSERT(phalange_config_[i]["fc_disc"].getType() == XmlRpc::XmlRpcValue::TypeInt);
           ROS_ASSERT(phalange_config_[i]["mu_0"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
           ROS_ASSERT(phalange_config_[i]["mu_T"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
           ROS_ASSERT(phalange_config_[i]["contact_type"].getType() == XmlRpc::XmlRpcValue::TypeString); 
           ROS_ASSERT(phalange_config_[i]["model_type"].getType() == XmlRpc::XmlRpcValue::TypeString); 
           ROS_ASSERT(phalange_config_[i]["rule_type"].getType() == XmlRpc::XmlRpcValue::TypeString);
           ROS_ASSERT(phalange_config_[i]["rule_parameter"].getType() == XmlRpc::XmlRpcValue::TypeDouble);   
           ROS_ASSERT(phalange_config_[i]["filter_patch"].getType() == XmlRpc::XmlRpcValue::TypeBoolean);
           ROS_ASSERT(phalange_config_[i]["display_color"].getType() == XmlRpc::XmlRpcValue::TypeArray); 

           ROS_ASSERT(phalange_config_[i]["display_color"].size()==3); //3 rgb values to specify a color
	   for (int32_t j = 0; j < phalange_config_[i]["display_color"].size() ;j++) 
	     ROS_ASSERT(phalange_config_[i]["display_color"][j].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  

	   active_phalanges_.push_back((std::string)phalange_config_[i]["phl_name"]);//by default, all phalanges are considered in the icr computation
         }
    }
  else
    {
      ROS_ERROR("The hand phalange configurations are not specified - cannot start the Icr Server");
      ROS_BREAK();
    }

  set_obj_srv_ = nh_.advertiseService("set_object",&IcrServer::setObject,this);
  set_qs_srv_ = nh_.advertiseService("set_spherical_q",&IcrServer::setSphericalQuality,this);
  set_active_phl_srv_ = nh_.advertiseService("set_active_phalanges",&IcrServer::setActivePhalanges,this);
  set_phl_param_srv_ = nh_.advertiseService("set_phalange_parameters",&IcrServer::setPhalangeParameters,this);
  ct_pts_sub_ = nh_.subscribe("contact_points",1, &IcrServer::contactPointsCallback,this);
   icr_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("icr_cloud",5);
   icr_pub_ = nh_.advertise<icr_msgs::ContactRegions>("contact_regions",5);
}
//------------------------------------------------------------------------
void IcrServer::computeSearchZones()
{
  lock_.lock();

  if(!gws_computed_)
    {
      ROS_DEBUG("Grasp Wrench Space not computed - cannot compute search zones");
      lock_.unlock();
      return;
    }

  //cannot compute ICR if the prototype grasp is not force closure
  if(!pt_grasp_->getGWS()->containsOrigin())
    return;

  sz_.reset(new SearchZones(pt_grasp_));
  sz_->computeShiftedSearchZones(qs_);
  sz_computed_=true;
  lock_.unlock();
}
//-------------------------------------------------------------------------
void IcrServer::computeICR()
{
  lock_.lock();

  //need to check if the OWS list and the Patch list contained in the grasp are computed
  if(!pt_grasp_initialized_)
    {
      ROS_DEBUG("Prototype grasp not initialized - cannot compute ICR");
      lock_.unlock();
      return;
    }

  if(!sz_computed_)
    {
      ROS_DEBUG("Search zones not computed - cannot compute ICR");
      lock_.unlock();
      return;
    }
  
  //this could possibly done more efficiently with the setSearchZones and setGrasp methods
   icr_.reset(new IndependentContactRegions(sz_,pt_grasp_));
   icr_->computeICR();

   icr_computed_=true;

  lock_.unlock();
}
//-------------------------------------------------------------------------
void IcrServer::publish()
{
  lock_.lock();
  if(!icr_computed_)
    {
      ROS_DEBUG("ICR not computed - cannot publish");
      lock_.unlock();
      return;
    }

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  if(!generateCloudAndMessage(cloud,*icr_msg_))
    {
      ROS_ERROR("ICR  message and cloud generation unsuccessful - cannot publish ICR");
      lock_.unlock();
      return; 
    }

  
  icr_cloud_pub_.publish(cloud);
  icr_pub_.publish(*icr_msg_);
  lock_.unlock();
}
  //-------------------------------------------------------------------------
  bool IcrServer::generateCloudAndMessage(pcl::PointCloud<pcl::PointXYZRGB> & cloud,icr_msgs::ContactRegions & icr_msg)
  {
    cloud.clear();
    //need to check the vertices from the ICR grasp's parent obj whichs associated OWS list and
    //Patch list were used to compute the given ICR
    TargetObjectPtr parent_obj=icr_->getGrasp()->getParentObj();

    for(uint i=0 ; i<icr_->getNumContactRegions() ; i++) 
      {
	int phl_id = getPhalangeId(icr_->getGrasp()->getFinger(i)->getName());
	if(phl_id == -1)
	  {
	    ROS_ERROR("Cannot extract color associated with the phalange - valid phalange specifications are:");
	    for(int j=0; j<phalange_config_.size();j++)
	      ROS_INFO("%s",((std::string)phalange_config_[j]["phl_name"]).c_str());

            return false;         
	  }

	Eigen::Vector3d color;
	for (int32_t j = 0; j < 3 ;j++) 
	  color(j)=(double)phalange_config_[phl_id]["display_color"][j];
          
	for(uint j=0; j < icr_->getContactRegion(i)->size();j++) 
	  {
	    uint pt_id = icr_->getContactRegion(i)->at(j)->patch_ids_.front(); //get the patch centerpoint id
	    const Eigen::Vector3d* v = parent_obj->getContactPoint(pt_id)->getVertex();
	    pcl::PointXYZRGB pt;
	    pt.x=v->x(); pt.x=v->y();pt.x=v->z();
	    pt.r=color(0); pt.g=color(1); pt.b=color(2);

	    cloud.points.push_back(pt);
	  }
      }
  
    cloud.header.frame_id = obj_frame_id_;
    cloud.width=cloud.points.size();
    cloud.height = 1;
    cloud.is_dense=true;

    return true;
  }
//-------------------------------------------------------------------------
  bool IcrServer::setPhalangeParameters(icr_msgs::SetPhalangeParameters::Request  &req, icr_msgs::SetPhalangeParameters::Response &res)
  {
    res.success=false;
    lock_.lock();

    int phl_id =getPhalangeId(req.name);

    if(phl_id == -1)
      {
	ROS_ERROR("Cannot set phalange parameters - valid phalange specifications are:");
	for(int j=0; j<phalange_config_.size();j++)
	  ROS_INFO("%s",((std::string)phalange_config_[j]["phl_name"]).c_str());

	lock_.unlock();
	return res.success;
      }

    if(req.mu_0 <= 0.0)
    {
      ROS_ERROR("%f is an invalid friction coefficent. mu_0 has to be bigger than 0",req.mu_0);
      lock_.unlock();
      return res.success;
     }    

  if(req.fc_disc < 3)
    {
      ROS_ERROR("%d is an invalid friction cone discretization. fc_disc has to be bigger than 2",req.fc_disc);
      lock_.unlock();
      return res.success;
     }  

  phalange_config_[phl_id]["mu_0"]=req.mu_0;
  phalange_config_[phl_id]["fc_disc"]=req.fc_disc;

    //changing the active phalange parameters (its not actually checked whether the change was made on an active phalange) potentially requires changes in the associated OWS and Patch list. Thus, reinitialization is necessary
    if(pt_grasp_initialized_)
      initPtGrasp();

    lock_.unlock();
    res.success=true;
    return true;

  }
//-------------------------------------------------------------------------
bool IcrServer::setActivePhalanges(icr_msgs::SetActivePhalanges::Request & req, icr_msgs::SetActivePhalanges::Response &res)
{
  res.success=false;

  if(req.phalanges.size() < 2 )
    {
      ROS_ERROR("At least two contacting phalanges are necessary for a force closure grasp");
      return res.success;
   }

  lock_.lock();
 
  for(unsigned int i=0;i<req.phalanges.size();i++)
    {
      int phl_id=getPhalangeId(req.phalanges[i]);

      if(phl_id == -1)
	{
          ROS_ERROR("Cannot set active phalanges - valid phalange specifications are:");
           for(int j=0; j<phalange_config_.size();j++)
             ROS_INFO("%s",((std::string)phalange_config_[j]["phl_name"]).c_str());

	  lock_.unlock();
	  return res.success;
        }
    }

  active_phalanges_.clear();
  for(unsigned int i=0; i<req.phalanges.size();i++)
    active_phalanges_.push_back(req.phalanges[i]);

//changing the active phalanges requires changing the number of fingers in a grasp, thus reinitialzation of the prototype grasp is necessary
 if(pt_grasp_initialized_)
  initPtGrasp();

  lock_.unlock();
  res.success=true;

  return res.success;
}
//-------------------------------------------------------------------------
bool IcrServer::setObject(icr_msgs::SetObject::Request  &req, icr_msgs::SetObject::Response &res)
{
  res.success=false;
  lock_.lock();

  pcl::PointCloud<pcl::PointNormal> obj_cloud;
  pcl::fromROSMsg(req.object.points,obj_cloud);

  ROS_ASSERT(obj_cloud.size()==req.object.neighbors.size()); //make sure there are neighbors for each point in the cloud

   obj_.reset(new TargetObject(req.object.name));

    IndexList neighbors;
    for(unsigned int i=0;i<obj_cloud.size();i++)  
      {    
        neighbors.resize(req.object.neighbors[i].indices.size());
  	std::copy(req.object.neighbors[i].indices.begin(),req.object.neighbors[i].indices.end(),neighbors.begin());     
	obj_->addContactPoint(ContactPoint(Eigen::Vector3d(obj_cloud[i].x,obj_cloud[i].y,obj_cloud[i].z),Eigen::Vector3d(obj_cloud[i].normal[0],obj_cloud[i].normal[1],obj_cloud[i].normal[2]),neighbors,i));
      }

  //Loading a new object requires recomputing the OWS list and the Patch list which is done by creating a new grasp an initializing it
  initPtGrasp();

  obj_frame_id_=obj_cloud.header.frame_id;
  obj_set_=true;

  lock_.unlock();
  res.success=true;

  return res.success;
}
//------------------------------------------------------------------------
void IcrServer::initPtGrasp()
{
 if(!obj_set_)
    {
      ROS_ERROR("No object is loaded - cannot initialize the prototype grasp");
      return;
   }

    FParamList phl_param;
    getActivePhalangeParameters(phl_param);

    //The init function also requires centerpoint ids, here only a set of dummy ids is created so
    //that OWS list and Patch list can be computed. The init function actually also computes the GWS
    //(should be changed in the icrcpp library). However, since the computation was done with dummy
    //centerpoint ids, the gws_computed_ flag is set to false anyway. The actual GWS with proper
    //centerpoints will be computed in the getContactPoints callback
    VectorXui dummy_cp_ids(phl_param.size()); 
    dummy_cp_ids.setZero();

    pt_grasp_.reset(new Grasp);
    pt_grasp_->init(phl_param,obj_,dummy_cp_ids);

  
  pt_grasp_initialized_=true;
  gws_computed_=false;
  
}
  //------------------------------------------------------------------------
  bool IcrServer::setSphericalQuality(icr_msgs::SetSphericalQuality::Request  &req, icr_msgs::SetSphericalQuality::Response &res)
{
  res.success=false;
  lock_.lock();
  qs_=req.qs;
  lock_.unlock();
  res.success=true;
  return res.success;
}
//-------------------------------------------------------------------------
void IcrServer::contactPointsCallback(icr_msgs::ContactPoints const & c_pts)
{
  lock_.lock();
  if(!obj_set_ || !pt_grasp_initialized_) //do nothing if no object is loaded or the grasp is not initialized
    return;

  all_phl_touching_=true;
  
  Eigen::Vector3d contact_position;
  bool phl_touching=false; 
  VectorXui centerpoint_ids(active_phalanges_.size());

  for (unsigned int i=0; i<centerpoint_ids.size();i++)
    {
      if(!cpFromCptsMsg(c_pts,active_phalanges_[i],contact_position,phl_touching))
	{
	  ROS_ERROR("Cannot compute the Grasp Wrench Space");
          all_phl_touching_=false;
	  return;
	}

      if(!phl_touching)
	all_phl_touching_=false;      
 
      centerpoint_ids(i)=findClosestStupid(&contact_position);
    }

  pt_grasp_->setCenterPointIds(centerpoint_ids);
  gws_computed_=true;

  lock_.unlock();
}
//-------------------------------------------------------------------------
  bool IcrServer::cpFromCptsMsg(icr_msgs::ContactPoints const & c_pts, const std::string & name,Eigen::Vector3d & contact_position,bool & touching)
{

  for(unsigned int i=0; i<c_pts.points.size();i++)
      if(!strcmp(name.c_str(),c_pts.points[i].phalange.c_str()))
	{
          contact_position(0)=c_pts.points[i].position.x;
          contact_position(1)=c_pts.points[i].position.y;
          contact_position(2)=c_pts.points[i].position.z;
          touching=c_pts.points[i].touching;
          return true;
        }
    
  ROS_ERROR("Could not find contact point corresponding to phalange %s in ContactPoints message",name.c_str());
  return false;
}
//-------------------------------------------------------------------------
void IcrServer::getActivePhalangeParameters(FParamList & phl_param)
{
  phl_param.resize(active_phalanges_.size());
 
  FingerParameters f_param;

  
  for(unsigned int i=0; i<active_phalanges_.size();i++)
    {
      getFingerParameters(active_phalanges_[i],f_param);
      phl_param[i]=f_param;
    }  
}
//--------------------------------------------------------------------------
unsigned int IcrServer::getPhalangeId(std::string const & name)
{
   int phl_id= -1;

    for(int i=0; i<phalange_config_.size();i++)
      if(!strcmp(name.c_str(),((std::string)phalange_config_[i]["phl_name"]).c_str()))    
	phl_id=i;
           
    if(phl_id == -1)
      {
	ROS_ERROR("Could not find phalange with name %s",name.c_str());
	return phl_id;
      }
    return phl_id;
}
//-------------------------------------------------------------------------
  void IcrServer::getFingerParameters(std::string const & name,FingerParameters & f_param)
  {
    int phl_id=getPhalangeId(name);
    if(phl_id == -1)
      return;
      
    std::string string_param;

    f_param.setForce((double)phalange_config_[phl_id]["force_magnitude"]);
    f_param.setFriction((double)phalange_config_[phl_id]["mu_0"]);
    f_param.setFrictionTorsional((double)phalange_config_[phl_id]["mu_T"]);
    f_param.setDiscretization((int)phalange_config_[phl_id]["fc_disc"]);

    string_param=(std::string)phalange_config_[phl_id]["phl_name"];
    f_param.setName(string_param);
    string_param=(std::string)phalange_config_[phl_id]["contact_type"];
    f_param.setContactType(string_param);
    string_param=(std::string)phalange_config_[phl_id]["model_type"];
    f_param.setContactModelType(string_param);
    string_param=(std::string)phalange_config_[phl_id]["rule_type"];
    f_param.setInclusionRuleType(string_param);

    f_param.setInclusionRuleParameter((double)phalange_config_[phl_id]["rule_parameter"]);
    f_param.setInclusionRuleFilterPatch((bool)phalange_config_[phl_id]["filter_patch"]);
  }
//---------------------------------------------------------------------------
uint IcrServer::findClosestStupid(Eigen::Vector3d* point_in) const 
{
  double min_dist = 100000;
  uint closest_idx_out = 0;
  const Eigen::Vector3d* point_on_object;
  Eigen::Vector3d pt_tmp;
  unsigned int size = obj_->getNumCp();

  for (uint i=0; i<size ; ++i) {
    point_on_object = obj_->getContactPoint(i)->getVertex();
    pt_tmp = *point_in - *point_on_object;
    double dist = pt_tmp.norm();
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx_out = i;
    }
  }
  return closest_idx_out;

}
//--------------------------------------------------------------------------------------------
std::string IcrServer::getComputationMode()
{
  std::string computation_mode;

  lock_.lock();
  computation_mode=computation_mode_;
  lock_.unlock();

  return computation_mode;
}
//--------------------------------------------------------------------------------------------
}//end namespace
