#include "../include/icr_server.h"
// #include <sys/time.h>
// #include <time.h>
#include <sensor_msgs/PointCloud2.h>
#include <algorithm>
#include <Eigen/Core>
#include <rosbag/bag.h>
#include <eigen_conversions/eigen_msg.h>

namespace ICR
{
  //-------------------------------------------------------------------------
  IcrServer::IcrServer() : nh_private_("~"), obj_set_(false), pt_grasp_initialized_(false), gws_computed_(false),
			   sz_computed_(false), icr_computed_(false), computation_mode_(MODE_CONTINUOUS), 
			   qs_(1e-10),computation_frame_id_("/default"), icr_msg_(new icr_msgs::ContactRegions), marker_size_(0.003), 
                           compute_in_palm_frame_(false), O_F_T_(Eigen::Affine3d::Identity())
  {
    std::string searched_param;

    nh_private_.searchParam("icr_database_directory", searched_param);
    nh_private_.getParam(searched_param, icr_database_dir_);

    nh_private_.searchParam("compute_in_palm_frame", searched_param);
    nh_private_.getParam(searched_param, compute_in_palm_frame_);

    nh_private_.searchParam("marker_size", searched_param);
    nh_private_.getParam(searched_param, marker_size_);

    if(nh_private_.searchParam("computation_mode", searched_param))
      {
	nh_private_.getParam(searched_param, computation_mode_);
	if(!(computation_mode_== MODE_CONTINUOUS  || computation_mode_ == MODE_STEP_WISE))
	  {
	    ROS_WARN("%d is an invalid computation mode which has to be either  0 (MODE_CONTINUOUS) or 1 (MODE_STEP_WISE). Using MODE_CONTINUOUS ... ",computation_mode_);
	    computation_mode_=MODE_CONTINUOUS;
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

    get_icr_srv_ = nh_.advertiseService("get_icr",&IcrServer::getIcr,this); 
    compute_sz_srv_ = nh_.advertiseService("compute_search_zones",&IcrServer::triggerSearchZonesCmp,this); 
    compute_icr_srv_ = nh_.advertiseService("compute_icr",&IcrServer::triggerIcrCmp,this); 
    set_mode_srv_ = nh_.advertiseService("set_computation_mode",&IcrServer::setComputationMode,this); 
    save_icr_srv_ = nh_.advertiseService("save_icr",&IcrServer::saveIcr,this); 
    set_obj_srv_ = nh_.advertiseService("set_object",&IcrServer::setObject,this);
    set_qs_srv_ = nh_.advertiseService("set_spherical_q",&IcrServer::setSphericalQuality,this);
    set_active_phl_srv_ = nh_.advertiseService("set_active_phalanges",&IcrServer::setActivePhalanges,this);
    set_phl_param_srv_ = nh_.advertiseService("set_phalange_parameters",&IcrServer::setPhalangeParameters,this);
    ct_pts_sub_ = nh_.subscribe("grasp",1, &IcrServer::graspCallback,this);
    icr_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("icr_cloud",1);
    icr_pub_ = nh_.advertise<icr_msgs::ContactRegions>("contact_regions",1);
    sp_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("seed_points",1);
    //dummy_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dummy_obj",1);
  }
  //------------------------------------------------------------------------
  void IcrServer::computeSearchZones()
  {
    lock_.lock();

    sz_computed_=false;

    if(!gws_computed_)
      {
	ROS_DEBUG("Grasp Wrench Space not computed - cannot compute search zones");
	lock_.unlock();
	return;
      }

    //cannot compute ICR if the prototype grasp is not force closure
    if(!pt_grasp_->getGWS()->containsOrigin())
      {
	ROS_DEBUG("Trying to compute search zones, but the GWS is not force closure");
	lock_.unlock();
	return;
      }

    sz_.reset(new SearchZones(pt_grasp_));
    //A spherical taks wrench space with radius qs_*GWS_Insphere_radius is used here ...
    sz_->setTaskWrenchSpace(WrenchSpacePtr(new SphericalWrenchSpace(6,qs_*pt_grasp_->getGWS()->getOcInsphereRadius())));
    sz_->computeShiftedSearchZones();
    sz_computed_=true;
    lock_.unlock();
  }
  //-------------------------------------------------------------------------
  void IcrServer::computeIcr()
  {
    lock_.lock();
    icr_computed_=false;

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
    icr_->computeICR(Full);

    /////////////////////////////DEBUG ///////////////////////////////////////////////
    //Sometimes the icr computation glitches and sets all points on the objec as icr points - it
    //seems that creating a new prototype grasp and search zones here solves the problem, but it
    //shouldn't happen - maybe resetting the pt_grasp_ pointer in the GraspCallback can f**k things
    //up ...?  EDIT: now the pt_grasp_ is only re-initialized, not resetted - maybe this solves the
    //problem ...? EDIT: Nope!  EDIT: one issue is the multithreaded icr computation - the region_id
    //IndependentContactRegions::computeICRFull/BFS can get out of range - not the cause of the
    //problem though ... have to check the primitive search zone inclusion test in detail!
    //
    if(icr_->getNumICRPoints() > obj_->getNumCp())
      {
	icr_->getGrasp()->getParentObj()->writeToFile("/home/rkg/Desktop/debug/points.txt","/home/rkg/Desktop/debug/normals.txt","/home/rkg/Desktop/debug/neighbors.txt");
        icr_->getSearchZones()->writeToFile("/home/rkg/Desktop/debug/hyperplanes.txt");
        icr_->getGrasp()->getFinger(0)->getOWS()->writeToFile("/home/rkg/Desktop/debug/wrenches.txt"); //assuming that all fingers share the same OWS!!!

	std::cout<<"G: ";
	for (unsigned int i=0; i < icr_->getNumContactRegions(); i++)
	  std::cout<< pt_grasp_->getFinger(i)->getCenterPointPatch()->patch_ids_.front()<<" ";
	std::cout<<std::endl;

        ROS_WARN("Icr computation glitched - overall number of points: %d, Spherical TWS radius: %f",icr_->getNumICRPoints(),dynamic_cast<SphericalWrenchSpace*>(sz_->getTaskWrenchSpace().get())->getRadius());
        lock_.unlock();
	exit(0);
	return;
        
      }
    //////////////////////////////DEBUG END///////////////////////////////////////////////

    //update the icr msg
    icr_msg_->regions.clear();
    pcl::PointCloud<pcl::PointXYZRGBNormal> region_cloud;
    icr_msgs::ContactRegion region_msg;
    std::vector<unsigned int> point_ids;
    Eigen::MatrixXd dist;
    int mp_id;
    icr_msg_->header.stamp=ros::Time::now();
    icr_msg_->header.frame_id=computation_frame_id_;

    for (unsigned int i=0; i < icr_->getNumContactRegions(); i++)
      {
	if(!cloudFromContactRegion(i,region_cloud,point_ids))//convert region i to a point cloud
	  {
	    ROS_ERROR("Cannot convert region %d to a point cloud",i);
	    lock_.unlock();
	    return;
	  }
	//compute the middle point
	computeAdjacencyMatrix(point_ids,dist);
        floydWarshall(dist);      
        dist.colwise().sum().minCoeff(&mp_id);  
        region_msg.mp_index=point_ids[mp_id];
	std::cout<<"mp_id: "<<mp_id<<"point_ids[mp_id] "<<point_ids[mp_id]<<std::endl;
	icr_->getGrasp()->getParentObj()->getContactPoint(point_ids[mp_id]);

        region_msg.middle_point.x= (*(icr_->getGrasp()->getParentObj()->getContactPoint(point_ids[mp_id])->getVertex()))(0);
        region_msg.middle_point.y= (*(icr_->getGrasp()->getParentObj()->getContactPoint(point_ids[mp_id])->getVertex()))(1);
        region_msg.middle_point.z= (*(icr_->getGrasp()->getParentObj()->getContactPoint(point_ids[mp_id])->getVertex()))(2);
	region_msg.phalange=(std::string)phalange_config_[getPhalangeId(icr_->getGrasp()->getFinger(i)->getName())]["phl_name"];
	pcl::toROSMsg(region_cloud,region_msg.points); 
	region_msg.indices=point_ids;
	icr_msg_->regions.push_back(region_msg);
      } 
    icr_msg_->parent_obj=icr_->getGrasp()->getParentObj()->getName();
    tf::poseEigenToMsg(O_F_T_,icr_msg_->palm_pose);
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
    
    pcl::PointCloud<pcl::PointXYZRGBNormal> icr_cloud;  
    pcl::PointCloud<pcl::PointXYZRGBNormal> region_cloud;
    icr_cloud.header.frame_id=icr_msg_->header.frame_id;
    icr_cloud.header.stamp=icr_msg_->header.stamp;
    for (unsigned int i=0; i < icr_msg_->regions.size(); i++)
      {
	pcl::fromROSMsg(icr_msg_->regions[i].points,region_cloud);
        icr_cloud+=region_cloud;
      }

    icr_cloud_pub_.publish(icr_cloud);
    icr_pub_.publish(*icr_msg_);
    lock_.unlock();
  }
  //-------------------------------------------------------------------------
  bool IcrServer::cloudFromContactRegion(unsigned int region_id,pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, std::vector<unsigned int> & point_ids)
  {
    int phl_id = getPhalangeId(icr_->getGrasp()->getFinger(region_id)->getName());
    if(phl_id == -1)
      {
	ROS_ERROR("Cannot extract color associated with the phalange - cannot convert ContactRegion to pcl::PointCloud");
	return false;         
      }

    Eigen::Vector3d color;
    for (int32_t j = 0; j < 3 ;j++) 
      color(j)=(double)phalange_config_[phl_id]["display_color"][j];


    cloud.clear();
    point_ids.resize(icr_->getContactRegion(region_id)->size());
    for(uint j=0; j < icr_->getContactRegion(region_id)->size();j++) 
      {
	uint pt_id = icr_->getContactRegion(region_id)->at(j)->patch_ids_.front(); //get the patch centerpoint id
	point_ids[j]=pt_id;
	//need to check the vertices from the ICR grasp's parent obj whichs associated OWS list and
	//Patch list were used to compute the given ICR
	const Eigen::Vector3d* v = icr_->getGrasp()->getParentObj()->getContactPoint(pt_id)->getVertex();
        const Eigen::Vector3d* vn = icr_->getGrasp()->getParentObj()->getContactPoint(pt_id)->getVertexNormal();

	pcl::PointXYZRGBNormal pt;
	pt.x=v->x(); pt.y=v->y();pt.z=v->z();
	pt.r=color(0); pt.g=color(1); pt.b=color(2);
        pt.normal[0]=vn->x();pt.normal[0]=vn->y();pt.normal[0]=vn->z();
	cloud.points.push_back(pt);
      }
      
    cloud.header.frame_id = computation_frame_id_;
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
	ROS_ERROR("Phalange name %s is invalid. Cannot set phalange parameters - valid phalange specifications are:",req.name.c_str());
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

    //changing the active phalange parameters (its not actually checked whether the change was made on an active or inactive phalange) potentially requires changes in the associated OWS and Patch list. Thus, reinitialization is necessary
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
    obj_->computeCentroid();
    obj_set_=true;

    //Loading a new object requires recomputing the OWS list and the Patch list which is done by creating a new grasp an initializing it
    initPtGrasp();

    computation_frame_id_=obj_cloud.header.frame_id; 
    //reset the object transform - actual transformation is done in the GraspCallback
    O_F_T_.setIdentity();

    lock_.unlock();
    res.success=true;

    ROS_INFO("Object %s set in the icr_server ",req.object.name.c_str());
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
    //centerpoints will be computed in the getGrasp callback
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
    if(!(req.qs > 0.0) || !(req.qs < 1.0))
      {
	ROS_ERROR("Qs has to be larger than zero and smaller than one - cannot change Qs");
	return res.success;
      }

    lock_.lock();

    qs_=req.qs;
    icr_computed_=false;
    sz_computed_=false;
 
    lock_.unlock();
    res.success=true;
    return res.success;
  }
  //-------------------------------------------------------------------------
  void IcrServer::graspCallback(icr_msgs::Grasp const & grasp)
  {
    lock_.lock();
    gws_computed_=false;
    if(!obj_set_ || !pt_grasp_initialized_) //do nothing if no object is loaded or the grasp is not initialized
      {
	lock_.unlock();
	return;
      }

    pcl::PointCloud<pcl::PointXYZRGB> seed_points; 
    pcl::PointXYZRGB pt;  pt.r=255.0; pt.g=0.0; pt.b=0.0;
    bool all_phl_touching=true; //not used right now
    bool phl_touching;
    Eigen::Vector3d contact_position;
    VectorXui centerpoint_ids(active_phalanges_.size());
    for (int i=0; i<centerpoint_ids.size();i++)
      {
	if(!cpFromGraspMsg(grasp,active_phalanges_[i],contact_position,phl_touching))
	  {
	    ROS_ERROR("Invalid Grasp message - cannot compute the Grasp Wrench Space");
	    lock_.unlock();
	    return;
	  }
	//	std::cout<<active_phalanges_[i]<<": "<<phl_touching<<" ";
	if(!phl_touching)
	  all_phl_touching=false;   
   
	if(compute_in_palm_frame_)
	  contact_position=O_F_T_.inverse()*contact_position;//transform the  contact position into the previous palm frame F before trying to find the closest points

	centerpoint_ids(i)=findObjectPointId(&contact_position);
	  
	pt.x=(*obj_->getContactPoint(centerpoint_ids(i))->getVertex())(0); 
	pt.y=(*obj_->getContactPoint(centerpoint_ids(i))->getVertex())(1); 
	pt.z=(*obj_->getContactPoint(centerpoint_ids(i))->getVertex())(2); 
	seed_points.points.push_back(pt);

	// visualize the target contact locations on the fingers also ...
	// pt.x=contact_position(0);
	// pt.y=contact_position(1);
	// pt.z=contact_position(2);
	// seed_points.points.push_back(pt);
      }
 
    Eigen::Affine3d O_C_T; //get the current pose of the palm frame expressed in the object frame
    tf::poseMsgToEigen(grasp.palm_pose,O_C_T);
    // pcl::PointCloud<pcl::PointXYZ> dummy_obj;
    // pcl::PointXYZ dummy_pt;
    //Compute the grasp wrench space either in the object frame or transform the object into the palm frame and re-initialize the grasp (computing new OWS & GWS)
    if(compute_in_palm_frame_)
      {
	Eigen::Affine3d C_F_T = O_C_T.inverse()*O_F_T_; //compute the pose of the former palm frame expressed in the current palm frame;
        obj_->transform(C_F_T); //Express object vertices/vertex normals (which are currently expressed in F_O_T) in the current palm frame 
	// for(uint n=0;n<obj_->getNumCp();n++)
	//   {
	//     dummy_pt.x=(*obj_->getContactPoint(n)->getVertex())(0);
	//     dummy_pt.y=(*obj_->getContactPoint(n)->getVertex())(1);
	//     dummy_pt.z=(*obj_->getContactPoint(n)->getVertex())(2);
	// dummy_obj.points.push_back(dummy_pt);
        //   }

        FParamList phl_param;
        getActivePhalangeParameters(phl_param);
	pt_grasp_.reset(new Grasp); // maybe this messes things up
        pt_grasp_->init(phl_param,obj_,centerpoint_ids);
	pt_grasp_initialized_=true;	

	computation_frame_id_=grasp.palm_frame_id;
      }
    else
      pt_grasp_->setCenterPointIds(centerpoint_ids); //this only re-computes the Grasp Wrench Space, not the OWS 

    // dummy_obj.header.frame_id=computation_frame_id_;
    // dummy_obj.header.stamp=ros::Time::now();
    // dummy_pub_.publish(dummy_obj);


    //memorize the current grasp pose for the next iteration
    O_F_T_=O_C_T; 
    
    seed_points.header.frame_id=computation_frame_id_;
    seed_points.header.stamp=grasp.header.stamp;
    sp_pub_.publish(seed_points);

    gws_computed_=true;

    //EXPERIMENTAL!!!!!!!!!!!!!!!!!!!! just check if its feasible to stop computatin once all fingers are in touch (e.g. for saving icr)
    // if(all_phl_touching && (computation_mode_ == MODE_CONTINUOUS))
    //   {
    // 	computation_mode_=MODE_STEP_WISE;   
    // 	ROS_INFO("All active phalanges are touching - leaving continuous mode and entering step wise mode...");
    //   }

    lock_.unlock();
  }
  //-------------------------------------------------------------------------
  bool IcrServer::cpFromGraspMsg(icr_msgs::Grasp const & grasp, const std::string & name,Eigen::Vector3d & contact_position,bool & touching)const
  {

    for(unsigned int i=0; i<grasp.points.size();i++)
      if(!strcmp(name.c_str(),grasp.points[i].phalange.c_str()))
	{
          contact_position(0)=grasp.points[i].position.x;
          contact_position(1)=grasp.points[i].position.y;
          contact_position(2)=grasp.points[i].position.z;
          touching=grasp.points[i].touching;
          return true;
        }
    
    ROS_ERROR("Could not find contact point corresponding to phalange %s in Grasp message",name.c_str());
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
      
    f_param.setForce((double)phalange_config_[phl_id]["force_magnitude"]);
    f_param.setFriction((double)phalange_config_[phl_id]["mu_0"]);
    f_param.setFrictionTorsional((double)phalange_config_[phl_id]["mu_T"]);
    f_param.setDiscretization((int)phalange_config_[phl_id]["fc_disc"]);
    f_param.setInclusionRuleParameter((double)phalange_config_[phl_id]["rule_parameter"]);
    f_param.setInclusionRuleFilterPatch((bool)phalange_config_[phl_id]["filter_patch"]);

    std::string string_param;
    string_param=(std::string)phalange_config_[phl_id]["phl_name"];
    f_param.setName(string_param);
    string_param=(std::string)phalange_config_[phl_id]["contact_type"];
    f_param.setContactType(string_param);
    string_param=(std::string)phalange_config_[phl_id]["model_type"];
    f_param.setContactModelType(string_param);
    string_param=(std::string)phalange_config_[phl_id]["rule_type"];
    f_param.setInclusionRuleType(string_param);
  }
  //---------------------------------------------------------------------------
  uint IcrServer::findObjectPointId(Eigen::Vector3d* point_in) const 
  {
    double min_dist = 1000000;
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
  int IcrServer::getComputationMode()
  {
    int computation_mode;

    lock_.lock();
    computation_mode=computation_mode_;
    lock_.unlock();

    return computation_mode;
  }
  //--------------------------------------------------------------------------------------------
  bool IcrServer::saveIcr(icr_msgs::SaveIcr::Request &req, icr_msgs::SaveIcr::Response &res)
  {
    res.success=false;

    if(req.file.empty())
      {
    	ROS_ERROR("Given file name is invalid - cannot save ICR");
    	return res.success;
      }

    lock_.lock();
    if(!icr_computed_)
      {
    	ROS_ERROR("No ICR computed - cannot save");
    	lock_.unlock();
    	return res.success;
      }

    rosbag::Bag bag(icr_database_dir_+ req.file + ".bag", rosbag::bagmode::Write);
    bag.write("contact_regions", icr_msg_->header.stamp, *icr_msg_);

    lock_.unlock();
    bag.close();
    res.success=true;

    ROS_INFO("saved ICR to: %s",(icr_database_dir_+ req.file + ".bag").c_str());
    return res.success;
  }
  //--------------------------------------------------------------------------------------------
bool IcrServer::setComputationMode(icr_msgs::SetComputationMode::Request &req, icr_msgs::SetComputationMode::Response &res)
  {
    res.success=false;
    lock_.lock();
  
    switch (req.mode) 
      {
      case MODE_CONTINUOUS : 
	ROS_INFO("Entering continous mode...");
	computation_mode_=MODE_CONTINUOUS;
	break;

      case MODE_STEP_WISE : 
	ROS_INFO("Entering step wise mode...");
	computation_mode_=MODE_STEP_WISE;
	break;

     case MODE_TRANSFER : 
	ROS_INFO("Entering transfer mode...");
	computation_mode_=MODE_TRANSFER;
	break;

      default : 
	ROS_WARN("%d is an invalid computation mode - cannot switch mode",req.mode);
        lock_.unlock();
        return res.success;
      }

    lock_.unlock();

    res.success=true;
    return res.success;
  }
  //--------------------------------------------------------------------------------------------
  bool IcrServer::triggerIcrCmp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    lock_.lock();

    if(computation_mode_==MODE_CONTINUOUS)
      {
	ROS_ERROR("The ICR server runs in continuous mode and must be switched to step wise mode to trigger manual Icr computation");
	lock_.unlock();
	return false;
      }

    lock_.unlock();

    computeIcr();
  
    ROS_INFO("Computed contact regions");
    return true;
  }
  //--------------------------------------------------------------------------------------------
  bool IcrServer::triggerSearchZonesCmp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    lock_.lock();

    if(computation_mode_==MODE_CONTINUOUS)
      {
	ROS_ERROR("The ICR server runs in continuous mode and must be switched to step wise mode to trigger manual search zone computation");
	lock_.unlock();
	return false;
      }

    sz_computed_=false;

    if(!gws_computed_)
      {
	ROS_ERROR("Grasp Wrench Space not computed - cannot compute search zones");
	lock_.unlock();
	return false;
      }

    //cannot compute ICR if the prototype grasp is not force closure
    if(!pt_grasp_->getGWS()->containsOrigin())
      {
	ROS_ERROR("Trying to compute search zones, but the GWS is not force closure");
	lock_.unlock();
	return false;
      }
    sz_.reset(new SearchZones(pt_grasp_));
    //A spherical task wrench space is used here ...
    sz_->setTaskWrenchSpace(WrenchSpacePtr(new SphericalWrenchSpace(6,qs_*pt_grasp_->getGWS()->getOcInsphereRadius())));
    sz_->computeShiftedSearchZones();
    sz_computed_=true;
    lock_.unlock();
    ROS_INFO("Computed search zones");
    return true;
  }
  //--------------------------------------------------------------------------------------------
  bool IcrServer::getIcr(icr_msgs::GetContactRegions::Request  &req, icr_msgs::GetContactRegions::Response &res)
  {
    res.success=false;
    lock_.lock();
    if(!icr_computed_)
      {
	ROS_ERROR("ICR are not computed - cannot get ICR");
	lock_.unlock();
	return res.success;
      }
    res.contact_regions=(*icr_msg_);
    lock_.unlock();

    res.success=true;
    return res.success;
  }
  //--------------------------------------------------------------------------------------------
  void IcrServer::computeAdjacencyMatrix(std::vector<unsigned int> const & point_ids, Eigen::MatrixXd & adj_mtrx)const
  {
    unsigned int N=point_ids.size();
    adj_mtrx.resize(N,N);
    adj_mtrx.setZero(N,N);

    ICR::TargetObjectPtr const obj_=icr_->getGrasp()->getParentObj();
    
    for (unsigned int i = 0; i < N; i++)
      for (unsigned int j = 0; j < N; j++)
	{
	  if(obj_->getContactPoint(point_ids[i])->isNeighbor(point_ids[j]))
	    adj_mtrx(i,j)=((*(obj_->getContactPoint(point_ids[i])->getVertex()))-(*(obj_->getContactPoint(point_ids[j])->getVertex()))).norm();
        }
  }
  //--------------------------------------------------------------------------------------------
  void IcrServer::floydWarshall(Eigen::MatrixXd & dist)const
  {
    unsigned int N=dist.cols();
    for (unsigned int k = 0; k < N; ++k) 
      {
	for (unsigned int i = 0; i < N; ++i)
	  for (unsigned int j = 0; j < N; ++j)
	    /* If i and j are different nodes and if
	       the paths between i and k and between
	       k and j exist, do */
	    if ((dist.coeff(i,k) * dist.coeff(k,j) != 0) && (i != j))
	      /* See if you can't get a shorter path
		 between i and j by interspacing
		 k somewhere along the current
		 path */
	      if ((dist.coeff(i,k) + dist.coeff(k,j) < dist.coeff(i,j)) ||
		  (dist.coeff(i,j) == 0))
		dist(i,j) = dist.coeff(i,k) + dist.coeff(k,j);
      }
  }
  //--------------------------------------------------------------------------------------------
}//end namespace
