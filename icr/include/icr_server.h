#ifndef icr_server_h___
#define icr_server_h___

#include "ros/ros.h"
#include "icr.h"
#include "../srv_gen/cpp/include/icr/compute_icr.h"
#include "../srv_gen/cpp/include/icr/load_object.h"
#include "../srv_gen/cpp/include/icr/set_finger_number.h"
#include "../srv_gen/cpp/include/icr/set_finger_parameters.h"
#include "icr/ContactPoints.h"

#include <boost/thread/mutex.hpp>
#include <sensor_msgs/PointCloud2.h>

//#include <sensor_msgs/PointCloud.h>
// PCL specific includes
//#include <pcl/ros/conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

/** \class IcrServer icr_server.h 
 * \brief Server that computes Independent Contact Regions
 *
 * The icrServer can be parametrized in several ways:
 * \li A model of an object can be loaded \ref loadWfrontObj.  
 * \li The number of fingers can be specified \ref addFingers.
 * \li Each finger can be parametrized \ref setFingerParameters.
 * Finally, the icr can be computed using \ref computeIcr. 
 *
 * Assumption is that IcrServer, when properly parametrized, returns
 * indices of vertices that consist ICRs. Communication with IcrServer
 * is done through services.
 *
 * Example sequence of calling of services that leads to icr computation:
 * ....
 * 
 */
class IcrServer
{
 private:
  //
  ros::NodeHandle nh_,nh_private_;
  //
  ros::ServiceServer compute_icr_service_;
  ros::ServiceServer compute_icr_file_server_;
  ros::ServiceServer load_wfront_obj_service_;
  ros::ServiceServer set_finger_number_service_;
  ros::ServiceServer set_finger_parameters_service_;
  /** \brief Discard previous configuration of fingers. Configures the
   *   fingers according to input file.
   */
  ros::ServiceServer set_fingers_service_;
  //
  ros::Publisher pub_icr_cloud_; 
  //
  ros::Subscriber sub_finger_tips_;
  boost::mutex data_mutex_;

  ICR::ObjectLoader* obj_loader_;
  ICR::FParamList* finger_parameters_;
  ICR::FingerParameters default_finger_params_; 
  ICR::IndependentContactRegions* icr_;
  sensor_msgs::PointCloud2 output_cloud_;

  /** \brief Should be set <true> whenever, the \ref
   * ICR::IndependentContactRegions::grasp_ "grasp" needs to be
   * updated prior to icr computation because the \ref
   * finger_parameters_ or the OWS has changed. The update is done in
   * \ref computeIcrCore and the variable is set to <false>.
   */
  bool grasp_update_needed_;
  double alpha_shift_; ///< fraction of the TWS sphere that determines the search zones
  
  bool loadFingerParameters();
  bool updateOneFinger(XmlRpc::XmlRpcValue &finger_config,
		       ICR::FingerParameters *finger_out);
  
  bool computeIcrCore(ICR::VectorXui &centerpoint_ids);
  
  void publishICRpc();
  
  void fingerTipCallback(const icr::ContactPoints& msg);

  /**\brief Returns the index of a vertex in the mesh (of an object
     stored in \ref obj_loader_ ) that is closest to point_in given as
     an argument. */
  uint findClosestStupid(Eigen::Vector3d* point_in) const;

 public:
  IcrServer();
  ~IcrServer();
  void publishCloud();
  
  /** \brief ROS service. Computes ICRs. 
   *
   * Given that all prerequisites are done (e.g. file loaded, fingers
   * defined, etc.) This service, given requested indices of the
   * center-points of a grasp, calculated ICRs and responds with
   * indices of them.  
   *
   * Called from terminal (assuming that the grasp and search zones
   * are precomputed) \verbatime $ rosservice call
   * /icr_server/compute_icr [1,2,3,4,5]
   *
   * \param req Part of ros srv uint16_t[] containing indices of the
   * center-points of a grasp 
   * \return res Structure with bool value indicating the successa and
   * indices of icr built
   */
  bool computeIcr(icr::compute_icr::Request  &req, 
		  icr::compute_icr::Response &res);

  /** \brief ROS service. Loads an *.obj file into IcrServer. return
   * true if succeded.
   * 
   * From command line: 
   * \verbatime $ rosservice call /icr_server/load_wfront_obj <path_to_a_file> <name> \endverbatime
   */
  bool loadWfrontObj(icr::load_object::Request  &req, 
		     icr::load_object::Response &res);

  /** \brief ros service. Sets the number of fingers in
   * IcrServer. Deletes previous fingers, thus new parametrization is
   * needed.
   * 
   * From command line: 
   * \verbatime $ rosservice /icr_server/set_finger_number call <number> \endverbatime
   */
  bool setFingerNumber(icr::set_finger_number::Request &req, 
		       icr::set_finger_number::Response &res); 


  /** \brief ros service. Sets parameters of added fingers.  */
  bool setFingerParameters(icr::set_finger_parameters::Request &req, 
			   icr::set_finger_parameters::Response &res);

};




#endif
