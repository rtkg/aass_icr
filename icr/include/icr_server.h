#ifndef icr_server_h___
#define icr_server_h___

#include "ros/ros.h"
#include "icr.h"
#include "../srv_gen/cpp/include/icr/compute_icr.h"
#include "../srv_gen/cpp/include/icr/compute_icr5.h"
#include "../srv_gen/cpp/include/icr/load_object.h"
#include "../srv_gen/cpp/include/icr/set_finger_number.h"
#include "../srv_gen/cpp/include/icr/set_finger_parameters.h"
#include <boost/thread/mutex.hpp>

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

  ros::NodeHandle nh_,nh_private_;
  ros::ServiceServer compute_icr_service_;
  ros::ServiceServer load_wfront_obj_service_;
  ros::ServiceServer set_finger_number_service_;
  ros::ServiceServer set_finger_parameters_service_;
  boost::mutex data_mutex_;

  bool computeIcrCore(ICR::VectorXui &centerpoint_ids);

  ICR::ObjectLoader* obj_loader_;
  ICR::FParamList* finger_parameters_;
  ICR::IndependentContactRegions* icr_;
  /** \brief If true, the \ref ICR::IndependentContactRegions::grasp_
   * "grasp" needs to be updated prior to icr computation because the
   * \ref finger_parameters_ or the OWS has changed. Not fully
   * implemented. Right now grasp is recalculated every time.
   */
   bool update_fingers_;

 public:
 
  IcrServer();
  ~IcrServer();

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

  /** \brief Ros service. Computes ICRs in the same way as the \ref
   *  computeIcr but the indices are returned in vectors -- one for
   *  each fingers. This service works for hands with up to 5 fingers
   *  maximum. If the hand has more fingers their respective ICRs are
   *  not returned. 
   * 
   * \return Ros srv icr/compute_icr5 structure containing 5 vectors
   * with point indices and bool set to true if fun executed correctly.
   */
  bool computeIcr5fingers(icr::compute_icr5::Request  &req, 
			  icr::compute_icr5::Response &res);

  /** \brief ROS service. Loads an *.obj file into IcrServer. return
   * true if succeded.
   * 
   * From command line: 
   * \verbatime $ rosservice /icr_server/load_wfront_obj call <path_to_a_file> <name> \endverbatime
   */
  bool loadWfrontObj(icr::load_object::Request  &req, 
		     icr::load_object::Response &res);

  /** \brief ros service. Sets the number of fingers in
   * IcrServer. Deletes previous fingers, thus new parametrization is needed.
   * 
   * From command line: 
   * \verbatime $ rosservice /icr_server/set_finger_number call <number> \endverbatime
   */
  bool setFingerNumber(icr::set_finger_number::Request &req, 
		       icr::set_finger_number::Response &res); 


  /** \brief ros service. Sets parameters of added fingers.
   * 
   */
  bool setFingerParameters(icr::set_finger_parameters::Request &req, 
			   icr::set_finger_parameters::Response &res);

};




#endif
