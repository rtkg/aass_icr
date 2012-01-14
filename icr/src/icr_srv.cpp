#include "ros/ros.h"
#include "std_msgs/String.h"
#include "icr.h"
#include <sstream>
#include <sys/time.h>
#include <time.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "icr_srv");
  ros::NodeHandle n;

  //Load a new target object 
  ICR::ObjectLoader obj_loader;
  obj_loader.loadObject("/home/rkg/ros/aass_icr/libicr/icrcpp/models/beer_can.obj","beer_can");
   
  //Create a list of 5 default finger parameters (default parameters defined in config.h) and a 
  //vector of centerpoint contact id's for the 5-fingered prototype grasp
  ICR::FParamList f_parameters;
  ICR::FingerParameters parameters;
   for (int i=0;i<5;i++)
     f_parameters.push_back(parameters);

   ICR::VectorXui centerpoint_ids(5);
   centerpoint_ids << 1838, 4526, 4362, 1083, 793;

   //Modify some of the finger parameters of the prototype-grasp.
   //The third finger utilizies the Multi-Point contact model, patches only contain the center-point
   //and border points
   f_parameters[0].setSoftFingerContact(1,6,0.5,0.5);
   f_parameters[2].setContactModelType(ICR::Multi_Point);
   f_parameters[2].setInclusionRuleFilterPatch(true);

   //Create a prototype grasp and search zones, the parameter alpha is the scale of the largest
   //origin-centered ball contained by the Grasp wrench space of the prototype grasp
   ICR::GraspPtr prototype_grasp(new ICR::Grasp());
   prototype_grasp->init(f_parameters,obj_loader.getObject(),centerpoint_ids);
   ICR::SearchZonesPtr search_zones(new ICR::SearchZones(prototype_grasp));
   double alpha=0.5;
   search_zones->computeShiftedSearchZones(alpha);

   //Create and plot the Independent Contact Regions
   ICR::IndependentContactRegions icr(search_zones,prototype_grasp);
   icr.computeICR();
   std::cout<<icr;


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  
  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // ros::Rate loop_rate(10);

  // /**
  //  * A count of how many messages we have sent. This is used to create
  //  * a unique string for each message.
  //  */
  // int count = 0;
  // while (ros::ok())
  // {
  //   /**
  //    * This is a message object. You stuff it with data, and then publish it.
  //    */
  //   std_msgs::String msg;

  //   std::stringstream ss;
  //   ss << "hello " << count;
  //   msg.data = ss.str();

  //   ROS_INFO("%s", msg.data.c_str());

  //   /**
  //    * The publish() function is how you send messages. The parameter
  //    * is the message object. The type of this object must agree with the type
  //    * given as a template parameter to the advertise<>() call, as was done
  //    * in the constructor above.
  //    */
  //   chatter_pub.publish(msg);

  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   ++count;
  // }


  // Utility for timing selected parts of the code - uncomment below and put the code to be timed at the marked location
  // struct timeval start, end;
  // double c_time;
  // gettimeofday(&start,0);
  //
  //  Put code to be timed here...
  //  
  // gettimeofday(&end,0);
  // c_time = end.tv_sec - start.tv_sec + 0.000001 * (end.tv_usec - start.tv_usec);
  // std::cout<<"Computation time: "<<c_time<<" s"<<std::endl;

  return 0;
}
