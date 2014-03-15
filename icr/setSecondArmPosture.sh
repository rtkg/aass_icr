#!/bin/bash

rostopic pub /sr_arm/sendupdate sr_robot_msgs/sendupdate "{sendupdate_length: 4 ,sendupdate_list: [{joint_name: ElbowJRotate,joint_target: 42},{joint_name: ElbowJSwing,joint_target: 32},{joint_name: ShoulderJRotate,joint_target: 3},{joint_name: ShoulderJSwing,joint_target: 63}]}"


  



 
 


 



 
 









