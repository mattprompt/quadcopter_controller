#ifndef CONTROL_NAVIGATION_HH

#define CONTROL_NAVIGATION_HH
//#include <gazebo/gazebo.hh>
//#include <gazebo/transport/transport.hh>
//#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include "common_cntrl.hh"

extern gazebo::msgs::PosesStamped pose_data;
extern gazebo::msgs::LinkData body_link_data;
extern gazebo::math::Quaternion body_orientation;
extern gazebo::math::Vector3 body_location;
int control_navigation_init(void);
int control_navigation_end(void);
int control_navigation_update(gazebo::math::Vector3 & desLoc, desCntrl_T & desCntrl, double curTime);

#endif