#ifndef CONTROL_ORIENTATION_HH

#define CONTROL_ORIENTATION_HH
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include "common_cntrl.hh"

int control_orientation_init(void);
int control_orientation_end(void);
int control_orientation_update(desCntrl_T & desCntrl, motInfo & mot, const double & curTime);
extern gazebo::msgs::PosesStamped pose_data;
extern gazebo::math::Quaternion body_orientation;

#endif