#ifndef TESTCNTRL_HH

#define TESTCNTRL_HH
//#include <gazebo/gazebo.hh>
//#include <gazebo/transport/transport.hh>
//#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include "common_cntrl.hh"

extern "C" 
{
  #include "BuddyBoxThread.h"
  #include "BuddyBox.h"
  #include "portaudio.h"
}

extern PASBuddyBox pasBB;
int test_control_update(desCntrl_T & desCntrl,gazebo::math::Vector3 & desLoc, const double & curSimTime, const int mav_number);
int test_control_init(void);
int test_control_end(void);

#endif