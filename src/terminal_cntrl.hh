#ifndef TERMINAL_CNTRL_H

#define TERMINAL_CNTRL_H
#include <boost/concept_check.hpp>
//#include <gazebo/gazebo.hh>
//#include <gazebo/transport/transport.hh>
//#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>
#include "common_cntrl.hh"



int term_init(void);
int term_end(void);
int term_handle_input(desCntrl_T & desCntrl, gazebo::math::Vector3 & desLoc);

#endif