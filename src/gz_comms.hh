#ifndef GZ_COMMS_H
#define GZ_COMMS_H

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include "common_cntrl.hh"

extern gazebo::msgs::IMU IMU_data;
extern gazebo::msgs::PosesStamped pose_data;
extern gazebo::msgs::Diagnostics diagnostics_data;
extern gazebo::msgs::LinkData body_link_data;
extern gazebo::math::Quaternion body_orientation;
extern gazebo::math::Vector3 body_location;

extern  gazebo::transport::NodePtr node;
extern  gazebo::transport::SubscriberPtr sub_IMU;
extern  gazebo::transport::SubscriberPtr sub_Poses;
extern  gazebo::transport::SubscriberPtr sub_Diagnostics;
extern  gazebo::transport::SubscriberPtr sub_Body_Link;

int gz_comms_init(std::string mav_name_in);
int gz_comms_end(void);
int gz_comms_send_update(gazebo::math::Vector3 & desLoc, desCntrl_T & desCntrl,motInfo & mot);
int gz_comms_lock_mutex(void);
int gz_comms_unlock_mutex(void);

void update_IMU_data(ConstIMUPtr &_msg);
void poses_callback(ConstPosesStampedPtr &_msg);
void diagnostics_callback(ConstDiagnosticsPtr &_msg);
void body_link_callback(ConstLinkDataPtr &_msg);
#endif // GZ_COMMS_H