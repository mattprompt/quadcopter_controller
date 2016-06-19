#ifndef RC_CONTROLLER_COMMS_HH

#define RC_CONTROLLER_COMMS_HH
extern "C"
{
  #include "BuddyBoxThread.h"
}

extern PASBuddyBox pasBB;
int rc_controller_comms_init(void);
int rc_controller_comms_end(void);
int rc_controller_comms_update(PASBuddyBox & pasBB_out);

#endif