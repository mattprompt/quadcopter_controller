#include "rc_controller_comms.hh"
extern "C"
{
  #include "BuddyBoxThread.h"
}

#define DEFAULT_SAMPLE_RATE 124000

PASBuddyBox pasBB;

int rc_controller_comms_init(void)
{
  pasBB.sampleRate =  DEFAULT_SAMPLE_RATE;
  initializeBuddyBoxThread(&pasBB);
  startBuddyBoxThread(&pasBB);
  return 0;
}

int rc_controller_comms_end(void)
{
  stopBuddyBoxThread(&pasBB);
  joinBuddyBoxThread(&pasBB);
  cleanupBuddyBoxThread(&pasBB);
  return 0;
}

int rc_controller_comms_update(PASBuddyBox & pasBB_out)
{
  if(isBuddyBoxThreadRunning(&pasBB) &&
     isBuddyBoxThreadCalibrated(&pasBB))
  {
    pasBB_out = pasBB;
  }
  return 0;
}





