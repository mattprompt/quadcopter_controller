#ifndef COMMON_CNTRL_H

#define COMMON_CNTRL_H

#include <boost/concept_check.hpp>
typedef struct 
{
  double ki = 1;
  double kp = 1;
  double kd = 1;
  double preVal = 0;
  double curVal = 0;
  double intSum = 0;
  double derDiff = 0;
} pidInfo;

typedef struct 
{
  double mot1;
  double mot2;
  double mot3;
  double mot4;
} motInfo;

typedef struct
{
  double thrust;
  double thrust_hover;
  double roll;
  double pitch;
  double yaw;
  double errThrust;
  double errRoll;
  double errPitch;
  double errYaw;
} desCntrl_T;

#endif