#include <gazebo/msgs/msgs.hh>
#include "control_navigation.hh"
#include <ncurses.h>

int control_navigation_init(void)
{
  return 0;
}

int control_navigation_end(void)
{
  return 0;
}

int control_navigation_update(gazebo::math::Vector3 & desLoc, desCntrl_T & desCntrl, double curTime) //std::ofstream & vecOutFile
{
  
  double angleLimit = 0.075; // limit in radians. 45 degrees = pi/4 ~= 0.75
  const double errVelLim = 5; // 1 m/s = 2.2 mph
  const double errLocLim = 5;
  const double errVelIntLim = 1; // hover thrust ~ 26, so some fraction of that is fine
  const double errLocIntLim = 1;
  //double desYaw = 0;
 
  static pidInfo navThrust; 	// Z
  static pidInfo navRoll; 	// Y
  static pidInfo navPitch; 	// X
  //static pidInfo navYaw; 	// ?
  static pidInfo navX_vel;
  static pidInfo navY_vel;
  static pidInfo navZ_vel;
  
  gazebo::math::Vector3 errVec;
  gazebo::math::Vector3 errVecLoc;
  gazebo::math::Vector3 tmpVec;
  //gazebo::msgs::Vector3d tmpMsgVec;
  gazebo::math::Vector3 curVel;
  gazebo::math::Vector3 errVel;
  gazebo::math::Vector3 errVelLoc;
  static gazebo::math::Vector3 desVel;
  static gazebo::math::Vector3 prevLoc;
  static double prevTime = 0;
  double dt;
  
  dt = curTime - prevTime;
  prevTime = curTime;
  
  if(pose_data.IsInitialized() == false) // if we don't have any position data, just exit
  {
    return 0;
  }
  
  // calculate position error
  // We're cheeting and using actual position not simulated sensor data with error
//  tmpVec = gazebo::msgs::Convert(pose_data.pose(0).position()); // poistion information as gazebo::math::Vector3
  errVec = desLoc - body_location; // get vector from current location to desired location
  //errVecLoc = errVec;
  //errVecLoc = body_orientation.RotateVector(errVec); // rotate error vector into current body frame so we can control from it
  errVecLoc = body_orientation.RotateVectorReverse(errVec);
  
  if (errVecLoc.x > errLocLim)
  {
    errVecLoc.x = errLocLim;
  }
  else if (errVecLoc.x < -1 * errLocLim)
  {
    errVecLoc.x = -1 * errLocLim;
  }

  if (errVecLoc.y > errLocLim)
  {
    errVecLoc.y = errLocLim;
  }
  else if (errVecLoc.y < -1 * errLocLim)
  {
    errVecLoc.y = -1 * errLocLim;
  }

//  if (errVecLoc.z > errLocLim)
//  {
//    errVecLoc.z = errLocLim;
//  }
//  else if (errVecLoc.z < -1 * errLocLim)
//  {
//    errVecLoc.z = -1 * errLocLim;
//  }
  
  mvprintw(10,30,"body x: %f",body_orientation.x);
  mvprintw(11,30,"body y: %f",body_orientation.y);
  mvprintw(12,30,"body z: %f",body_orientation.z);
  mvprintw(13,30,"body w: %f",body_orientation.w);

  // calculate velocity error 
//  curVel = (tmpVec - prevLoc)/dt;
  curVel = gazebo::msgs::Convert(body_link_data.linear_velocity());
  errVel = desVel - curVel;
  //errVelLoc = errVel;
  //errVelLoc = body_orientation.RotateVector(errVel);
  errVelLoc = body_orientation.RotateVectorReverse(errVel);
  //errVelLoc = desVel;
  //errVelLoc.Correct();
  
  if (errVelLoc.x > errVelLim)
  {
    errVelLoc.x = errVelLim;
  }
  else if (errVelLoc.x < -1 * errVelLim)
  {
    errVelLoc.x = -1 * errVelLim;
  }
  
  if (errVelLoc.y > errVelLim)
  {
    errVelLoc.y = errVelLim;
  }
  else if (errVelLoc.y < -1 * errVelLim)
  {
    errVelLoc.y = -1 * errVelLim;
  }  
  
  if (errVelLoc.z > errVelLim)
  {
    errVelLoc.z = errVelLim;
  }
  else if (errVelLoc.z < -1 * errVelLim)
  {
    errVelLoc.z = -1 * errVelLim;
  }
  
  mvprintw(4,0,"Vx: %f\nVy: %f\n,Vz: %f\n",curVel.x,curVel.y,curVel.z);
  printw("errVelLoc x: %f\ny: %f\nz: %f",errVelLoc.x,errVelLoc.y,errVelLoc.z);
  refresh();
  

  // calculate integral
  navThrust.intSum = navThrust.intSum + errVecLoc.z * dt;
  navRoll.intSum   = navRoll.intSum   + errVecLoc.y * dt; 
  navPitch.intSum  = navPitch.intSum  + errVecLoc.x * dt;
  //navYaw.intSum    = navYaw.intSum    + ??? * dt; // for now leaf out yaw. Don't care which direction it's pointing

  if (navThrust.intSum > errLocIntLim)
  {
    navThrust.intSum = errLocIntLim;
  }
  else if (navThrust.intSum < -1* errLocIntLim)
  {
    navThrust.intSum = -1 * errLocIntLim;
  }

  if (navRoll.intSum > errLocIntLim)
  {
    navRoll.intSum = errLocIntLim;
  }
  else if (navRoll.intSum < -1* errLocIntLim)
  {
    navRoll.intSum = -1 * errLocIntLim;
  }

  if (navPitch.intSum > errLocIntLim)
  {
    navPitch.intSum = errLocIntLim;
  }
  else if (navPitch.intSum < -1* errLocIntLim)
  {
    navPitch.intSum = -1 * errLocIntLim;
  }

  navX_vel.intSum = navX_vel.intSum + errVelLoc.x * dt;
  navY_vel.intSum = navY_vel.intSum + errVelLoc.y * dt;
  navZ_vel.intSum = navZ_vel.intSum + errVelLoc.z * dt;
  
  if (navX_vel.intSum > errVelIntLim)
  {
    navX_vel.intSum = errVelIntLim;
  }
  else if (navX_vel.intSum < -1* errVelIntLim)
  {
    navX_vel.intSum = -1 * errVelIntLim;
  } 
  
  if (navY_vel.intSum > errVelIntLim)
  {
    navY_vel.intSum = errVelIntLim;
  }
  else if (navY_vel.intSum < -1* errVelIntLim)
  {
    navY_vel.intSum = -1 * errVelIntLim;
  }
  
  if (navZ_vel.intSum > errVelIntLim)
  {
    navZ_vel.intSum = errVelIntLim;
  }
  else if (navZ_vel.intSum < -1* errVelIntLim)
  {
    navZ_vel.intSum = -1 * errVelIntLim;
  }
  
  // calculate derivative
  navThrust.derDiff = (errVecLoc.z - navThrust.preVal) / dt;
  navRoll.derDiff   = (errVecLoc.y - navRoll.preVal) / dt;
  navPitch.derDiff  = (errVecLoc.x - navPitch.preVal) / dt;
  //navYaw.derDiff    = (??? - navYaw.preVal) / dt;
  
  navX_vel.derDiff = (errVelLoc.x - navX_vel.preVal) / dt;
  navY_vel.derDiff = (errVelLoc.y - navY_vel.preVal) / dt;
  navZ_vel.derDiff = (errVelLoc.z - navZ_vel.preVal) / dt;
  
  
  // update previous values
  navThrust.preVal = errVecLoc.z;
  navRoll.preVal   = errVecLoc.y;
  navPitch.preVal  = errVecLoc.x;
  //navYaw.preVal    = ????;
  
  navX_vel.preVal = errVelLoc.x;
  navY_vel.preVal = errVelLoc.y;
  navZ_vel.preVal = errVelLoc.z;
  
  prevLoc = body_location;
  
  desCntrl.thrust =  0.05*(20*navThrust.kp*errVecLoc.z   + 0*1*navThrust.ki*navThrust.intSum    + 0*1*navThrust.kd*navThrust.derDiff) + 
			1*(0*navZ_vel.kp*errVelLoc.z 	 + 0*navZ_vel.ki*navZ_vel.intSum 	+ 0*navZ_vel.kd*navZ_vel.derDiff);//+ desCntrl.thrust_hover);
  
  // Notes:
  // roll/pitch Prop Gain 0.1 too high
  // vel prop gain 0.1 too high
  
  // for positive Y we need negative roll. See coordinate system.

  desCntrl.roll   = -1*0.5*(0.01*navRoll.kp*errVecLoc.y  + 0*0.002*navRoll.ki*navRoll.intSum     + 0*0.001*navRoll.kd*navRoll.derDiff) +
                    -2*0.5*(0.02*navY_vel.kp*errVelLoc.y    + 0*navY_vel.ki*navY_vel.intSum          + 0*navY_vel.kd*navY_vel.derDiff);
		    
  desCntrl.pitch  =  1*0.5*(0.01*navPitch.kp*errVecLoc.x + 0*0.002*navPitch.ki*navPitch.intSum   + 0*0.001*navPitch.kd*navPitch.derDiff) +
		     2*0.5*(0.02*navX_vel.kp*errVelLoc.x    + 0*navX_vel.ki*navX_vel.intSum 	       + 0*navX_vel.kd*navX_vel.derDiff);

		     
  if(desCntrl.roll > angleLimit)
  {
    desCntrl.roll = angleLimit;
  }
  else if(desCntrl.roll < -1*angleLimit)
  {
    desCntrl.roll = -1*angleLimit;
  }  
  
  if(desCntrl.pitch > angleLimit)
  {
    desCntrl.pitch = angleLimit;
  }
  else if(desCntrl.pitch < -1*angleLimit)
  {
    desCntrl.pitch = -1*angleLimit;
  }
  
     
  return 0;
}