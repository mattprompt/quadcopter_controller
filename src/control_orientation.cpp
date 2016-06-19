#include "control_orientation.hh"
#include <ncurses.h>

int control_orientation_init(void)
{
  body_orientation.SetFromEuler(0,0,0);
  return 0;
}

int control_orientation_end(void)
{
  return 0;
}

//int update_orientation(const gazebo::msgs::IMU & IMU_data_ptr,gazebo::math::Quaternion & body_orientation,const double_t dt)
//static int update_orientation(void)
//{
  // For now we're cheating and using the actual ideal value from the gazebo quad body.
  // Not possible in reality as the orientation would need to be estimated from the IMU data.
  
  //gazebo::msgs::Vector3d oldVec; 
  //gazebo::msgs::Vector3d newVec;
  //gazebo::math::Vector3 omega;
  //gazebo::math::Vector3 axis;
  
  //gazebo::math::Quaternion oldQuat = body_orientation; 
  //gazebo::math::Quaternion newQuat;
  
  //double angle;
  //double tmpValue = 0;
  //double M11, M12, M13, M21, M22, M23, M31, M32, M33;
  //double dOmegaX, dOmegaY, dOmegaZ;
  
  // tmp values
  //double q0_dot;
  //gazebo::math::Vector3 q123_dot;
  //gazebo::math::Quaternion q_dot;
 
  
  //omega = gazebo::msgs::Convert(IMU_data_ptr.angular_velocity());
//  oldQuat.GetAsAxis(axis,angle);
  
//  q0_dot = axis.Dot(omega);
//  q123_dot = axis.Cross(omega) + angle*omega;
//  q_dot.SetFromAxis(q123_dot,q0_dot);
//  q_dot.Integrate(
  
  //newQuat = oldQuat.Integrate(omega,dt);
//  *body_orientation = newQuat;
//  if(pose_data.pose_size() == 0)
//  {
//    __asm__("int $3");
//  }
//  body_orientation = gazebo::msgs::Convert(pose_data.pose(0).orientation()); // We're cheeting using actual orientaion
//  
//  return 0;
//}

/**
 * 
 * int update_orientation(IMU_data,body_orientation)
 * 
 */
// int update_orientation(gazebo::msgs::IMU *IMU_data_ptr,gazebo::msgs::Vector3d *body_orientation, double_t dt)
// {
//   gazebo::msgs::Vector3d oldVec = *body_orientation;
//   gazebo::msgs::Vector3d newVec;
//   double tmpValue = 0;
//   double M11, M12, M13, M21, M22, M23, M31, M32, M33;
//   double dOmegaX, dOmegaY, dOmegaZ;
//   
//   // Do Direct Cosine Matrix Calc
//   //     | 11 12 13 |
//   // M = | 21 22 23 |
//   //     | 31 32 33 |
//   
//   dOmegaX = IMU_data_ptr->angular_velocity().x() * dt;
//   dOmegaY = IMU_data_ptr->angular_velocity().y() * dt;
//   dOmegaZ = IMU_data_ptr->angular_velocity().z() * dt;
//   
//   M11 =  1;
//   M22 =  1 * M11;  
//   M33 =  1 * M11;
//   
//   M12 = -1 * dOmegaZ;
//   M21 = -1 * M12;
//   
//   M13 =  1 * dOmegaY;
//   M31 = -1 * M13;
//   
//   M23 = -1 * dOmegaX;
//   M32 = -1 * M23;
// 
//   tmpValue = oldVec.x() * M11 + oldVec.y() * M21 + oldVec.z() * M31;
//   newVec.set_x(tmpValue);
//   
//   tmpValue = oldVec.x() * M12 + oldVec.y() * M22 + oldVec.z() * M32;
//   newVec.set_y(tmpValue);
//   
//   tmpValue = oldVec.x() * M13 + oldVec.y() * M23 + oldVec.z() * M33;
//   newVec.set_z(tmpValue);
//   
//   *body_orientation = newVec;
//   
//   return 0;
// }

int control_orientation_update(desCntrl_T & desCntrl, motInfo & mot, const double & curTime)
{

  
  //static gazebo::math::Quaternion quarDes; 
  //const gazebo::msgs::IMU & IMU_data_ptr
  static pidInfo motRoll; 	// X
  static pidInfo motYaw;	// Z
  static pidInfo motPitch;	// Y
  static double prevTime = 0;
  double dt;
  double tMult;
  double errYawLim = 0.1;
  
  dt = curTime - prevTime;
  //update_orientation();
  
  //quarDes.SetFromEuler(0,0,0);
  
  tMult = std::sqrt( std::pow(desCntrl.thrust,2) + std::pow(body_orientation.x,2) + std::pow(body_orientation.y,2));
  
  desCntrl.errRoll  = desCntrl.roll  - body_orientation.GetRoll();
  desCntrl.errPitch = desCntrl.pitch - body_orientation.GetPitch();
  desCntrl.errYaw   = desCntrl.yaw   - body_orientation.GetYaw();

//#warning "Yaw Hack In Place!"
//  desCntrl.errYaw = 0; // Yaw Hack
  
  if (desCntrl.errYaw > errYawLim)
  {
    desCntrl.errYaw = errYawLim;
  }
  else if (desCntrl.errYaw < -1*errYawLim)
  {
    desCntrl.errYaw = -1*errYawLim;
  }
  
  //clear();
  mvprintw(0,0,"Yaw: %f\n",body_orientation.GetYaw());
  printw("errYaw: %f\n",desCntrl.errYaw);
  refresh();
  //vecDes.
  
//  motRoll.curVal = -1*motRoll.kp*IMU_data_ptr->angular_velocity().x() + 
//		   -1*motRoll.ki*(IMU_data_ptr->angular_velocity().x()*dt + motRoll.preVal) +
//		   -1*motRoll.kd*(IMU_data_ptr->angular_velocity().x() - motRoll.preVal)/dt;
  
//   motRoll.curVal = 100*motRoll.kp*(desCntrl->errRoll);// + 
		   //motRoll.ki*(IMU_data_ptr->angular_velocity().x()*dt + motRoll.preVal) +
		   //motRoll.kd*(IMU_data_ptr->angular_velocity().x() - motRoll.preVal)/dt;
  
//  motPitch.curVal = -1*motPitch.kp*IMU_data_ptr->angular_velocity().y() + 
//		   -1*motPitch.ki*(IMU_data_ptr->angular_velocity().y()*dt + motPitch.preVal) +
//		   -1*motPitch.kd*(IMU_data_ptr->angular_velocity().y() - motPitch.preVal)/dt; 
//		   
//  motYaw.curVal = -1*motYaw.kp*IMU_data_ptr->angular_velocity().z() + 
//		   -1*motYaw.ki*(IMU_data_ptr->angular_velocity().z()*dt + motYaw.preVal) +
//		   -1*motYaw.kd*(IMU_data_ptr->angular_velocity().z() - motYaw.preVal)/dt;
//		   
 
  // Calc integral
  motRoll.intSum  = motRoll.intSum  + desCntrl.errRoll*dt;
  motPitch.intSum = motPitch.intSum + desCntrl.errPitch*dt;
  motYaw.intSum   = motYaw.intSum   + desCntrl.errYaw*dt;

  // Calc Derivative
  motRoll.derDiff  = (desCntrl.errRoll  - motRoll.preVal)/dt;
  motPitch.derDiff = (desCntrl.errPitch - motPitch.preVal)/dt;
  motYaw.derDiff   = (desCntrl.errYaw   - motYaw.preVal)/dt;
  
  motRoll.curVal  = 1*0.1*4*(50*motRoll.kp*(desCntrl.errRoll)   + 0*10*motRoll.ki*motRoll.intSum   + 0*3*10*motRoll.kd*motRoll.derDiff);
  motPitch.curVal = 1*0.1*4*(50*motPitch.kp*(desCntrl.errPitch) + 0*10*motPitch.ki*motPitch.intSum + 0*3*10*motPitch.kd*motPitch.derDiff);
  motYaw.curVal   = 2*0.1*.05*(50*motYaw.kp*(desCntrl.errYaw)   + 0*10*motYaw.ki*motYaw.intSum     + 0*10*motYaw.kd*motYaw.derDiff);
  
  // Save err values
  motRoll.preVal  = desCntrl.errRoll;
  motPitch.preVal = desCntrl.errPitch;
  motYaw.preVal   = desCntrl.errYaw;
  
  
//     ^x
//     |	
// y <-oz
//
//     2
//     |
// 4---+---3
//     |
//     1
//
// + Roll -> -y -> -M3, +M4
// + Pitch -> +x -> -M2, +M1
// + Yaw -> CC
  
  mot.mot1 = 1 * tMult +  0 * motRoll.curVal + 	1 * motYaw.curVal +  1 * motPitch.curVal;
  mot.mot3 = 1 * tMult + -1 * motRoll.curVal + -1 * motYaw.curVal +  0 * motPitch.curVal;
  mot.mot2 = 1 * tMult +  0 * motRoll.curVal +  1 * motYaw.curVal + -1 * motPitch.curVal;
  mot.mot4 = 1 * tMult +  1 * motRoll.curVal + -1 * motYaw.curVal +  0 * motPitch.curVal;
  
  if(mot.mot1 < 0)
  {
    mot.mot1 = 0;
  }
  
  if(mot.mot2 < 0)
  {
    mot.mot2 = 0;
  }
  
  if(mot.mot3 < 0)
  {
    mot.mot3 = 0;
  }
  
  if(mot.mot4 < 0)
  {
    mot.mot4 = 0;
  }
    
  return 0;  

}

