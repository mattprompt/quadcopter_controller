//#include <iostream>
//
//int main(int argc, char **argv) {
//    std::cout << "Hello, world!" << std::endl;
//    return 0;
//}

//     ^x
//     |	
// y <-oz
//
//     2
//     |
// 4---+---3
//     |
//     1

#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <ctime>
#include <stdio.h>
#include <unistd.h>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include "common_cntrl.hh"
#include "gz_comms.hh"
#include "terminal_cntrl.hh"
#include "control_orientation.hh"
#include "control_navigation.hh"
#include "testCntrl.hh"
#include "mavlink_comms.hh"
#include <ncurses.h>

//ConstIMUPtr &IMU_data;


//ConstIMUPtr IMU_data_ptr = &IMU_data;

/**
 * 
 */
//int rotCalc(double_t wx, double_t wy, double_t wz)
static std::string mav_name;
static int mav_number;


int sys_init(int _argc, char *_argv[], char **_envVar)
{

  int i;
  
  term_init();    // Setup terminal comms
  
  switch(_argc)
  {
    case 3:
      mav_name = _argv[1];
      mav_number = atoi(_argv[2]);
      break;
    default:
      mav_name = "quad_2292_1";
      mav_number = 1;
      break;
  }

//  for(i=0; i< _argc; i++)
//  {
//    printw("arg %d:\n",i);
//    printw("%s\n",_argv[i]);
//  }  
//
//  while(*_envVar != NULL)
//  {
//    printw("%s\n",*(_envVar++));
//  }
//  
//  printw("%s\n",*_envVar);
//  refresh();
  
  //gazebo::setupClient(_argc, _argv);     // Load gazebo
  gazebo::setupClient();     // Load gazebo
  gz_comms_init(mav_name);
  control_navigation_init();
  control_orientation_init();
  mavlink_comms_init(mav_number);
  test_control_init();
  
  return 0;
}

int sys_end()
{
  //outfile.close();
  test_control_end();
  control_navigation_end();
  control_orientation_end();
  gz_comms_end();
  term_end();  
  gazebo::shutdown();
  
}

/////////////////////////////////////////////////
int main(int _argc, char *_argv[], char **_envVar)
{

  int allDone = 0;
  //char filename[100];  
  double dt = 0; // actual time step, seconds
  
  const static double freqFastLoop = 500;
  const static double freqSlowLoop = 20;
  const static double freqReallySlowLoop = 1;
  
  const static double dtFastLoop = 1/freqFastLoop;
  const static double dtSlowLoop = 1/freqSlowLoop;
  const static double dtReallySlowLoop = 1/freqReallySlowLoop;
  
  double dtPrevFastLoop = 0;
  double dtPrevSlowLoop = 0;
  double dtPrevReallySlowLoop = 0;
  
  double prevSimTime = 0;
  double curSimTime = 0;
  
  double prevTimeFastLoop = 0;
  double prevTimeSlowLoop = 0;
  double prevTimeReallySlowLoop = 0;
  
  motInfo mot;

  //int winCnt = 0;
  gazebo::math::Vector3 desLoc;
  desCntrl_T desCntrl;
  
  // General Startup
  sys_init(_argc, _argv, _envVar);


  
  //desLoc = 0;
  desCntrl.pitch = 0;
  desCntrl.roll = 0;
  desCntrl.thrust = 0;
  desCntrl.yaw = 0; //0.785
  

  while (!allDone)
  {
    //mvprintw(0,0,"%s\n\n",(diagnostics_data.DebugString()).c_str());
    //refresh();
    curSimTime = gazebo::msgs::Convert(diagnostics_data.sim_time()).Double();
    dt = curSimTime - prevSimTime;
    dtPrevFastLoop = curSimTime - prevTimeFastLoop;
    dtPrevSlowLoop = curSimTime - prevTimeSlowLoop;
    dtPrevReallySlowLoop = curSimTime - prevTimeReallySlowLoop;
    
    //allDone = test_cntrl(desLoc, curSimTime);
    if( dtPrevFastLoop >= dtFastLoop )
    {
      gz_comms_lock_mutex();
      control_orientation_update(desCntrl, mot, curSimTime);
      gz_comms_send_update(desLoc,desCntrl,mot);
      prevTimeFastLoop = curSimTime;
      gz_comms_unlock_mutex();
      
    }
    
    if( dtPrevSlowLoop >= dtSlowLoop )
    {
      gz_comms_lock_mutex();
      allDone |= term_handle_input(desCntrl, desLoc);
      allDone |= test_control_update(desCntrl,desLoc,curSimTime,mav_number);  
      control_navigation_update(desLoc,desCntrl,curSimTime);     
      prevTimeSlowLoop = curSimTime;
      gz_comms_unlock_mutex();
      
    }
    
    if( dtPrevReallySlowLoop >= dtReallySlowLoop )
    {
      mavlink_comms_send();
      prevTimeReallySlowLoop = curSimTime;
    }


  }

  // Make sure to shut everything down.
  sys_end();


  return 0;
}


