#include <ctime>
#include <cstdio>
#include <iostream>
#include "testCntrl.hh"
#include "rc_controller_comms.hh"

#include <ncurses.h>

#define YAW_CH 3
#define THRUST_CH 2
#define Z_CH 2
#define X_CH 1
#define Y_CH 0

extern PASBuddyBox pasBB;

struct ch_info_t {
  double cur_val;
  double pos_lim;
  double neg_lim;
  double scale;
  double offset;
};

//ch_info_t ch_info_cal[4];
ch_info_t ch_info_cur[4];
//static double ch0_pos_lim;
//static double ch0_neg_lim;

//static double ch1_pos_lim;
//static double ch1_neg_lim;

//static double ch2_pos_lim;
//static double ch2_neg_lim;

//static double ch3_pos_lim;
//static double ch3_neg_lim;

//static double thrust_scale;
//static double thrust_offset;
//static double pos_scale;
//static double pos_offset;
//static double yaw_scale;
//static double yaw_offset;

enum test_control_mode_t {
  KEYBOARD,
  RC,
  AUTO
} test_control_mode;

int test_control_rc_scale(void)
{
  int i;
  double diff_tmp;
  
  // scale value to be between -1 and 1
  for(i=0;i<4;i++)
  { 
    
    diff_tmp = (ch_info_cur[i].pos_lim - ch_info_cur[i].neg_lim);
    
    if(diff_tmp != 0) // ensure we don't divide by zero
    {
      ch_info_cur[i].scale = 2/(ch_info_cur[i].pos_lim - ch_info_cur[i].neg_lim);
      ch_info_cur[i].offset = -1*(ch_info_cur[i].pos_lim + ch_info_cur[i].neg_lim)/(ch_info_cur[i].pos_lim - ch_info_cur[i].neg_lim);
    }
    else // diff between max and min is zero, so something is amiss, but let someone else deal with it. Just ensure we don't crash.
    {
      // range will be approximatly 0 - 1, so scale to -1 to 1
      ch_info_cur[i].scale = 2;
      ch_info_cur[i].offset = -1;
    }
      
  }
  
  return 1;
}

int test_control_rc_update(void)
{
  int i;
  int do_scale = 0;
  int inputChannelCount;
  
  inputChannelCount = getBuddyBoxThreadInputChannelCount(&pasBB);
  mvprintw(29,10,"ch cnt: %d",inputChannelCount);
  
  for(i=0;i<4;i++)
  {
    
    ch_info_cur[i].cur_val = getBuddyBoxThreadInputChannelValue(&pasBB,i);    
    mvprintw(30+i,10,"ch%d: %f",i,ch_info_cur[i].cur_val);
    
    if(ch_info_cur[i].cur_val > ch_info_cur[i].pos_lim)
    {
      ch_info_cur[i].pos_lim = ch_info_cur[i].cur_val;
      do_scale = 1;
    }
    else if(ch_info_cur[i].cur_val < ch_info_cur[i].neg_lim)
    {
      ch_info_cur[i].neg_lim = ch_info_cur[i].cur_val;
      do_scale = 1;
    }
	
  }
  
  refresh();
  
  if (do_scale == 1)
  {
    test_control_rc_scale();
  }
  
  return 1;
}

int test_control_rc_cal(void)
{
  std::clock_t start_time;
  
  double diff_tmp;
  
  double wait_time = 10;
  double time_passed = 0;
  int allDone = 0;
  int i;

    for(i=0;i<4;i++)
    { 
      ch_info_cur[i].pos_lim = 0;
      ch_info_cur[i].neg_lim = 1;         
    }  
  
  
  clear();
  mvprintw(0,0,"Move controls through entire range of motion. You have 10s. GO!");
  refresh();
  
  start_time = std::clock();
  
  while(time_passed < wait_time)
  {
    test_control_rc_update();
    
    time_passed = (std::clock() - start_time)/((double) CLOCKS_PER_SEC);
  
  }
      
  return 1;
  
}

int test_control_init(void)
{
  rc_controller_comms_init();
  test_control_rc_cal();
  return 1;
}

int test_control_end(void)
{
  rc_controller_comms_end();
  return 1;
}

static int test_control_auto(desCntrl_T & desCntrl, gazebo::math::Vector3 & desLoc, const double & curSimTime, const int mav_number)
{
  static int state = 0;
  static double deltaTime = 0;
  static double tStep = 45;

  //rc_controller_comms_update(pasBB);
  
  switch (state)
  {
    case 0:
      if (curSimTime > deltaTime)
      {
	deltaTime = curSimTime + tStep;
	desLoc.z = 40 + mav_number * 3;
	desLoc.y = mav_number * 5;
	desCntrl.yaw = 0;
	state = 5;
      }
      break;
      
    case 6:
      if (curSimTime > deltaTime)
      {
	deltaTime = curSimTime + tStep;
	desCntrl.yaw = 3.14/2;
	state = 7;
      }
      break;

    case 7:
      if (curSimTime > deltaTime)
      {
	deltaTime = curSimTime + tStep;
	desCntrl.yaw = 0;
	state = 5;
      }
      break;
      
    case 1:
      if (curSimTime > deltaTime)
      {
	deltaTime = curSimTime + tStep;
	desLoc.x = 10;
	//desCntrl.pitch = .01;
	state = 2;
      }
      break;
      
    case 2:
      if (curSimTime > deltaTime)
      {
	deltaTime = curSimTime + tStep;
	desLoc.x = 0;
	//desCntrl.pitch = -.01;
	state = 3;
      }
      break;

    case 3:
      if (curSimTime > deltaTime)
      {
	deltaTime = curSimTime + tStep;
	desLoc.y = 10;
	//desCntrl.pitch = 0;
	//desCntrl.roll = .01;
	state = 4;
      }
      break;

    case 4:
      if (curSimTime > deltaTime)
      {
	deltaTime = curSimTime + tStep;
	desLoc.y = 0;
	//desCntrl.roll = -.01;
	state = 5;
      }
      break;


    case 5:
      if (curSimTime > deltaTime+tStep)
      {	
	return 0;
      }
      break;
      
    default:
      break;
  }
  
  return 1;
  
}

static int test_control_rc(desCntrl_T & desCntrl, gazebo::math::Vector3 & desLoc, const double & curSimTime, const int mav_number)
{
  unsigned int i, inputChannelCount;
  //const static double thrust_scale = 0.1;
  //const static double thrust_offset = -1 * 0;
  const static double pos_scale = 0.1;
  //const static double pos_offset = -1 * 0.5;
  const static double yaw_scale = 0.01;
  //const static double yaw_offset = -1 * 0.5;
  static double ch0_val;
  static double ch1_val;
  static double ch2_val;
  static double ch3_val;
//  static ch_info_t ch_info_cur[4];
  
//  ch_info_cur[YAW_CH].scale = yaw_scale;
//  ch_info_cur[THRUST_CH] = thrust_scale;
//  ch_info_cur[X_CH].scale = pos_scale;
//  ch_info_cur[Y_CH].scale = pos_scale;
//  ch_info_cur[Z_CH].scale = pos_scale;
  
//  inputChannelCount = getBuddyBoxThreadInputChannelCount(&pasBB_local);
  
//  ch0_val = getBuddyBoxThreadInputChannelValue(&pasBB_local,0);
//  ch1_val = getBuddyBoxThreadInputChannelValue(&pasBB_local,1);
//  ch2_val = getBuddyBoxThreadInputChannelValue(&pasBB_local,2);
//  ch3_val = getBuddyBoxThreadInputChannelValue(&pasBB_local,3);  

//  inputChannelCount = getBuddyBoxThreadInputChannelCount(&pasBB);
//  mvprintw(29,10,"ch cnt: %d",inputChannelCount);
//  
//  for(i=0;i<4;i++)
//  {
//    ch_info_cur[i].cur_val = getBuddyBoxThreadInputChannelValue(&pasBB,i);
//    mvprintw(30+i,10,"ch%d: %f",i,ch_info_cur[i].cur_val);
//  }
//  
//  refresh();

  test_control_rc_update();

  desCntrl.yaw = desCntrl.yaw + yaw_scale * (ch_info_cur[YAW_CH].scale * ch_info_cur[YAW_CH].cur_val + ch_info_cur[YAW_CH].offset);
  desLoc.x = desLoc.x + pos_scale * (ch_info_cur[X_CH].scale * ch_info_cur[X_CH].cur_val + ch_info_cur[X_CH].offset);
  desLoc.y = desLoc.y + pos_scale * (ch_info_cur[Y_CH].scale * ch_info_cur[Y_CH].cur_val + ch_info_cur[Y_CH].offset);
  desLoc.z = desLoc.z + pos_scale * (ch_info_cur[Z_CH].scale * ch_info_cur[Z_CH].cur_val + ch_info_cur[Z_CH].offset);
    
  if(desLoc.z < 0)
  {
    desLoc.z = 0;
  }
  
  
//  for (i = 0; i < inputChannelCount; i++)
//      printf("%f\t", getBuddyBoxThreadInputChannelValue(pasBB, i));
//  printf("\n");
  
  return 1;
}

int test_control_update(desCntrl_T & desCntrl, gazebo::math::Vector3 & desLoc, const double & curSimTime, const int mav_number)
{
  
  switch(mav_number)
  {
    case 1:
      test_control_mode = RC;
      break;
    case 2:
    case 3:
      test_control_mode = AUTO;
      break;
    default:
      test_control_mode = KEYBOARD;
      break;
  }
  
  switch(test_control_mode)
  {
    case RC:
      test_control_rc(desCntrl, desLoc, curSimTime, mav_number);
      break;
    case AUTO:
      test_control_auto(desCntrl, desLoc, curSimTime, mav_number);
      break;
    default:
	break;
  }
  
  return 0;
}