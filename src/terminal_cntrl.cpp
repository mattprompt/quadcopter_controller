#include "terminal_cntrl.hh"
#include <ncurses.h>


int term_init(void)
{
  initscr();
  //raw();
  timeout(0);
  nodelay(stdscr,TRUE);
  cbreak();
  echo();
  keypad(stdscr,TRUE);
  printw("Term started\n");
  refresh();
  return 0;
}

int term_end(void)
{	
  endwin();
  return 0;
}

///
///
///

int term_handle_input(desCntrl_T & desCntrl, gazebo::math::Vector3 & desLoc)
{
  int ch;
  
  do
  {
    ch = getch();
    switch(ch)
    {
      case KEY_DOWN:
	desLoc.x--;
	break;
      case KEY_UP:
	desLoc.x++;
	break;
      case KEY_LEFT:
	desLoc.y--;
	break;
      case KEY_RIGHT:
	desLoc.y++;
	break;
      case 'a':
      case 'A':
	desCntrl.thrust_hover++;
	break;
      case 'f':
      case 'F':
	desCntrl.thrust_hover--;
	break;
      case 'e':
	desLoc.z++;
	break;
      case 'l':
	desCntrl.yaw += .01;
	break;
      case 'j':
	desCntrl.yaw -= .01;
	break;
      case 'x':
	desLoc.z--;
	break;
      case 'q':
	return 1;
      default:
	break;
    }
  } while(ch != ERR);
  

//    int i=0;
//    while(!i)
//    {
//        usleep(1);
//        i=getch();
//        printw("%d ",i);
//        if(i>0)
//            i=1;
//        else
//            i=0;
//    }

/*  char buf[255];
  int i;
  if ((std::cin.rdbuf()->in_avail() > 0)) 
  {

    std::cin >> buf;
    
    //std::cout << (int)buf;
    
    for (i=0;i<strlen(buf);i++)
    {
      //std::cout << "0x" << std::hex << (unsigned short)((unsigned char) buf[i]) <<" " << std::endl;
      //std::cout << std::hex << static_cast<int>(buf[i]) << std::endl;
      switch(buf[i])
      {
	case 'a':
	case 'A':
	  thrust[0]++;
	  break;
	case 'f':
	case 'F':
	  thrust[0]--;
	  break;
	case 'q':
	  return 1;
	default:
	  break;
      }
    }
  }*/  
  
  return 0;
}

/*
/// Output to terminal
    // print to screen
    if(winCnt++ == 100)
    {
      winCnt = 0;
      clear();
      move(0,0);
  //    std::cout << "\033[1;1H";//<< std::endl;
  //    std::cout << std::string(30,'\n');// << std::endl;
  //    std::cout << "\033[1;1H";// << std::endl;
  //    std::cout << body_orientation.DebugString() << "\n";
  //    std::cout << IMU_data.DebugString() << "\n";
      //printw("%s\n\n",(IMU_data.DebugString()).c_str());
  //    std::cout << "thrust: " << thrust << "\n\n";// << std::endl;
      printw("time: %f, dt: %f\n",curSimTime,dt);
      printw("thrust: %f\n",desCntrl.thrust_hover);
  //    std::cout << "mot1: " << mot.mot1 + thrust << "\n";
      printw("mot1: %f\n", mot.mot1);
  //    std::cout << "mot2: " << mot.mot2 + thrust << "\n";
      printw("mot2: %f\n", mot.mot2);
  //    std::cout << "mot3: " << mot.mot3 + thrust << "\n";
      printw("mot3: %f\n", mot.mot3);
  //    std::cout << "mot4: " << mot.mot4 + thrust << "\n\n";
      printw("mot4: %f\n", mot.mot4);
      
      printw("cntrl Thrust: %f\n", desCntrl.thrust);
      printw("cntrl Roll: %f\n", desCntrl.roll);
      printw("cntrl Pitch: %f\n", desCntrl.pitch);
      printw("cntrl Yaw: %f\n", desCntrl.yaw);
      
      printw("DesLoc - x: %f, y: %f, z: %f\n", desLoc.x, desLoc.y, desLoc.z);
      
  //    std::cout << "roll: " << body_orientation.GetRoll() << "\n";
      printw("roll: %f\n", body_orientation.GetRoll());
  //    std::cout << "pitch: " << body_orientation.GetPitch() << "\n";
      printw("pitch: %f\n", body_orientation.GetPitch());
  //    std::cout << "yaw: " << body_orientation.GetYaw() << "\n" << std::endl;
      printw("yaw: %f\n", body_orientation.GetYaw());
      //std::cout << gazebo::msgs::Convert(body_orientation).DebugString() << "\n" << std::endl;
      printw("%s\n\n",(gazebo::msgs::Convert(body_orientation).DebugString()).c_str());
      //clear();
      //move(0,0);
      //pose_data.
      //printw("%s\n\n", (pose_data.pose(0).DebugString()).c_str());
      //printw("x: %s\n", (pose_data.pose(0).position().DebugString()).c_str());
      refresh();
    }
    */