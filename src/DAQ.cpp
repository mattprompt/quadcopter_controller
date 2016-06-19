// This module will eventually handle data aquisition, but right now it's just a bunch of random notes.
// DAQ.cpp is currently not part of the build 

// Write to Files

  /////////////////////////////////////////////////////
  // Setup output file headers
  //filename << (now->tm_year + 1900) << '_' << (now->tm_mon + 1) << '_' <<  now->tm_mday << ".csv";

  outfile.open ("outfile1.csv");
  vecOutFile.open("vecOutFile1.csv");
//  outfile.open (filename);
  outfile    << "time,roll,pitch,yaw,m1,m2,m3,m4\n";
  vecOutFile << "time,curX,curY,curZ,desX,desY,desZ,errX,errY,errZ,errLocX,errLocY,errLocZ,desThrust,desRoll,desPitch,errRoll,errPitch,errYaw,desYaw\n";
 
  //////////////////////////////////////////////////////


    outfile << tSamp << "," 
	    << body_orientation.GetRoll() << "," << body_orientation.GetPitch() << "," << body_orientation.GetYaw() << ","
	    << mot.mot1 << "," << mot.mot2 << "," << mot.mot3 << "," << mot.mot4 << "\n";  
	    
	    
  vecOutFile << tSamp << "," 
	     << tmpVec.x         << "," << tmpVec.y          << "," << tmpVec.z        << ","
	     << desLoc.x         << "," << desLoc.y          << "," << desLoc.z        << ","
	     << errVec.x         << "," << errVec.y          << "," << errVec.z        << ","
	     << errVecLoc.x      << "," << errVecLoc.y       << "," << errVecLoc.z     << ","
	     << desCntrl.thrust  << "," << desCntrl.roll     << "," << desCntrl.pitch  << ","
	     << desCntrl.errRoll << "," << desCntrl.errPitch << "," << desCntrl.errYaw << ","
	     << desCntrl.yaw     << "\n"; 