/**
@file gripperTest.cpp
@brief Send open and close command to gripper via bluetooth
@date June 2014

BEFORE running this node:
from bluetooth manager (the bluetooth symbol that is further to the left):
1) click on devices and remove all devices there
2) click setup new device, password is 1234
3) should pop up with message successfully opened port /dev/rfcomm0 
4) do the same for the other bluetooth (this time, port /dev/rfcomm0)

5) run this node now

Arduino code is at:
/home/robocuphome/Dropbox/Robocup@Home/ProjectWork//ObjectGrasping/Gripper/BlueTooth comm
*/

#include "ros/ros.h"
#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <iostream>

#include "utilities/gripperTestRequest.h"
#include "utilities/gripperTestStatus.h"

#define OPEN 0x00
#define CLOSE 0xFF

using namespace std;

// File descriptor
int  fd, fd_l, fd_r;
ros::Publisher pub;

void
gripperCB(utilities::gripperTestRequest req) {

  unsigned char toSend;
  
  if (req.cmd == req.CMD_OPEN) {
    toSend = OPEN;
  } else if (req.cmd == req.CMD_CLOSE) {
    toSend = CLOSE;
  } else {
    ROS_ERROR("gripperTest: Error in msg received");
    exit(EXIT_FAILURE);
  }
  
  //**WRITE
  int n_written = 0;
  if (req.limb==req.LEFT){
  	n_written = write(fd_l, &toSend, 1);
  } else if (req.limb == req.RIGHT) {
    n_written = write(fd_r, &toSend, 1);
  } else if (req.limb == req.BOTH) {
    n_written = write(fd_l, &toSend, 1);
    n_written = write(fd_r, &toSend, 1);
  } else {
    ROS_ERROR("gripperTest: Error in msg received");
    exit(EXIT_FAILURE);
  }
  
  cerr<<"n_written: "<<n_written<<endl; 
  cerr<<"byte sent: "<<toSend<<endl;   

  sleep(2); // delay for 2 seconds
  
  //publish data back
  utilities::gripperTestStatus status;
  status.positionReached = true; //TEST
  pub.publish(status);

}

void
initPort(int fd) {
  /* Error Handling */
  if ( fd < 0 ) {
    std::cout << "Error " << errno << " opening port " << ": " << strerror (errno) << endl;
    exit(EXIT_FAILURE);
  }
  /* *** Configure Port *** */
  struct termios tty;
  struct termios tty_old;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr ( fd, &tty ) != 0 )
  {
  cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
  }

  /* Save old tty parameters */
  tty_old = tty;

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)B57600);
  cfsetispeed (&tty, (speed_t)B57600);

  /* Setting other Port Stuff */
  
  tty.c_cflag     &=  ~PARENB;        // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;
/*
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= B57600 | CS8;
  tty.c_cflag |= CLOCAL;
  tty.c_oflag = tty.c_lflag=0;
*/  
  tty.c_cflag     &=  ~CRTSCTS;       // no flow control
  tty.c_cc[VMIN]      =   1;                  // read doesn't block
  tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush( fd, TCIFLUSH );
  if ( tcsetattr ( fd, TCSANOW, &tty ) != 0)
  {
  cout << "Error " << errno << " from tcsetattr" << endl;
  }

  cout<<"Setup completed"<<endl;

}

int
main (int argc, char **argv) {
	fd_l = open( "/dev/rfcomm0", O_RDWR| O_NOCTTY | O_NDELAY);
	fd_r = open( "/dev/rfcomm1", O_RDWR| O_NOCTTY | O_NDELAY);
	
  initPort(fd_l);
	initPort(fd_r);

  ros::init(argc,argv, "gripper_test_both");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("gripper_test_both/request", 1, gripperCB);
	pub = n.advertise<utilities::gripperTestStatus>("gripper_test_both/status", 1);

  ros::spin();


}
