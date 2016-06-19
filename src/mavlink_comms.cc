/*******************************************************************************
  Copyright (C) 2010  Bryan Godbolt godbolt ( a t ) ualberta.ca
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  ****************************************************************************/
/*
  This program sends some data to qgroundcontrol using the mavlink protocol.  The sent packets
  cause qgroundcontrol to respond with heartbeats.  Any settings or custom commands sent from
  qgroundcontrol are printed by this program along with the heartbeats.
  
  
  I compiled this program sucessfully on Ubuntu 10.04 with the following command
  
  gcc -I ../../pixhawk/mavlink/include -o udp-server udp-server-test.c
  
  the rt library is needed for the clock_gettime on linux
  */

/* Based on
 * http://qgroundcontrol.org/dev/mavlink_linux_integration_tutorial
 */

/* These headers are for QNX, but should all be standard on unix/linux */
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#endif
  
/* This assumes you have the mavlink headers on your include path
  or in the same folder as this source file */
#include "include/common/mavlink.h"
  
  
#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

static struct sockaddr_in gcAddr; 
static struct sockaddr_in locAddr;
uint8_t buf[BUFFER_LENGTH];
int sock;
mavlink_message_t msg;
static int mav_number;

uint64_t microsSinceEpoch(void)
{
  
    struct timeval tv;
  
    uint64_t micros = 0;
  
    gettimeofday(&tv, NULL);  
    micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
  
    return micros;
}

int mavlink_comms_init(int mav_number_in)
{
  char target_ip[100];
  mav_number = mav_number_in;

  //int success = 0;

  // Change the target ip if parameter was given
  strcpy(target_ip, "127.0.0.1");
//  if (argc == 2)
//  {
//	  strcpy(target_ip, argv[1]);
//  }
  
  sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  memset(&locAddr, 0, sizeof(locAddr));
  locAddr.sin_family = AF_INET;
  locAddr.sin_addr.s_addr = INADDR_ANY;
  locAddr.sin_port = htons(14550+mav_number);

  /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
  if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
  {
	  perror("error bind failed");
	  close(sock);
	  exit(EXIT_FAILURE);
  } 

  /* Attempt to make it non blocking */
  if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
	  fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
	  close(sock);
	  exit(EXIT_FAILURE);
  }


  memset(&gcAddr, 0, sizeof(gcAddr));
  gcAddr.sin_family = AF_INET;
  gcAddr.sin_addr.s_addr = inet_addr(target_ip);
  gcAddr.sin_port = htons(14550);
  
  return 0;
}

int mavlink_comms_end(void)
{
  return 0;
}

int mavlink_comms_receive(void)
{
  //struct sockaddr_in fromAddr;
  socklen_t fromlen;
  ssize_t recsize;
  unsigned int temp = 0;
  int i = 0;
  
  memset(buf, 0, BUFFER_LENGTH);
  recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
  if (recsize > 0)
  {
	  // Something received - print out all bytes and parse packet
	  mavlink_message_t msg;
	  mavlink_status_t status;

	  printf("Bytes Received: %d\nDatagram: ", (int)recsize);
	  for (i = 0; i < recsize; ++i)
	  {
		  temp = buf[i];
		  printf("%02x ", (unsigned char)temp);
		  if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
		  {
			  // Packet received
			  printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
		  }
	  }
	  printf("\n");
  }
  memset(buf, 0, BUFFER_LENGTH);
  //sleep(1); // Sleep one second
  return 0;
}

int mavlink_comms_send(void)
{
  int bytes_sent;
  uint16_t len;
  static float position[6] = {};
  

  position[0] = 33;
  position[1] = -117;
  position[2] = 300+mav_number;
  position[3] = 0;
  position[4] = 0;
  position[5] = 0;  
  
  /*Send Heartbeat */
  mavlink_msg_heartbeat_pack(mav_number, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

  /* Send Status */
  mavlink_msg_sys_status_pack(mav_number, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));

  /* Send Local Position */
  mavlink_msg_local_position_ned_pack(mav_number, 200, &msg, microsSinceEpoch(), 
								  position[0], position[1], position[2],
								  position[3], position[4], position[5]);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

  /* Send attitude */
  mavlink_msg_attitude_pack(mav_number, 200, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
  
  return 0;
}


