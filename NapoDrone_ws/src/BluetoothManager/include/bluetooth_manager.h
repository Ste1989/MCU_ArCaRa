//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================
#include "ros/ros.h"

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>

#include <sys/time.h>


//client
ros::ServiceClient get_time_sec0;

int freq_ros_node;


/***************************FUNZIONI************************************************************************/
void write_to_serial(int* serial);
void read_from_serial(int* serial);
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
int serial_init(int* fd,const char* seriale_dev);


