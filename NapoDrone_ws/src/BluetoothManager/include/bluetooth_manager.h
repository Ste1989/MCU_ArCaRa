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
#include <geometry_msgs/PoseStamped.h>
#include <sys/time.h>
#include <queue>
#include <string>       // std::string
#include <sstream>      // std::stringstream
//Subscribe
ros::Subscriber pose_sub;

//client
ros::ServiceClient get_time_sec0;

int freq_ros_node;

double xt_;
double xt;
double yt_;
double yt;
double zt_;
double zt;
 
//
int num_campioni_delta;

//coda di trasmissione
std::queue<double> coda_position_x;
std::queue<double> coda_position_y;
std::queue<double> coda_position_z;

/***************************FUNZIONI************************************************************************/
void init_global_var();
void write_to_serial(int* serial);
void read_from_serial(int* serial);
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
int serial_init(int* fd,const char* seriale_dev);
void ekf_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);


