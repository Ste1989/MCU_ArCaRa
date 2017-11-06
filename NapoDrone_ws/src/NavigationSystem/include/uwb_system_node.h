//#ifndef UWB_SYSTEM_H
//#define UWB_SYSTEM_H

#include <ros/ros.h>
#include <stdio.h>
#include <sys/time.h>
#include "uwb_manager/RangeUwb.h"
#include "autopilot_manager/init_time.h"

using namespace std;


//subscriber ros
ros::Subscriber rangeUWB_sub;
//client
ros::ServiceClient get_time_sec0;

uwb_manager::RangeUwb range_uwb;
char new_range_packet;


//calcola tempo
timeval current_time, range_time;
double elapsed_time_range;

bool log_file;
double secs_0;
int freq_ros_node;

FILE* fd;
std::string log_uwb_path;
//dichiarazione delle callback
void rangeUWB_cb(const uwb_manager::RangeUwb::ConstPtr& msg);
void init_global_var();

//#endif