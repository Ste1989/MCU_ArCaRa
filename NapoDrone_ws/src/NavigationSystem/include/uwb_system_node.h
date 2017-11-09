//#ifndef UWB_SYSTEM_H
//#define UWB_SYSTEM_H

#include <ros/ros.h>
#include <stdio.h>
#include <sys/time.h>
#include "uwb_manager/RangeUwb.h"
#include "autopilot_manager/init_time.h"
#include <Eigen/Dense>

using namespace std;
using Eigen::MatrixXd;

//subscriber ros
ros::Subscriber rangeUWB_sub;
//client
ros::ServiceClient get_time_sec0;

uwb_manager::RangeUwb range_uwb;
char new_range_packet;

//timeval
timeval  current_time, filter_time, range_time;
double elapsed_time_filter, elapsed_time_range;


//ancore
double anchor0[3];
double anchor1[3];
double anchor2[3];
double anchor3[3];

//somma nell'intervallo dei range
uwb_manager::RangeUwb sum_range_dt;


bool log_file;
double secs_0;
int freq_ros_node;

FILE* fd;
std::string log_uwb_path;
//dichiarazione delle callback
void rangeUWB_cb(const uwb_manager::RangeUwb::ConstPtr& msg);
void init_global_var();
void EKF_solo_range(double range0,double range1,double range2,double range3,double dt);

//#endif