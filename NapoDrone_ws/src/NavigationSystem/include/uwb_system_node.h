//#ifndef UWB_SYSTEM_H
//#define UWB_SYSTEM_H

#include <ros/ros.h>
#include <stdio.h>
#include <sys/time.h>
#include "uwb_manager/RangeUwb.h"
#include "autopilot_manager/init_time.h"
#include <Eigen/Dense>
#include <fstream>
#include <malloc.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::ArrayXd;

//subscriber ros
ros::Subscriber rangeUWB_sub;
//client
ros::ServiceClient get_time_sec0;

uwb_manager::RangeUwb range_uwb;
char new_range_packet;


//timeval
timeval  current_time, filter_time, range_time;
double elapsed_time_filter, elapsed_time_range;


//per lettura file log
int Num_measure;
double* time_log, *range1_log, *range2_log, *range3_log, *range4_log;

//ancore
Vector3d anchor0;
Vector3d anchor1;
Vector3d anchor2;
Vector3d anchor3;

//somma nell'intervallo dei range
uwb_manager::RangeUwb sum_range_dt;

//EKF///////////////////////////////////////
bool first_cycle_EKF;
MatrixXd A(6,6);
VectorXd x_k(6);
ArrayXd  d_hat(4);
MatrixXd H(4,6);
MatrixXd R(4,4);
MatrixXd P(6,6);
MatrixXd Q(6,6);
MatrixXd K(6,4);
MatrixXd anchor_pos(4,3);
int freq_filter;
double time_ms;
double dt_filter ;
/////////////////////////////////////////////

bool log_file;
bool debug;
double secs_0;
int freq_ros_node;

FILE* fd;
std::string log_uwb_path;
//dichiarazione delle callback
void rangeUWB_cb(const uwb_manager::RangeUwb::ConstPtr& msg);
void init_global_var();
void EKF_solo_range(VectorXd range,  double dt, VectorXd& position_estimated);
void triangolazione_range(VectorXd range,  Vector3d& pos_triangolata);
void leggi_file_debug();

//#endif