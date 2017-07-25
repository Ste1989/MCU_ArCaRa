//#ifndef NAVIGATION_H
//#define NAVIGATION_H

#include <ros/ros.h>
#include <stdio.h>
#include <sys/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "autopilot_manager/init_time.h"

using namespace std;


//subscriber ros
ros::Subscriber imu_sub;
ros::Subscriber mag_sub;
ros::Subscriber attitude_sub;
//client
ros::ServiceClient get_time_sec0;

sensor_msgs::Imu imu_state;
char new_imu_packet;
sensor_msgs::Imu attitude_state;
char new_attitude_packet;
sensor_msgs::MagneticField mag_state;
char new_mag_packet;


//calcola tempo
timeval current_time, imu_time, mag_time, attitude_time;
double elapsed_time_imu, elapsed_time_mag, elapsed_time_attitude;

//log file:
//0: nessun sensore
//1: solo imu
//2: mag
//3: imu +mag
//4: assetto
//5 : imu+mag+assetto
int log_file;
double secs_0;
int freq_ros_node;
//scrittirua su file
std::string log_imu_path;
std::string log_mag_path;
std::string log_attitude_path;
FILE* fd;















//dichiarazione delle callback
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
void attitude_cb(const sensor_msgs::Imu::ConstPtr& msg);
void mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg);
void init_global_var();

//#endif