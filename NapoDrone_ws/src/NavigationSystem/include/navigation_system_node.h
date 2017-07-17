//#ifndef NAVIGATION_H
//#define NAVIGATION_H

#include <ros/ros.h>
#include <stdio.h>
#include <sys/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

using namespace std;


//subscriber ros
ros::Subscriber imu_sub;
ros::Subscriber mag_sub;
ros::Subscriber attitude_sub;


sensor_msgs::Imu imu_state;
char new_imu_packet;
sensor_msgs::Imu attitude_state;
char new_attitude_packet;
sensor_msgs::MagneticField mag_state;
char new_mag_packet;


//log file:
//0: nessun sensore
//1: solo imu
//2: mag
//3: imu +mag
//4: assetto
//5 : imu+mag+assetto
int log_file;
double secs_0;
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