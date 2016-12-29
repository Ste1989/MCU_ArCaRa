#include "ros/ros.h"

#include <sstream>
#include "std_msgs/String.h"
#include <unistd.h>
#include <signal.h>
#include "mraa.hpp"
#include "mraa/pwm.h"
#include <std_msgs/Int32.h>

//sudo chmod 777 /sys/class/gpio/export
//sudo chmod 777 /sys/class/gpio/unexport
//sudo chmod 777 value 
//sudo chmod 777 direction
//sudo chmod 777 /sys/class/pwm/pwmchip0/export ecc..


mraa::Pwm* pwm;
mraa::Gpio* gpio;
mraa_pwm_context pwm2;
double time_open2close;
//topic per comando della pinza
ros::Subscriber gripper_sub;
ros::Publisher gripper_pub;

//FUNZIONI
void sig_handler(int signo);
void init_pwm_gpio(int pwm_pin, int gpio_pin);
void cmdgripperCallback(const std_msgs::Int32::ConstPtr& msg);
void cmd_gripper(int dir, int speed);
