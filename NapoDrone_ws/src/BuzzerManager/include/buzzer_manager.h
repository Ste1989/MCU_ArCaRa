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





mraa::Gpio* gpio_buzzer;


//topic per comando della pinza
ros::Subscriber buzzer_sub;


//funzioni per il tempo
timeval duration_time,current_time;
double elapsed_time_note;

//FUNZIONI
void sig_handler(int signo);
void init_pwm_gpio(int gpio_buzz);
void buzzerCallback(const std_msgs::Int32::ConstPtr& msg);
void tone_buzzer(int frequency, int duration);
