#include "ros/ros.h"

#include <sstream>
#include "std_msgs/String.h"
#include <unistd.h>
#include <signal.h>
#include "mraa.hpp"
#include "mraa/pwm.h"
#include <geometry_msgs/Vector3.h>

//sudo chmod 777 /sys/class/gpio/export
//sudo chmod 777 /sys/class/gpio/unexport
//sudo chmod 777 value 
//sudo chmod 777 direction
//sudo chmod 777 /sys/class/pwm/pwmchip0/export ecc..

//mraa::Pwm* pwm;
mraa::Pwm* pwm;
mraa::Gpio* gpio;

int running;
/****************************************************************************/
/*                                                                          */
/*             SIG HANDLER                                                  */
/***************************************************************************/
/*void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing PWM nicely\n");
        running = -1;
    }
}
// Callback for gripper: opening or closing 
*/
/****************************************************************************/
/*                                                                          */
/*             COMMAND GRIPPPER                                             */
/***************************************************************************/
void cmdgripperCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{   
    double speed = 0.0f;
    int us = 0;
    int dir = 0;
    // Parsing data
    speed = msg->x;
    us = msg->y;
    dir = msg->z;
 
    // Set the output duty-cycle percentage, as a float
    // Values above or below this range will be set at either 0.0f or 1.0f
    speed = speed > 1.0f ? 1.0f : speed;
    speed = speed < 0.0f ? 0.0f : speed;
    pwm->write(speed);
 
    // Set direction : 0 o 1
    gpio->write(dir);
 
    //delete pwm;
 
    //return MRAA_SUCCESS;*/
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "gripper_manager");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //leggi launch file
  int pwm_pin;
  n.param<int>("/gripper_manager/pwm_pin", pwm_pin, 0);
  //TOPIC SUBSCRIBE OR PUBLISH
  ros::Subscriber cmd_gripper_sub = n.subscribe("cmdgripper", 1000, cmdgripperCallback);

  ros::Rate loop_rate(100);

  /*********Init*************************/
  //signal(SIGINT, sig_handler);
  // Init PWM pin and ENABLE
  pwm = new mraa::Pwm(pwm_pin);
  if (pwm == NULL) 
    ROS_ERROR("non sono riusciuto a aprire il in 3");
  //periodo 2ms
  pwm->period(2000);
  pwm->enable(true);
  pwm->write(1);
  ROS_INFO("PWM CONFIGURATO");

     
  // Init GPIO pin for direction
  gpio = new mraa::Gpio(37);
  if (gpio == NULL) 
    ROS_ERROR("non sono riusciuto a aprire il in 37");
  mraa::Result response = gpio->dir(mraa::DIR_OUT);
  ROS_INFO("PIN IN USCITA");
  gpio->write(1);
  /*******Ciclo******************************/
  while (ros::ok())
  {

    gpio->write(1);
    ros::spin();

    loop_rate.sleep();
  }


  return 0;
}