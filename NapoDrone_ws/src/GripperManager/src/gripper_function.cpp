#include "gripper_manager.h"
/****************************************************************************/
/*                                                                          */
/*             SIG HANDLER                                                  */
/***************************************************************************/
void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing GPIO e PWM\n");
          delete gpio;
          delete pwm;
    }
    return;
}

/****************************************************************************/
/*                                                                          */
/*            INIT                                                          */
/***************************************************************************/
void init_pwm_gpio(int pwm_pin, int gpio_pin)
{

  //Init PWM pin and ENABLE
  //pwm = new mraa::Pwm(pwm_pin);
  pwm2 = mraa_pwm_init(pwm_pin);
  //if (pwm == NULL) 
  //  ROS_ERROR("non sono riusciuto a confighurare il pwm");
  //periodo 2ms
  //pwm->period(2000);
  //pwm->enable(1);
  //pwm->write(0);
  mraa_pwm_period_us(pwm2, 2000);
  mraa_pwm_enable(pwm2, 1);
  mraa_pwm_write(pwm2, 0);
  ROS_INFO("PWM CONFIGURATO");

  // Init GPIO pin for direction
  gpio = new mraa::Gpio(gpio_pin);
  if (gpio == NULL) 
    ROS_ERROR("non sono riusciuto a configurare il pin");
  mraa::Result response = gpio->dir(mraa::DIR_OUT);
  ROS_INFO("PIN IN USCITA");

}

/****************************************************************************/
/*                                                                          */
/*             COMMAND GRIPPPER                                             */
/*                                                                          */
/***************************************************************************/
void cmdgripperCallback(const std_msgs::Int32::ConstPtr& msg)
{   
    double speed = 0;
    double dir = 0;
    //msg->data contiene 0: NO_GRIPPER_REQ,1: CLOSE,2: OPEN
    //CHIUDO
    if(msg->data == 1)
    {
      dir = 1;
      speed = 1;
      ROS_INFO("CHIUDO PINZA CMD");

    }
    //APRO
    if(msg->data == 2)
    {
      dir = 0;
      speed = 1;
      ROS_INFO("APRO PINZA CMD");
    }

    //se ho ricevuto un pacchetto di comando valido
    if(msg->data != 0)
    {
  		cmd_gripper(dir,speed);
  		std_msgs::Int32 msg;
  		msg.data = dir;
  		gripper_pub.publish(msg);
      
    }else
    {
      pwm->write(0);
    }
    
}

/****************************************************************************/
/*                                                                          */
/*             GRIPPER FUNCTION                                            */
/*                                                                          */
/***************************************************************************/
void cmd_gripper(int dir, int speed)
{
	//dir = 0 OPEN
	//dir = 1 CLOSE
	gpio->write(dir);
    //pwm->enable(1);
    mraa_pwm_write(pwm2, 1);
    ROS_INFO("ESEGUO");
    //dopo qualche secondo secondi non comando più la pinza
    usleep(1000000 * time_open2close);
    ROS_INFO("STOP");
    mraa_pwm_write(pwm2, 0);
    gpio->write(0);
    //pwm->enable(0);
    


    return;

}