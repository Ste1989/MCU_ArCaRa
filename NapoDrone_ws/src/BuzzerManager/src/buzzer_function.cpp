#include "buzzer_manager.h"
/****************************************************************************/
/*                                                                          */
/*             SIG HANDLER                                                  */
/***************************************************************************/
void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing GPIO e PWM\n");
     
    }
    return;
}

/****************************************************************************/
/*                                                                          */
/*            INIT                                                          */
/***************************************************************************/
void init_pwm_gpio(int gpio_buzz)
{

  //INIT PIN PER IL BUZZER
  // Init GPIO pin for direction
  gpio_buzzer = new mraa::Gpio(gpio_buzz);
  if (gpio_buzzer == NULL) 
    ROS_ERROR("non sono riusciuto a configurare il pin");
  mraa::Result  response = gpio_buzzer->dir(mraa::DIR_OUT);
  ROS_INFO("PIN IN USCITA");
  gpio_buzzer->write(0);


}
/****************************************************************************/
/*                                                                          */
/*             BUZZER CALLBACK                                           */
/*                                                                          */
/***************************************************************************/
void buzzerCallback(const std_msgs::Int32::ConstPtr& msg)
{
  if(msg->data == 1)
  {
    //cmd
    tone_buzzer(0,100);
    usleep(300000);
    tone_buzzer(0,300);
    usleep(300000);
    tone_buzzer(0,500);
    usleep(300000);
    tone_buzzer(0,700);
  }
  if(msg->data == 2)
  {
    //gripper
    tone_buzzer(0,100);
    usleep(200000);
    tone_buzzer(0,200);
    usleep(200000);
    tone_buzzer(0,300);
  }
  if(msg->data == 3)
  {
    //param
    tone_buzzer(0,500);
    usleep(200000);
    tone_buzzer(0,500);
  }
   
  if(msg->data == 4)
  {
    //waypoint
    tone_buzzer(0,1000);
  }
   
  if(msg->data == 5)
  {
    //mode
    tone_buzzer(0,1000);
  }
   
  if(msg->data == 6)
  {
    //new waypoint
    tone_buzzer(0,1000);
  }
   

  if(msg->data == 7 )
  {
    //batteria
    tone_buzzer(0,1000);
    usleep(100000);
    tone_buzzer(0,1000);
    usleep(100000);
    tone_buzzer(0,1000);
    usleep(100000);
    tone_buzzer(0,1000);



  }
  if(msg->data == 8 )
  {
    //manca la posa da 0.3 secondi
    tone_buzzer(0,1000);
  }
  if(msg->data == 9 )
  {
    //manca la posa da 1 secondo
    tone_buzzer(0,500);
    usleep(100000);
    tone_buzzer(0,500);
    usleep(100000);
    tone_buzzer(0,500);
    usleep(100000);
    tone_buzzer(0,500);
    usleep(100000);
    tone_buzzer(0,500);
    usleep(100000);
    tone_buzzer(0,500);
    usleep(100000);
    tone_buzzer(0,500);
    usleep(100000);
    tone_buzzer(0,500);
    usleep(100000);
    tone_buzzer(0,500);

  }
  if(msg->data == 10 || msg->data == 11 )
  {
    //sta per atterarre o aprire la pinza
    tone_buzzer(0,100);
  }

  if(msg->data == 12)
    tone_buzzer(0,2500);


   return;
}
/****************************************************************************/
/*                                                                          */
/*             BUZZER FUNCTION                                            */
/*                                                                          */
/***************************************************************************/
void tone_buzzer(int frequency, int duration)
{

  
  gettimeofday(&duration_time, NULL);
  elapsed_time_note = 0;
  while(elapsed_time_note < duration)
  {

    gpio_buzzer->write(1);
    /*usleep(frequency);
    gpio_buzzer->write(0);
    usleep(frequency);*/

    gettimeofday(&current_time, NULL);
    //calcolo tempo di controllo
    elapsed_time_note = (current_time.tv_sec - duration_time.tv_sec) * 1000;
    elapsed_time_note += (current_time.tv_usec - duration_time.tv_usec) / 1000;
  }
  gpio_buzzer->write(0);


}