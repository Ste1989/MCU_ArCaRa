#include "buzzer_manager.h"


/***********************************************************************************************/
/*                                                                                             */
/*                      MAIN                                                                   */
/*                                                                                             */
/***********************************************************************************************/
int main(int argc, char **argv)
{

  ros::init(argc, argv, "buzzer_manager");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //leggi launch file
  int pwm_pin, dir_pin, buzzer_pin;
  n.param<int>("/buzzer_manager/buzzer_pin", buzzer_pin, 13);

  //topic subscribe
  buzzer_sub = n.subscribe("napodrone/buzzer", 1, buzzerCallback);
  

  /*********Init*******************************************************************************/
  signal(SIGINT, sig_handler);

  init_pwm_gpio(buzzer_pin);
  
  ros::Rate loop_rate(100); 


  /*******Ciclo***************************************************************************/
  while (ros::ok())
  {

    ros::spin();

    loop_rate.sleep();
  }
  
  return 0;
}