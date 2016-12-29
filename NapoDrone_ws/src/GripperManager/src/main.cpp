#include "gripper_manager.h"


/***********************************************************************************************/
/*                                                                                             */
/*                      MAIN                                                                   */
/*                                                                                             */
/***********************************************************************************************/
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
  int pwm_pin, dir_pin;
  n.param<int>("/gripper_manager/pwm_pin", pwm_pin, 32);
  n.param<int>("/gripper_manager/dir_pin", dir_pin, 37);
  n.param<double>("/gripper_manager/time_grip", time_open2close, 2.5);

  //topic subscribe
  gripper_sub = n.subscribe("napodrone/gripper_request", 1, cmdgripperCallback);
  gripper_pub = n.advertise<std_msgs::Int32>("napodrone/gripper_status", 1);

  /*********Init*******************************************************************************/
  signal(SIGINT, sig_handler);

  init_pwm_gpio(pwm_pin, dir_pin);
  

  ros::Rate loop_rate(100); 
  //apro la pinza e notifico che è aperta
  //ROS_INFO("APRO LA PINZA");
  //cmd_gripper(0, 1);
  //std_msgs::Int32 msg;
  //msg.data = 0;
  //gripper_pub.publish(msg);
  /*******Ciclo***************************************************************************/
  while (ros::ok())
  {

    ros::spin();

    loop_rate.sleep();
  }
  
  return 0;
}