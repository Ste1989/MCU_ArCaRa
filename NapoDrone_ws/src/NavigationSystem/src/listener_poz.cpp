//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================

#include "ros/ros.h"
#include <errno.h>
#include <unistd.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <queue>
#include <sys/time.h>
#include <geometry_msgs/Pose.h>






/********************************************************************************************/
/*                                                                                         */
/*    QUATERNION_2_EULER                                                                    */
/*                                                                                         */
/*******************************************************************************************/
void quaternion_2_euler(double xquat, double yquat, double zquat, double wquat, double& roll, double& pitch, double& yaw)
{
  //Trasformo le corrdinate SVO in coordinate frame camera.
  double r11 = wquat*wquat + xquat*xquat - yquat*yquat - zquat*zquat;
  double r12 = 2*(xquat*yquat - wquat*zquat);
  double r13 = 2*(zquat*xquat + wquat*yquat);
  double r21 =  2*(xquat*yquat + wquat*zquat);
  double r22 = wquat*wquat - xquat*xquat + yquat*yquat - zquat*zquat;
  double r23 = 2*(yquat*zquat - wquat*xquat);
  double r31 = 2*(zquat*xquat - wquat*yquat);
  double r32 = 2*(yquat*zquat + wquat*xquat);
  double r33 = wquat*wquat - xquat*xquat - yquat*yquat + zquat*zquat;
  //Scrivo la trasposta:
  double rt11,rt12,rt13,rt21,rt22,rt23,rt31,rt32,rt33;
  rt11 = r11;
  rt12 = r21;
  rt13 = r31;
  rt21 = r12;
  rt22 = r22;
  rt23 = r32;
  rt31 = r13;
  rt32 = r23;
  rt33 = r33;
  //calcolo angoli di eulero
  roll = atan2(rt23,rt33);
  pitch = -asin(rt13);
  yaw = atan2(rt12,rt11);
}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK STIMA DI POSA                                                            */
/*                                                                                         */
/*******************************************************************************************/
void poses_cb(const geometry_msgs::Pose::ConstPtr& msg)
{



  double rool, pitch, yaw;
  quaternion_2_euler(msg->orientation.x,msg->orientation.y,
    msg->orientation.z,msg->orientation.w, 
    rool, pitch, yaw);
  std::cout << "ROOL: " << rool*180/3.14 << "PITCH: " << pitch*180/3.14 << "YAW: " << yaw*180/3.14 << std::endl;
  //ho una stima buona della posizione della camera
  FILE* fd1;
  fd1 = fopen("/home/robot/MCU_ArCaRa/NapoDrone_ws/log/pozyx.txt","a");
  /*fprintf(fd1, "%i", msg->stamp.sec);
  fprintf(fd1, "%s", "  ");
  fprintf(fd1, "%i", msg->stamp.nsec);
  fprintf(fd1, "%s", "  ");*/
  fprintf(fd1, "%f", msg->position.x);
  fprintf(fd1, "%s", "  ");
  fprintf(fd1, "%f", msg->position.y);
  fprintf(fd1, "%s", "  ");
  fprintf(fd1, "%f", msg->position.z);
  fprintf(fd1, "%s", "  ");
  fprintf(fd1, "%f", rool);
  fprintf(fd1, "%s", "  ");
  fprintf(fd1, "%f", pitch);
  fprintf(fd1, "%s", "  ");
  fprintf(fd1, "%f\n", yaw);
  fclose(fd1);



}

/*****************************************************************/
/*                                                               */
/*                 MAIN                                          */
/*****************************************************************/
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
  */
    ros::init(argc, argv, "listener_poz");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;
    //ros::Subscriber imu_topic = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
    ros::Subscriber aruco_pose_topic = n.subscribe<geometry_msgs::Pose>("/pozyx_pose", 100, poses_cb);




    while(ros::ok())
   {

        //vedi se arrivato qualcosa sulle callback 
        ros::spin();
    }

  return 0;
}
