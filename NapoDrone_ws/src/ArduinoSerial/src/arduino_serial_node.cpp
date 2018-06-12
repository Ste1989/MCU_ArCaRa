#include "arduino_serial.h"

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
  ros::init(argc, argv, "Arduino Serial");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;
    


  

    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/ArduinoSerial/dev", seriale_dev, "/dev/ttyUSB0");
    n.param<int>("/ArduinoSerial/freq_ros_node", freq_ros_node, 200);
    bool tag1,tag2,tag3,tag4,tag5,tag6,tag7,tag8;
    n.param<bool>("/ArduinoSerial/tag1", tag1, false);
    n.param<bool>("/ArduinoSerial/tag2", tag2, false);
    n.param<bool>("/ArduinoSerial/tag3", tag3, false);
    n.param<bool>("/ArduinoSerial/tag4", tag4, false);
    n.param<bool>("/ArduinoSerial/tag5", tag5, false);
    n.param<bool>("/ArduinoSerial/tag6", tag6, false);
    n.param<bool>("/ArduinoSerial/tag7", tag7, false);
    n.param<bool>("/ArduinoSerial/tag8", tag8, false);
    



    int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());
    init_global_var();
    //subscribe
    //pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/ekf_pose", 1000, ekf_pose_cb);
    ros::Rate loop_rate(freq_ros_node); 
  
    range1_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range_1", 1000);
    range2_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range_2", 1000);
    range3_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range_3", 1000);
    range4_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range_4", 1000);
    range5_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range_5", 1000);
    range6_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range_6", 1000);
    range7_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range_7", 1000);
    range8_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range_8", 1000);
    
    anchor_range_pub = n.advertise<geometry_msgs::Pose>("/anchor_range",1000);
    //service_pub = n.subscribe<std_msgs::Int16>("/service", 1, service_cb);
    service_calib_srv = n.advertiseService("service_calib", service_calib);
    service_start_srv = n.advertiseService("service_start", service_start);

    ros::Time start_time = ros::Time::now();
    //legenda
    //richiesta = 5 T1ON
    //richiesta = 6 T1OFF
    //richiesta = 7 T2ON
    //richiesta = 8 T2OFF
    //richiesta = 9 T3ON
    //richiesta = 10 T3OFF
    //richiesta = 11 T4ON
    //richiesta = 12 T4OFF
    //richiesta = 13 T5ON
    //richiesta = 14 T5OFF
    //richiesta = 15 T6ON
    //richiesta = 16 T6OFF
    //richiesta = 17 T7ON
    //richiesta = 18 T7OFF
    //richiesta = 19 T8ON
    //richiesta = 20 T8OFF
    if(tag1)
      write_to_serial(&serial, 5);
    else
      write_to_serial(&serial, 6);
     if(tag2)
      write_to_serial(&serial, 7);
    else
      write_to_serial(&serial, 8);
     if(tag3)
      write_to_serial(&serial, 9);
    else
      write_to_serial(&serial, 10);
     if(tag4)
      write_to_serial(&serial, 11);
    else
      write_to_serial(&serial, 12);
     if(tag5)
      write_to_serial(&serial, 13);
    else
      write_to_serial(&serial, 14);
     if(tag6)
      write_to_serial(&serial, 15);
    else
      write_to_serial(&serial, 16);
     if(tag7)
      write_to_serial(&serial, 17);
    else
      write_to_serial(&serial, 18);
     if(tag8)
      write_to_serial(&serial, 19);
    else
      write_to_serial(&serial, 20);


    while(ros::ok())
    {
       
         /*********LEGGO SU SERIALE***********************************************************/

          // Read data from the COM-port
          read_from_serial(&serial);

          //invio su seriale
          if(servizio_richiesto > 0)
          {
            write_to_serial(&serial, servizio_richiesto);
            servizio_richiesto = 0;
          }
          
          loop_rate.sleep();

          ros::spinOnce();
    }


  return 0;
}