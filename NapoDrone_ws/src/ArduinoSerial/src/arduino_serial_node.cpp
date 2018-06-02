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
    



    int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());
    init_global_var();
    //subscribe
    //pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/ekf_pose", 1000, ekf_pose_cb);
    ros::Rate loop_rate(freq_ros_node); 
    range_pub = n.advertise<sensor_msgs::Imu>("/pozyx_range", 1000);

    //service_pub = n.subscribe<std_msgs::Int16>("/service", 1, service_cb);
    service_calib_srv = n.advertiseService("service_calib", service_calib);
    ros::Time start_time = ros::Time::now();
    

    while(ros::ok())
    {
       
         /*********LEGGO SU SERIALE***********************************************************/

          // Read data from the COM-port
          read_from_serial(&serial);
          
          loop_rate.sleep();

          ros::spinOnce();
    }


  return 0;
}