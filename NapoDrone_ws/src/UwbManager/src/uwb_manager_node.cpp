#include "uwb_manager.h"

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
  ros::init(argc, argv, "UWB_Manager");

   /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;
    //publisher
    uwb_topic = n.advertise<uwb_manager::RangeUwb>("/uwb/range", 1);
    //client service
    get_time_sec0 = n.serviceClient<autopilot_manager::init_time>("/get_time_t0");
    
    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/UwbManager/dev", seriale_dev, "/dev/ttyACM0");
    n.param<bool>("/UwbManager/enable_log", enable_log, false);
    n.param<int>("/UwbManager/freq_ros_node", freq_ros_node, 50);

    
    //init variabili gloabli
    init_global_var();
    int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());

    //frequenza a cui far girare il nodo
    ros::Rate loop_rate(freq_ros_node);
    
    

    if (result == 1)
    {
        ROS_INFO("seriale uwb aperta");
        while(ros::ok())
        {
          gettimeofday(&current_time, NULL);
          /*LEGGO LA SERIALE*************************************************************************/  
          read_from_serial(&serial);
          
          //loop rate
          loop_rate.sleep();
        }

    }


  return 0;
}