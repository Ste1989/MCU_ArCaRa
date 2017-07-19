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
    uwb_topic = n.advertise<std_msgs::Int32>("napodrone/uwb", 1);
    
    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/UwbManager/dev", seriale_dev, "/dev/ttyACM0");
    
    
    int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());
    
    
    
    ROS_INFO("seriale uwb aperta");
    if (result == 1)
    {
        while(ros::ok())
        {
          gettimeofday(&current_time, NULL);
          /*LEGGO LA SERIALE*************************************************************************/  
          read_from_serial(&serial);
        
        }

    }


  return 0;
}