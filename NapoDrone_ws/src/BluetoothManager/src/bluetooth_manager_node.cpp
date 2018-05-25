#include "bluetooth_manager.h"

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
  ros::init(argc, argv, "Serial_Manager");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;
    


  

    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/BluetoothManager/dev", seriale_dev, "/dev/ttyUSB0");
    n.param<int>("/BluetoothManager/freq_ros_node", freq_ros_node, 50);
    n.param<int>("/BluetoothManager/num_campioni_delta", num_campioni_delta, 1);

    /*coda_position_x.push(1);
    coda_position_x.push(2);
    coda_position_x.push(3);
    coda_position_x.push(4);

    std::cout << coda_position_x.front() << std::endl;
    coda_position_x.pop();
    std::cout << coda_position_x.front() << std::endl;
    coda_position_x.pop();
    std::cout << coda_position_x.front() << std::endl;
    coda_position_x.pop();
    std::cout << coda_position_x.front() << std::endl;
    coda_position_x.pop();*/
    

    int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());
    init_global_var();
    //subscribe
    pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/ekf_pose", 1, ekf_pose_cb);
    ros::Rate loop_rate(freq_ros_node); 
    

    
    
    while(ros::ok())
    {

         /*********LEGGO SU SERIALE***********************************************************/
         
          int bytes = 0;
         
          unsigned char buf[1024];

          // Read data from the COM-port
          bytes= read(serial, buf, sizeof buf);
          if (bytes > 0)
          {
            for (int i = 0; i < bytes ;i++)
            {
              std:: cout <<"RICEVUTO: "<<  (char)(buf[i]) << std::endl;

              if((char)(buf[i]) == 'd')
              {
                //ho una richiesta di inviare il valore che si trova in fondo alla coda
                
                write_to_serial(&serial);      
              }
              
            }
          }
          


          loop_rate.sleep();
        

    }


  return 0;
}