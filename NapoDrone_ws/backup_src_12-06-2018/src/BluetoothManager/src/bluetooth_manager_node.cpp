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
    pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/ekf_pose", 1000, ekf_pose_cb);
    ros::Rate loop_rate(freq_ros_node); 
    




    if(false)
    {
      xt = 1;
      yt = 2;
      zt = 3;
      xt_ = 0;
      yt_ = 0;
      zt_ = 0;
    //devo prendere la stima arrivata in precedenza e calcolare il treno di delta
    double delta_x = (xt - xt_)/num_campioni_delta;
    double delta_y = (yt - yt_)/num_campioni_delta;
    double delta_z = (zt - zt_)/num_campioni_delta;

    double x = xt_;
    double y = yt_;
    double z = zt_;

    for(int i = 0; i < num_campioni_delta-1; i++)
    {
        x = x + delta_x;
        y = y + delta_y;
        z = z + delta_z;

        coda_position_x.push(x);
        coda_position_y.push(y);
        coda_position_z.push(z);
    }
   
    //inserisco la misura vera
    coda_position_x.push(xt);
    coda_position_y.push(yt);
    coda_position_z.push(zt);
    }
    ros::Time filter_time = ros::Time::now();
    while(ros::ok())
    {
       // std:: cout <<"CIAO: "<< std::endl;
         /*********LEGGO SU SERIALE***********************************************************/
         
          int bytes = 0;
         
          unsigned char buf[1024];

          // Read data from the COM-port
          bytes= read(serial, buf, sizeof buf);
          if (bytes > 0)
          {
            for (int i = 0; i < bytes ;i++)
            {
              //std:: cout <<"RICEVUTO: "<<  (char)(buf[i]) << std::endl;




              if((char)(buf[i]) == 'd')
              {
                //ho una richiesta di inviare il valore che si trova in fondo alla coda
                
                write_to_serial(&serial);      
              }
              
            }
          }
          
          int n_size = coda_position_x.size();
          if(n_size > num_campioni_delta)
          {

            for(int i=0; i< num_campioni_delta ; i++)
            {
              coda_position_x.pop();
              coda_position_y.pop();
              coda_position_z.pop();
            }

          }

          
        
          ros::spinOnce();
          //loop_rate.sleep();
    }


  return 0;
}