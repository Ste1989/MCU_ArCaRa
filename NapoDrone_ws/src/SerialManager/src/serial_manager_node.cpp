#include "serial_manager.h"

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
    req_topic = n.advertise<std_msgs::Int32>("napodrone/cmd_request", 1);
    param_topic = n.advertise<serial_manager::Param>("napodrone/param_request", 1);
    mode_topic = n.advertise<std_msgs::Int32>("napodrone/mode_request", 1);
    status_topic = n.subscribe<std_msgs::Int32>("napodrone/px4_status",10, &Status_Pixhawk_Callback);

    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/SerialManager/dev", seriale_dev, "/dev/ttyUSB0");


    int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());
    //inizializzo time_1
    gettimeofday(&time_1, NULL);
    //ros::Rate loop_rate(100); // 100 Hz
    if (result == 1)
    {
        while(ros::ok())
        {
            read_from_serial(&serial);

            //leggo il tempo e calcolo quanto è passato dall'ultimo pacchetto ricevuto
            gettimeofday(&time_2, NULL);
            elapsed_time = (time_2.tv_sec - time_1.tv_sec) * 1000;
            elapsed_time += (time_2.tv_usec - time_1.tv_usec) / 1000;
 /*           cout << elapsed_time << "ms.\n";

           if(elapsed_time > 2000)
            {
                //comunicazione persa
                coda_send_seriale.push('C');
                coda_send_seriale.push('O');
                coda_send_seriale.push('M');
                coda_send_seriale.push(' ');
                coda_send_seriale.push('L');
                coda_send_seriale.push('O');
                coda_send_seriale.push('S');
                coda_send_seriale.push('T');
                coda_send_seriale.push('.');
            }*/


            //controllo se vi è una richiesta di comando
            check_send_request();


            //vedi se arrivato qualcosa sulle callback
            ros::spinOnce();

            //funzione per scrivere su seriale
           write_to_serial(&serial);


            //loop_rate.sleep();
        }

    }


  return 0;
}