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
    gripper_topic = n.advertise<std_msgs::Int32>("napodrone/gripper_request", 1);
    param_topic = n.advertise<serial_manager::Param>("napodrone/param_request", 1);
    mode_topic = n.advertise<std_msgs::Int32>("napodrone/mode_request", 1);
    waypoint_topic = n.advertise<geometry_msgs::Pose>("napodrone/waypoint", 1);

    status_topic = n.subscribe<std_msgs::Int32>("napodrone/px4_status",10, &Status_Pixhawk_Callback);
    pose_topic = n.subscribe<geometry_msgs::PoseStamped>("/napodrone_pose", 1, &Pose_cb);

    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/SerialManager/dev", seriale_dev, "/dev/ttyUSB0");
    n.param<bool>("/SerialManager/stream_pose", stream_pose, false);
    n.param<int>("/SerialManager/ack_el_time", ack_el_time, 250);
    n.param<int>("/SerialManager/pose_el_time", pose_el_time, 500);

    int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());
    //inizializzo new_pkt_time
    gettimeofday(&new_pkt_time, NULL);
    gettimeofday(&ping_time, NULL);
    gettimeofday(&stream_pose_time, NULL);
    //ros::Rate loop_rate(100); // 100 Hz
    if (result == 1)
    {
        while(ros::ok())
        {

          /*LEGGO LA SERIALE*************************************************************************/  
          read_from_serial(&serial);

          /*CALCOLO TEMPI*******************************************************************************/
          //leggo il tempo e calcolo quanto è passato dall'ultimo pacchetto ricevuto
          gettimeofday(&current_time, NULL);
          elapsed_time_pkt_received = (current_time.tv_sec - new_pkt_time.tv_sec) * 1000;
          elapsed_time_pkt_received += (current_time.tv_usec - new_pkt_time.tv_usec) / 1000;
          //calcolo tempo per inviare un ping
          elapsed_time_ping = (current_time.tv_sec - ping_time.tv_sec) * 1000;
          elapsed_time_ping += (current_time.tv_usec - ping_time.tv_usec) / 1000;
          //calcolo tempo per invio di una misura di poszione
          elapsed_time_pose = (current_time.tv_sec - stream_pose_time.tv_sec) * 1000;
          elapsed_time_pose += (current_time.tv_usec - stream_pose_time.tv_usec) / 1000;
          
          /********INVIO UN PING******************************************************************************/
          if(elapsed_time_ping > ack_el_time)
          {

            coda_send_seriale.push(HEADER_A);
            coda_send_seriale.push(HEADER_B);
            coda_send_seriale.push(PAYLOAD_PING);
            coda_send_seriale.push(PAYLOAD_ACK); 

            //aggiorno ping_time
            gettimeofday(&ping_time, NULL);
          }
          /*************************INVIO POSE*********************************************************************/
          if(stream_pose && elapsed_time_pose > pose_el_time && new_packet_pose)
          {

            //devo inviare la posizione letta
            coda_send_seriale.push(HEADER_A);
            coda_send_seriale.push(HEADER_B);
            coda_send_seriale.push(PAYLOAD_POSE);
            encode_payload(P_world_body_world.position.x );
            encode_payload(P_world_body_world.position.y );
            encode_payload(P_world_body_world.position.z );
            encode_payload(P_world_body_world.orientation.z );

            new_packet_pose = 0;


          }

 /*         
 /*        cout << elapsed_time_pkt_received << "ms.\n";

           if(elapsed_time_pkt_received > 2000)
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

          /*********CONTROLLO RICHIESTE DI COMANDI***********************************************************/
          //controllo se vi è una richiesta di comando
          check_send_request();

          /*********LEGGO LE CALLBACK***********************************************************/
          //vedi se arrivato qualcosa sulle callback
          ros::spinOnce();

          /*********INVIO SU SERIALE***********************************************************/
          //funzione per scrivere su seriale
          write_to_serial(&serial);


            //loop_rate.sleep();
        }

    }


  return 0;
}