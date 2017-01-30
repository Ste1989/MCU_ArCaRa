
#include "autopilot_manager_node.h"


/********************************************************************************************/
/*                                                                                         */
/*    MAIN                                                                                 */
/*                                                                                         */
/*******************************************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pix_manager_node");
    ros::NodeHandle nh;

    //stato del drone
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    //leggo il barometro
    //pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("mavros/imu/atm_pressure", 1, pressure_cb);
    //comandi
    cmd_sub =  nh.subscribe<std_msgs::Int32>("/napodrone/cmd_request", 1, cmd_cb);
    //mode request
    mode_sub =  nh.subscribe<std_msgs::Int32>("/napodrone/mode_request", 1, mode_cb);
    //param request
    param_sub =  nh.subscribe<serial_manager::Param>("/napodrone/param_request", 1, param_cb);
    //waypoint
    waypoint_sub =  nh.subscribe<geometry_msgs::Pose>("/napodrone/waypoint", 1, waypoint_cb);
    //aruco poses
    aruco_poses_sub =  nh.subscribe<geometry_msgs::PoseStamped>("/napodrone_pose", 1, poses_cb);
    //topic per override la radio
    rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1);
    //topic per scrivere lo stato
    state_pub = nh.advertise<std_msgs::Int32>("napodrone/px4_status", 10);
    

    //service per armare il drone
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //service per cambiare il modo del drone
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    //service per avere lo strem di dati dal drone
    set_stream_rate_client = nh.serviceClient<mavros_msgs::StreamRate>("mavros/set_stream_rate");


    //leggo i parametri specificati nel launch file
    nh.param<int>("/AutopilotManager/loop_rate", loop_rate, 20);
    nh.param<int>("/AutopilotManager/stream_rate", stream_rate, 50);
    nh.param<double>("/AutopilotManager/alt_takeoff", alt_takeoff_target, -1.0);
    nh.param<double>("/AutopilotManager/alt_partenza", alt_takeoff_partenza, -0.3);
    ROS_INFO("ALTEZZA DI TAKEOFF: %f", alt_takeoff_target);
    ROS_INFO("ALTEZZA DI TAKEOFF: %f", alt_takeoff_partenza);
    
    //PID file
    nh.param<std::string>("/AutopilotManager/pid_file", PID_file, "");


    //////////////////////////////////////LEGGI FILE PID////////////////////////////////////////////
    if(PID_file == "empty")
        ROS_WARN("PID filename empty! Check the launch file paths");
    else
    {
        ROS_INFO_STREAM("PID file path: " << PID_file );
        leggi_PID_file(PID_file);
    }
    
    //INIT//////////////////////////////////////////////////////////////////////////////////////////
    init_global_variables();
    gettimeofday(&control_time, NULL);
    waypoint_recv = 0;
    /********************************************************************************************/
    /*                                                                                         */
    /*                     CICLO PRINCIPALE CONTROLLO                                          */
    /*                                                                                         */
    /*******************************************************************************************/
    while(ros::ok())
    {
        //A)se non sono in manuale
        if(waypoint_recv && !manual_mode)
        {
            if(waypoint_recv == 1)
            {   
                ROS_INFO("DRONE IN AUTOMATICO: NUOVO WAYPOINT");
                //se è un nuovo waypoint rinizializzo i controllori
                pid_controllers.roll.init_PID();
                pid_controllers.pitch.init_PID();
                pid_controllers.yaw.init_PID();
                pid_controllers.altitude.init_PID();
         

                waypoint_recv = 2;
            }

            //calolo tempo attuale
            gettimeofday(&current_time, NULL);
            //calcolo tempo di controllo
            elapsed_time_control = (current_time.tv_sec - control_time.tv_sec) * 1000;
            elapsed_time_control += (current_time.tv_usec - control_time.tv_usec) / 1000;
            //calcolo tempo dall'ultima posa ricevuta
            elapsed_time_pose = (current_time.tv_sec - pose_time.tv_sec) * 1000;
            elapsed_time_pose += (current_time.tv_usec - pose_time.tv_usec) / 1000;
            //calolo tempo passato dalla ichiesta di takeoff
            elapsed_time_takeoff = (current_time.tv_sec - takeoff_time.tv_sec) * 1000;
            elapsed_time_takeoff += (current_time.tv_usec - takeoff_time.tv_usec) / 1000;

            //GESTIONE DEL TAKEOFF===============================================================
            //se è passato più di un secondo dalla richiesta di takeoff resetto il bit di takeoff
            if (elapsed_time_takeoff > 1000)
                init_takeoff = false;

            if(stato_takeoff == 1)
            {
                //devo controllare che l'altezza raggiunta sia circa quella impostata
                //se si drone decollato
                if(abs(P_world_body_world.position.z - current_waypoint_world.position.z) < 0.1)
                {
                    stato_takeoff = 2;
                    //scrivo su topic che sono connesso
                    std_msgs::Int32 msg;
                    msg.data = HOVER;
                    state_pub.publish(msg);
                }

            }

            //se non ho una posa nuova ma sono in fase di takeoff 
            if(!new_pose_recv  && init_takeoff)
            {
                ROS_INFO("NON HO LA POSA MA INGANNO COME SE CE L AVESSE");
                //inganno il controllo che una nuova posa 
                new_pose_recv = 1;
                //metto la quota di parteza
                P_world_body_world.position.z = alt_takeoff_partenza;
                P_world_body_world.orientation.z = 0;

            }else
            {
                //dovrebbe entrare dopo nel caso che non ha più stima da un seconod della posa
            }


          
            if(new_pose_recv && elapsed_time_control  >= (1000/loop_rate))
            {
                ROS_INFO("CICLO CONTROLLO");
                cout << "FREQUENZA CONTROLLO: " << 1000/elapsed_time_control <<endl;
                update_control();
                new_pose_recv = 0;
                gettimeofday(&control_time, NULL);
            }
            else
            {
                //devo controllare che non sia passato più di un secondo dall'ultima posa ricevuta e non sono in fase di decollo
                if(elapsed_time_pose > 1000/1 && !init_takeoff)
                {
                    ROS_WARN("NON HO STIMA DELLA POSIZIONE DA 1 SECONDO");
                    //imposto i valori di pwm di yaw, pitch e roll al minimo
                    //todo: per l'altezza?
                    double pwm_throttle = 1500;
                    //warning_stop(pwm_throttle);
                    clear_radio_override();
                    
                }
                else
                {
                    //do nothing
                }

            }
            /************************************************************************************************/
            
        }
        else
        {
            //il drone è comandato via radio: do nothing
        }
        
        //B)Controllo se vi sono delle richieste di comando
        void check_request();


        //C)leggo i topic
        ros::spinOnce();
        

    }


    return 0;
}

