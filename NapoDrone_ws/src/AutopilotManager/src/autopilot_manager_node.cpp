
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
     //leggo coordinate carico scarico centro
    double x_carico,y_carico,z_carico,x_centro,y_centro,z_centro,x_scarico,y_scarico,z_scarico;
    nh.param<double>("/AutopilotManager/x_carico", x_carico, 0);
    nh.param<double>("/AutopilotManager/y_carico", y_carico, 0);
    nh.param<double>("/AutopilotManager/z_carico", z_carico, -1);
    nh.param<double>("/AutopilotManager/x_centro", x_centro, 0);
    nh.param<double>("/AutopilotManager/y_centro", y_centro, 0);
    nh.param<double>("/AutopilotManager/z_centro", z_centro, -1);
    nh.param<double>("/AutopilotManager/x_scarico", x_scarico, 0);
    nh.param<double>("/AutopilotManager/y_scarico", y_scarico, 0);
    nh.param<double>("/AutopilotManager/z_scarico", z_scarico, -1);
    //PID file
    nh.param<std::string>("/AutopilotManager/pid_file", PID_file, "");

    ROS_INFO("ALTEZZA DI TAKEOFF: %f", alt_takeoff_target);
    ROS_INFO("ALTEZZA DI TAKEOFF: %f", alt_takeoff_partenza);
    ROS_INFO("x_carico: %f ", x_carico);
    ROS_INFO("y_carico: %f ", y_carico);
    ROS_INFO("z_carico: %f ", z_carico);
    ROS_INFO("x_centro: %f ", x_centro);
    ROS_INFO("y_centro: %f ", y_centro);
    ROS_INFO("z_centro: %f ", z_centro);
    ROS_INFO("x_scarico: %f ", x_scarico);
    ROS_INFO("y_scarico: %f ", y_scarico);
    ROS_INFO("z_scarico: %f ", z_scarico);

    waypoint_carico.position.x = x_carico;
    waypoint_carico.position.y = y_carico;
    waypoint_carico.position.z = z_carico;
    waypoint_carico.orientation.z = 0;

    waypoint_centro.position.x = x_centro;
    waypoint_centro.position.y = y_centro;
    waypoint_centro.position.z = z_centro;
    waypoint_centro.orientation.z = 0;

    waypoint_scarico.position.x = x_scarico;
    waypoint_scarico.position.y = y_scarico;
    waypoint_scarico.position.z = z_scarico;
    waypoint_scarico.orientation.z = 0;
    
    


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
        //calolo tempo passato dalla richiesta di land
        elapsed_time_land = (current_time.tv_sec - land_time.tv_sec) * 1000;
        elapsed_time_land += (current_time.tv_usec - land_time.tv_usec) / 1000;
        //calolo tempo passato dalla richiesta di land
        elapsed_time_land_over = (current_time.tv_sec - land_over_time.tv_sec) * 1000;
        elapsed_time_land_over += (current_time.tv_usec - land_over_time.tv_usec) / 1000;
        
        //A)se non sono in manuale
        if(!manual_mode)
        {
            //il drone può trovarsi in 5 stati codificati nella viariabile drone_state:
            //1-landed
            //2-takeoff
            //3-hold position
            //4-go to waypoint
            //5-land
            //a seconda di dove si trova faccio un azione oppure un altra
            switch(drone_state)
            {
                case LANDED_STATE:
                    //da qui posso solo decollare
                    init_takeoff = false;
                    break;
                case LANDING_STATE:
                    //qua devo controllare di essere arrivato a terra e disarmare
                    if(abs(P_world_body_world.position.z - alt_takeoff_partenza) < 0.05)
                    {
                        drone_state = LANDED_STATE;
                    }
                    break;
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                case TAKEOFF_STATE:
                    //qui devo gestire il decollo
                    //devo controllare che l'altezza raggiunta sia circa quella impostata
                    //se si drone è decollato-->devo andare al centro
                    if(abs(P_world_body_world.position.z -  current_waypoint_world.position.z ) <  0.15 && elapsed_time_takeoff > 0.5)
                    {
                        //il drone è decollato, devo adesso andare al centro, ma nel fratempo matengo la posizione sopra
                        current_waypoint_world.position.x = P_world_body_world.position.x;
                        current_waypoint_world.position.y = P_world_body_world.position.y;
                        current_waypoint_world.position.z = 1; 
                        current_waypoint_world.orientation.z = 0;
                        pid_controllers.roll.init_PID();
                        pid_controllers.pitch.init_PID();
                        pid_controllers.altitude.init_PID();
                        //init_takeoff = false;
                        ROS_INFO("DECOLLATO");
                        ROS_INFO("VADO AL CENTRO IN HOVER");
                        //gli dico di andare al centro
                        waypoint_world_GOAL = waypoint_centro;
                        drone_state = GOTO_STATE;
                        
                    }
                    else
                    {
                        //se è passato più di 1 secondo dalla richiesta di takeoof e non ho ancora una stima di poszione
                        if ( elapsed_time_pose > 1000/1)
                        {
                            ROS_WARN("NON HO STIMA DELLA POSIZIONE DA 1 SECONDO");
                            //imposto i valori di pwm di yaw, pitch e roll al minimo
                            //todo: per l'altezza?
                            double pwm_throttle = PWM_MEDIUM_THROTTLE;
                            warning_stop(pwm_throttle);
                        }
                        //if (elapsed_time_takeoff <= 1000 && elapsed_time_pose > 1000/1)
                           // P_world_body_world.position.z = alt_takeoff_partenza;
                        //se è passato più di 5 secondi e il drone è ancora in takeoof_state significa che qualcosa non è anadato bene
                        //scelgo di atterrare
                        //if (elapsed_time_takeoff > 5000)
                            //..land
                    }
                    break;
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                case HOLD_POSITION_STATE:


                    //qua devo controllare il caso in cui sono andato troppo lontano dal punto in cui devo stare fermo ci torno
                    //con una traiettoria
                    if(abs(P_world_body_world.position.x - current_waypoint_world.position.x ) > 0.5 && abs(P_world_body_world.position.y-current_waypoint_world.position.y ) > 0.5)
                    {
                        //sono troppo lontano dal punto in cui devo stare fermo:
                        //imposto il waypoint nel punto in cui si trova il drone e il suo goal la poszione che aveva
                        waypoint_world_GOAL = current_waypoint_world;
                        //imposto il suo waypoint attuale
                        current_waypoint_world.position.x = P_world_body_world.position.x;
                        current_waypoint_world.position.y = P_world_body_world.position.y;
                        current_waypoint_world.position.z = 1; 
                        current_waypoint_world.orientation.z = 0;
                        pid_controllers.roll.init_PID();
                        pid_controllers.pitch.init_PID();
                        pid_controllers.altitude.init_PID();
                        drone_state = GOTO_STATE;

                    }
                    //controllo se ho una richiesta di atterraggio nel punto, se si e sono abbastanza vicino al punto di atterraggio inizio l'atterragio
                    if(abs(P_world_body_world.position.x - current_waypoint_world.position.x ) < 0.15 && abs(P_world_body_world.position.y-current_waypoint_world.position.y ) < 0.15 && land_req)
                    {
                        //devo atterrarre nel punto
                        gettimeofday(&land_time, NULL);
                        drone_state = LAND_STATE;
                    }

                    //qui devo mantenere la posizione, potrei voler andare in un altro punto o atterrare
                    //controllo che abbia la posizione stimata
                    if(elapsed_time_pose > 1000/1)
                    {
                        ROS_WARN("NON HO STIMA DELLA POSIZIONE DA 1 SECONDO");
                        //imposto i valori di pwm di yaw, pitch e roll al minimo
                        //todo: per l'altezza?
                        double pwm_throttle = PWM_MEDIUM_THROTTLE;
                        warning_stop(pwm_throttle);
                        //clear_radio_override();               
                    }           
                    break;
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
                case GOTO_STATE:
                    //qui devo gestire l'andare in una posizione
                    if(abs(P_world_body_world.position.x - waypoint_world_GOAL.position.x ) > 0.3 || abs(P_world_body_world.position.y-waypoint_world_GOAL.position.y ) > 0.3)
                    {
                        /*cout << waypoint_world_GOAL.position.x << endl;
                        cout << waypoint_world_GOAL.position.y << endl;*/

                        if(abs(P_world_body_world.position.x - current_waypoint_world.position.x ) < 0.1 && abs(P_world_body_world.position.y-current_waypoint_world.position.y ) < 0.1)
                        {
                            //calcolo angolo tra la mia posizione e il goal
                            double gx = waypoint_world_GOAL.position.x;
                            double gy = waypoint_world_GOAL.position.y;
                            double px = P_world_body_world.position.x;
                            double py = P_world_body_world.position.y;
                            double alfa = atan2((gx-px),(gy-py));
                            double intorno_max = 0.3;
                            double delta_y = intorno_max * cos(alfa);
                            double delta_x = intorno_max * sin(alfa);
                            //genero nuovo waypoint
                            ROS_INFO("GENERO NUOVO WAYPOINT");
                            current_waypoint_world.position.x = px + delta_x;
                            current_waypoint_world.position.y = py + delta_y;
                            //se il nuovo waypoint generato nrlla coord y non è nella fascia
                            if(abs(current_waypoint_world.position.y - gy) > 0.2)
                            {
                                if((current_waypoint_world.position.y - gy) >0)
                                    current_waypoint_world.position.y = gy + 0.2;
                                else
                                    current_waypoint_world.position.y = gy - 0.2;
                            }
                            current_waypoint_world.position.z = -1;
                            current_waypoint_world.orientation.z = 0.0;
                            ROS_INFO("X: %f",current_waypoint_world.position.x );
                            ROS_INFO("Y: %f",current_waypoint_world.position.y );
                            //inizializzo PID
                            //inizializzo i controllori
                            pid_controllers.roll.init_PID();
                            pid_controllers.pitch.init_PID();
                            //pid_controllers.yaw.init_PID();
                            //pid_controllers.altitude.init_PID();

                        }
                        else
                        {
                            //..sto andando nel punto
                        }

                    }
                    else
                    {
 
                        //sono sul punto: devo vedere se quello finale allora vado in hold position state
                        ROS_INFO("VADO AL PUNTO DI ARRIVO");
                        current_waypoint_world = waypoint_world_GOAL;
                        cout << waypoint_world_GOAL.position.x << endl;
                        cout << waypoint_world_GOAL.position.y << endl;
                        pid_controllers.roll.init_PID();
                        pid_controllers.pitch.init_PID();
                        pid_controllers.altitude.init_PID();
                        drone_state = HOLD_POSITION_STATE;
    
                    }

                    //controllo che abbia la posizione stimata
                    if(elapsed_time_pose > 1000/1)
                    {
                        ROS_WARN("NON HO STIMA DELLA POSIZIONE DA 1 SECONDO");
                        //imposto i valori di pwm di yaw, pitch e roll al minimo
                        //todo: per l'altezza?
                        double pwm_throttle = PWM_MEDIUM_THROTTLE;
                        warning_stop(pwm_throttle);
                        //clear_radio_override();               
                    }
                    //controllo se ho raggiunto la posizione
                    break;
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
                case LAND_STATE:
                    //qui devo gestire l'atterraggio
                    //devo variare l altrezza di atterraggio

                    if(land_req == 1){
                        current_waypoint_world.position.z = -0.9;
                        pid_controllers.altitude.init_PID();
                        ROS_INFO("nuova quota %f", current_waypoint_world.position.z);
                        land_req = 2;
                    }

                    if(elapsed_time_land > 1000  && land_req == 2){
                        current_waypoint_world.position.z = -0.8;
                        pid_controllers.altitude.init_PID();
                        ROS_INFO("nuova quota %f", current_waypoint_world.position.z);
                        land_req = 3;}

                    if(elapsed_time_land > 1000*3 && land_req ==3)
                    {
                        current_waypoint_world.position.z = -0.75;
                        pid_controllers.altitude.init_PID();
                        ROS_INFO("nuova quota %f", current_waypoint_world.position.z);
                        gettimeofday(&land_over_time, NULL);
                        elapsed_time_land_over = 0;
                        land_req = 4;
                    }
                    if(land_req == 4)
                    {
                        if(abs(P_world_body_world.position.x - current_waypoint_world.position.x ) < 0.1 && abs(P_world_body_world.position.y-current_waypoint_world.position.y ) < 0.1)
                            if(elapsed_time_land_over >= 0)
                             {   
                                drone_state = LANDING_STATE;
                                ROS_INFO("atterrato");
                             }
                            else
                                ROS_INFO("aspetto");
                        else{
                            //resetto contatore tempo
                            gettimeofday(&land_over_time, NULL);
                            ROS_INFO("troppo lontano");
                            }
                        cout << "X: "<<P_world_body_world.position.x - current_waypoint_world.position.x << endl;
                        cout << "Y: "<<P_world_body_world.position.y - current_waypoint_world.position.y << endl;
                    }
                    break;
                    
            }
            
            //se ho una nuov posa e sono passati 1000/loop_rate ms chiamo update_control 
            if(new_pose_recv && elapsed_time_control  >= (1000/loop_rate) && elapsed_time_pose <= 1000/1)
            {
                //ROS_INFO("CICLO CONTROLLO");
                //cout << "FREQUENZA CONTROLLO: " << 1000/elapsed_time_control <<endl;
                update_control();
                //resetto new_pose_recv
                new_pose_recv = 0;
                gettimeofday(&control_time, NULL);
            }else
            {
                if(elapsed_time_control  >= (1000/loop_rate) && elapsed_time_takeoff <= 1000/1)
                {
                    //ROS_INFO("CICLO CONTROLLO");
                    //cout << "FREQUENZA CONTROLLO: " << 1000/elapsed_time_control <<endl;
                    update_control();
                    gettimeofday(&control_time, NULL);
                }

            }
            
        }
        else
        {
            //il drone è comandato via radio: do nothing
        }
        
        //B)Controllo se vi sono delle richieste di comando
        check_request();


        //C)leggo i topic
        ros::spinOnce();
        

    }


    return 0;
}

