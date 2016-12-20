
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
    pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("mavros/imu/atm_pressure", 1, pressure_cb);
    //comandi
    cmd_sub =  nh.subscribe<std_msgs::Int32>("/napodrone/cmd_request", 1, cmd_cb);
    //mode request
    mode_sub =  nh.subscribe<std_msgs::Int32>("/napodrone/mode_request", 1, mode_cb);
    //param request
    param_sub =  nh.subscribe<serial_manager::Param>("/napodrone/param_request", 1, param_cb);
    //waypoint
    waypoint_sub =  nh.subscribe<geometry_msgs::Pose>("/napodrone/waypoint", 1, waypoint_cb);
    //aruco poses
    aruco_poses_sub =  nh.subscribe<aruco_mapping::ArucoMarker>("/aruco_poses", 1, poses_cb);
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
    nh.param<int>("/AutopilotManager/loop_rate", loop_rate, 30);
    nh.param<int>("/AutopilotManager/stream_rate", stream_rate, 50);
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
    //imposto la frequenza del nodo 
    //ros::Rate rate(loop_rate);
   
    // wait for FCU connection
    /*while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }*/
    //scrivo su topic che sono connesso
    std_msgs::Int32 msg;
    msg.data = CONNECTED;
    state_pub.publish(msg);
    ROS_INFO("SONO CONNESSO ALL'AUTOPILOTA");

    /*
    //una volta connesso avvio lo stream dati dal drone
    mavros_msgs::StreamRate srv_rate;
    srv_rate.request.stream_id = 0;
    srv_rate.request.message_rate = stream_rate;
    srv_rate.request.on_off = 1;

    set_stream_rate_client.call(srv_rate);
    ROS_INFO("STREAM DATI AVVIATO");
    */
    bool res = false;
    gettimeofday(&control_time, NULL);
    waypoint_recv = 0;
    /********************ciclo principale*************************************************************************************/
    while(ros::ok())
    {
        //GESTIONE DELLA RICHIESTA DI COMANDO
        
        if(current_cmd_req != NO_REQ)
        {
            switch(current_cmd_req)
            {
            
                case ARM:
                    ROS_INFO("COMANDO : ARMA");
                    res = arm_vehicle();
                    if(res)
                    {   
                        ROS_INFO("ARMATO");
                        current_cmd_req = NO_REQ;
                        std_msgs::Int32 msg;
                        msg.data = ARMED;
                        state_pub.publish(msg);
                    }
                    else
                        current_cmd_req = NO_REQ;
                    break;
                case DISARM:
                    ROS_INFO("COMANDO : DISARMA");
                    res = disarm_vehicle();
                    if(res)
                    {   
                        ROS_INFO("DISARMATO");
                        current_cmd_req = NO_REQ;
                        std_msgs::Int32 msg;
                        msg.data = ARMABLE;
                        state_pub.publish(msg);
                    }
                    else
                        current_cmd_req = NO_REQ;
                    break;
                case TAKEOFF:
                    ROS_INFO("COMANDO : TAKE OFF");
                    res = takeoff_vehicle();
                    if(res)
                    {   
                        ROS_INFO("TAKE OFF");
                        current_cmd_req = NO_REQ;
                        std_msgs::Int32 msg;
                        msg.data = TAKEOFF;
                        state_pub.publish(msg);
                    }
                    else
                    {
                        if(!init_takeoff)
                            current_cmd_req = NO_REQ;
                        else
                            current_cmd_req = TAKEOFF;
                    }
                    break;
                case LAND:
                    ROS_INFO("COMANDO : LAND");
                    //res = arm_vehicle();
                    if(res)
                    {   
                        ROS_INFO("LANDED");
                        current_cmd_req = NO_REQ;
                        std_msgs::Int32 msg;
                        msg.data = LANDED;
                        state_pub.publish(msg);
                    }
                    else
                        current_cmd_req = NO_REQ;
                    break;
                case RTL:
                    ROS_INFO("COMANDO : RTL");
                    //res = arm_vehicle();
                    if(res)
                    {   
                        ROS_INFO("RTL");
                        current_cmd_req = NO_REQ;
                        std_msgs::Int32 msg;
                        msg.data = RTL;
                        state_pub.publish(msg);
                    }
                    else
                        current_cmd_req = NO_REQ;
                    break;
                case EMERGENCY_STOP:
                    ROS_INFO("COMANDO : EMERGENCY_STOP");
                    //res = arm_vehicle();
                    if(res)
                    {   
                        ROS_INFO("EMERGENCY_STOP");
                        current_cmd_req = NO_REQ;
                        std_msgs::Int32 msg;
                        msg.data = EMERGENCY_STOP;
                        state_pub.publish(msg);
                    }
                    else
                        current_cmd_req = NO_REQ;
                    break;

                case CLEAR_RADIO_OVERRIDE:
                    ROS_INFO("COMANDO : CLEAR_RADIO_OVERRIDE");
                    clear_radio_override();
                    current_cmd_req = NO_REQ;
                    break;

                case HOLD_POSITION:
                    ROS_INFO("COMANDO : HOLD POSITION");
                    hold_position();
                    current_cmd_req = NO_REQ;
                    break;
               
            } 

        }//fine current_cmd_req/////////////////////////////////////////////////////////////////////////////////////
        if(current_mode_req != NO_MODE)
        {
            mavros_msgs::SetMode offb_set_mode;
            switch(current_mode_req)
            {

                case ALT_HOLD:
                    ROS_INFO("FM : ALT HOLD ");
                    
                    offb_set_mode.request.custom_mode = "ALT_HOLD";
                    set_mode_client.call(offb_set_mode);
                    
                    if(offb_set_mode.response.success)
                    {   
                        ROS_INFO("ALT HOLD");
                        current_mode_req = NO_MODE;
                    }
                    else
                        current_mode_req = NO_MODE;
                    break;

                case STABILIZE:
                    ROS_INFO("FM : STABILIZE ");
                    
                    offb_set_mode.request.custom_mode = "STABILIZE";
                    set_mode_client.call(offb_set_mode);
                    
                    if(offb_set_mode.response.success)
                    {   
                        ROS_INFO("STABILIZE");
                        current_mode_req = NO_MODE;
                    }
                    else
                        current_mode_req = NO_MODE;
                    break;

                case LOITER:
                    ROS_INFO("FM : LOITER ");
                    
                    offb_set_mode.request.custom_mode = "LOITER";
                    set_mode_client.call(offb_set_mode);
                    
                    if(offb_set_mode.response.success)
                    {   
                        ROS_INFO("LOITER");
                        current_mode_req = NO_MODE;
                    }
                    else
                        current_mode_req = NO_MODE;
                    break;

                default:
                    current_mode_req = NO_MODE;


            }
        }//fine current_mode_req/////////////////////////////////////////////////////////////////////////////////////


    /********************************************************************************************/
    /*                                                                                         */
    /*                     CICLO PRINCIPALE CONTROLLO                                          */
    /*                                                                                         */
    /*******************************************************************************************/
    if(waypoint_recv && !manual_mode)
    {
        if(waypoint_recv == 1)
        {   
            ROS_INFO("NUOVO WAYPOINT");
            //se è un nuovo waypoint rinizializzo i controllori
            pid_controllers.roll.init_PID();
            pid_controllers.pitch.init_PID();
            pid_controllers.yaw.init_PID();
            //pid_controllers.altitude.init_PID();

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


        /******************CALOCLO DELL'AZIONE DI CONTROLLO*****************************************/
        //il ciclo di controllo lo eseguo a loop_rate Hz

        if(elapsed_time_control  >= (1000/loop_rate)) //30Hz
        {   
            ROS_INFO("CICLO CONTROLLO");
            cout << "FREQUENZA CONTROLLO" << 1000/elapsed_time_control <<endl;
            //verifico che ho dati di stima di posizione validi da almeno 1 secondd
            if(elapsed_time_pose <= 1000/1)
            {
                update_control();
                
            }
            else
            {
                ROS_WARN("NON HO STIMA DELLA POSIZIONE DA 1 SECONDO");
                //imposto i valori di pwm di yaw, pitch e roll al minimo
                //todo: per l'altezza?
                double pwm_throttle = 1500;
                warning_stop(pwm_throttle);
                
            }
            gettimeofday(&control_time, NULL); 
        }

        //test gradino per identificazione 
        //step_test();
        /************************************************************************************************/
        

    }






        ros::spinOnce();
        //rate.sleep();

    }


    return 0;
}

