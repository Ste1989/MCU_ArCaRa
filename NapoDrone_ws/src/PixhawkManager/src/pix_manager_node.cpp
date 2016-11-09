
#include "pix_manager_node.h"


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
    param_sub =  nh.subscribe<std_msgs::Int32>("/napodrone/param_request", 1, param_cb);
  
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
    nh.param<int>("/PixManager/loop_rate", loop_rate, 100);
    nh.param<int>("/PixManager/stream_rate", stream_rate, 50);
    //PID param
    nh.param<double>("/PixManager/Kp_roll", Kp_roll, 0);
    nh.param<double>("/PixManager/Ki_roll", Ki_roll, 0);
    nh.param<double>("/PixManager/Kd_roll", Kd_roll, 0);
    nh.param<double>("/PixManager/Ts_roll", Ts_roll, 0);
    nh.param<double>("/PixManager/Nd_roll", Nd_roll, 0);

    nh.param<double>("/PixManager/Kp_pitch", Kp_pitch, 0);
    nh.param<double>("/PixManager/Ki_pitch", Ki_pitch, 0);
    nh.param<double>("/PixManager/Kd_pitch", Kd_pitch, 0);
    nh.param<double>("/PixManager/Ts_pitch", Ts_pitch, 0);
    nh.param<double>("/PixManager/Nd_pitch", Nd_pitch, 0);
    
    nh.param<double>("/PixManager/Kp_yaw", Kp_yaw, 0);
    nh.param<double>("/PixManager/Ki_yaw", Ki_yaw, 0);
    nh.param<double>("/PixManager/Kd_yaw", Kd_yaw, 0);
    nh.param<double>("/PixManager/Ts_yaw", Ts_yaw, 0);
    nh.param<double>("/PixManager/Nd_yaw", Nd_yaw, 0);

    nh.param<double>("/PixManager/Kp_alt", Kp_alt, 0);
    nh.param<double>("/PixManager/Ki_alt", Ki_alt, 0);
    nh.param<double>("/PixManager/Kd_alt", Kd_alt, 0);
    nh.param<double>("/PixManager/Ts_alt", Ts_alt, 0);
    nh.param<double>("/PixManager/Nd_alt", Nd_alt, 0);
    //INIT//////////////////////////////////////////////////////////////////////////////////////////
    init_global_variables();
    //imposto la frequenza del nodo 
    ros::Rate rate(loop_rate);
   
    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    //scrivo su topic che sono connesso
    std_msgs::Int32 msg;
    msg.data = CONNECTED;
    state_pub.publish(msg);
    ROS_INFO("SONO CONNESSO ALL'AUTOPILOTA");

   
    //una volta connesso avvio lo stream dati dal drone
    mavros_msgs::StreamRate srv_rate;
    srv_rate.request.stream_id = 0;
    srv_rate.request.message_rate = stream_rate;
    srv_rate.request.on_off = 1;

    set_stream_rate_client.call(srv_rate);
    ROS_INFO("STREAM DATI AVVIATO");

    /********************ciclo principale*************************************************************************************/
    while(ros::ok())
    {
        //GESTIONE DELLA RICHIESTA DI COMANDO
        bool res;
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
               
            } 

        }//fine current_cmd_req/////////////////////////////////////////////////////////////////////////////////////



        ros::spinOnce();
        //rate.sleep();

    }


    return 0;
}

