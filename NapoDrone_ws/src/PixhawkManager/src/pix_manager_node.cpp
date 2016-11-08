
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
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    //comandi
    cmd_sub =  nh.subscribe<std_msgs::Int32>("/napodrone/cmd_request", 1, cmd_cb);
    //mode request
    mode_sub =  nh.subscribe<std_msgs::Int32>("/napodrone/mode_request", 1, mode_cb);
    //param request
    param_sub =  nh.subscribe<std_msgs::Int32>("/napodrone/param_request", 1, param_cb);
  
    //topic per override la radio
    rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
    //topic per scrivere lo stato
    state_pub = nh.advertise<std_msgs::Int32>("napodrone/px4_status", 10);
    

    //service per armare il drone
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //service per cambiare il modo del drone
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //leggo i parametri specificati nel launch file
    nh.param<int>("/PixManager/loop_rate", loop_rate, 50);

    //INIT//////////////////////////////////////////////////////////////////////////////////////////
    //imposto la frequenza del nodo 
    ros::Rate rate(loop_rate);
    //inizializzo richiesta di comando
    current_cmd_req = NO_REQ;

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
    ROS_INFO("SONO CONNESSO 1");

    //impostiamo lo stato del drone a STABILIZE DI DEFAULT
    mavros_msgs::SetMode default_mode;
    default_mode.request.custom_mode = init_flight_mode;
    ros::Time last_request = ros::Time::now();
    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    
   
    //Attendo che sia in default mode
    /*
    while(ros::ok() && current_state.mode != init_flight_mode)
    {
        if( (ros::Time::now() - last_request > ros::Duration(5.0)))
            ROS_INFO("PROVO  SETTARE IL MODE");
            if( set_mode_client.call(default_mode) && default_mode.response.success){
                ROS_INFO("DEFUALT MODE ENABLED");
            }
            last_request = ros::Time::now();
    }
    ROS_INFO("DEFUALT MODE SET");
    */
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
                    break;
                case TAKEOFF:
                    ROS_INFO("COMANDO : TAKE OFF");
                    //res = arm_vehicle();
                    if(res)
                    {   
                        ROS_INFO("TAKE OFF");
                        current_cmd_req = NO_REQ;
                        std_msgs::Int32 msg;
                        msg.data = TAKEOFF;
                        state_pub.publish(msg);
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
                    break;
               
            } 

        }//fine current_cmd_req/////////////////////////////////////////////////////////////////////////////////////



        ros::spinOnce();
        rate.sleep();

    }


    return 0;
}

/*        if(current_state.armed)
        {
        ROS_INFO("Vehicle override");
        //local_pos_pub.publish(pose);
        mavros_msgs::OverrideRCIn rc;
        
        rc.channels[0] = 0;
        rc.channels[1] = 0;
        rc.channels[2] = 1500;
        rc.channels[3] = 0; // yaw
        //rc.channels[4] = 500;
        rc_pub.publish(rc);



        }
        else{
        
        //local_pos_pub.publish(pose);
        mavros_msgs::OverrideRCIn rc;
        
        rc.channels[3] = 0;
        rc.channels[1] = 0;
        rc.channels[2] = 0;
        rc.channels[0] = 0;
        //rc.channels[4] = 500;
        rc_pub.publish(rc);
        }

    }
*/
