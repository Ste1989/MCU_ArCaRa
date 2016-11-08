#include "pix_manager_node.h"

/********************************************************************************************/
/*                                                                                         */
/*    CALBACK PER LEGGERE LO STATO DEL DRONE                                               */
/*                                                                                         */
/*******************************************************************************************/
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    
    //guardo se è cambiato lo stato della connessione
    if(current_state.connected != msg->connected)
    {   
        if(msg->connected)
        {   
           std_msgs::Int32 msg;
           msg.data = CONNECTED;
           state_pub.publish(msg);
           ROS_INFO("SONO CONNESSO ALL'AUTOPILOTA");
        }   
        else
        {
           std_msgs::Int32 msg;
           msg.data = DISCONNECTED;
           state_pub.publish(msg);
           ROS_INFO("SONO DISCONNESSO ALL'AUTOPILOTA");

        }

    }
    current_state = *msg;
    //aremd
    //connected
    //mode
    //guided
}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK PER RICHIESTE DI COMANDI                                                     */
/*                                                                                         */
/*******************************************************************************************/
void cmd_cb(const std_msgs::Int32::ConstPtr& msg)
{	
	ROS_INFO("SONO QUA");
 	switch(msg->data)
    {
            
    	case ARM:
         	current_cmd_req = ARM;          
			break;
		case DISARM:
         	current_cmd_req = DISARM;          
			break;

		case TAKEOFF:
         	current_cmd_req = TAKEOFF;          
			break;

		case LAND:
         	current_cmd_req = LAND;          
			break;

		case RTL:
         	current_cmd_req = RTL;          
			break;

		case EMERGENCY_STOP:
         	current_cmd_req = EMERGENCY_STOP;          
			break;
		default:
			current_cmd_req = NO_REQ;          
			break;
     }         
}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK PER RICHIESTE DI FLIGHT MODE                                                    */
/*                                                                                         */
/*******************************************************************************************/
void mode_cb(const std_msgs::Int32::ConstPtr& msg)
{
  
}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK PER RICHIESTE DI PARAMETRI                                                    */
/*                                                                                         */
/*******************************************************************************************/
void param_cb(const std_msgs::Int32::ConstPtr& msg)
{
  
}

/********************************************************************************************/
/*                                                                                         */
/*    ARMA VEHICLE                                                                         */
/*                                                                                         */
/*******************************************************************************************/
bool arm_vehicle()
{
	//funzione richiamata per armare il veicolo
	mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    //provo a armare
    arming_client.call(arm_cmd);
    return arm_cmd.response.success;
	

}
/********************************************************************************************/
/*                                                                                         */
/*    DISARMA VEHICLE                                                                         */
/*                                                                                         */
/*******************************************************************************************/
bool disarm_vehicle()
{
	//funzione richiamata per disarmare il veicolo
	mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
    
    //provo a armare
    arming_client.call(disarm_cmd);
    return disarm_cmd.response.success;
	

}