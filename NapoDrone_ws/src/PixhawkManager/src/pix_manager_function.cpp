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
		case CLEAR_RADIO_OVERRIDE:
			current_cmd_req = CLEAR_RADIO_OVERRIDE;
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
   	switch(msg->data)
    {
		case STABILIZE:
         	current_mode_req = STABILIZE;          
			break;
		case ALT_HOLD:
         	current_mode_req = ALT_HOLD;          
			break;
		case AUTO:
         	current_mode_req = AUTO;          
			break;
		case ACRO:
         	current_mode_req = ACRO;          
			break;
		case SPORT:
         	current_mode_req = SPORT;          
			break;
		case DRIFT:
			current_mode_req = DRIFT;
			break;
		case GUIDED:
			current_mode_req = GUIDED;          
			break;
		case CIRCLE:
			current_mode_req = CIRCLE;          
			break;
		case POS_HOLD:
			current_mode_req = POS_HOLD;          
			break;
		case BRAKE:
			current_mode_req = BRAKE;          
			break;
		case FOLLOW_ME:
			current_mode_req = FOLLOW_ME;          
			break;
		case SIMPLE_SUPER:
			current_mode_req = SIMPLE_SUPER;          
			break;
		default:
			current_mode_req = NO_MODE;
     }         
}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK PER RICHIESTE DI PARAMETRI                                                    */
/*                                                                                         */
/*******************************************************************************************/
void param_cb(const serial_manager::Param::ConstPtr& msg)
{
    switch(msg->header)
    {
		case ALT_TAKEOFF:
         	alt_takeoff_target = msg->param; 
         	ROS_INFO("RICEVUTO PARAMTRO ALTEZZA");         
			break;
		default:          
			break;
	}
}/********************************************************************************************/
/*                                                                                         */
/*    CALBACK PER RWAYPOINT                                                                */
/*                                                                                         */
/*******************************************************************************************/
void waypoint_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
  //ricevuto un nuovo waypoint
  current_waypoint = *msg;
  ROS_INFO("RICEVUTO NUOVO WAYPOINT"); 
  waypoint_recv = 1;


}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK STIMA DI POSIZIONE     	                                                    */
/*                                                                                         */
/*******************************************************************************************/
void poses_cb(const aruco_mapping::ArucoMarker::ConstPtr& msg)
{
	marker_visibile = msg->marker_visibile;
	if(marker_visibile)
	{
		//ho una stima buona della posizione della camera
		global_camera_pose.position.x = msg->global_camera_pose.position.x;
		global_camera_pose.position.y = msg->global_camera_pose.position.y;
		global_camera_pose.position.z = msg->global_camera_pose.position.z;
		//transformo il quaternione in un angoli di eulero
		quaternion_2_euler(msg->global_camera_pose.orientation.x,msg->global_camera_pose.orientation.y,
			msg->global_camera_pose.orientation.z,msg->global_camera_pose.orientation.w, 
			global_camera_pose.orientation.x,global_camera_pose.orientation.y,global_camera_pose.orientation.z);

		cout << "POSA" << endl;
		cout << global_camera_pose.position.x  << " " <<  global_camera_pose.position.y << " " << global_camera_pose.position.z  << endl;
		cout << "ORIENTAZIONE" << endl;
		cout << global_camera_pose.orientation.x*180.0/3.14 << " " <<  global_camera_pose.orientation.y*180.0/3.14 << " " << global_camera_pose.orientation.z*180.0/3.14  << endl;
	}
	else
	{
		//non ho una stima della poszione del drone
	}


}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK PER RICHIESTE DI PARAMETRI                                                    */
/*                                                                                         */
/*******************************************************************************************/
void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
  
  current_pressure = msg->fluid_pressure;
  if(init_pressure == 0)
  	init_pressure = msg->fluid_pressure;
  else
  {
  	
  	//calcolo altezza:
  	//8.23(m) : 100(pa) = altezza : variazione
  	alt_from_barometer = -(8.23*(current_pressure - init_pressure))/100.0 ; 
  	/*std::cout << "alt: " <<  alt << std::endl;
  	FILE* fd;
  	fd = fopen("/home/sistema/press.txt", "a");
  	fprintf(fd, "%f\n",alt );
  	fclose(fd);*/
  }
  

}
/********************************************************************************************/
/*                                                                                         */
/*    ARMA VEHICLE                                                                         */
/*                                                                                         */
/*******************************************************************************************/
bool arm_vehicle()
{


	//devo impostare il pwm del throttle al valore minimo.
	mavros_msgs::OverrideRCIn radio_pwm;
	radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
	radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
	radio_pwm.channels[RC_THROTTLE] = PWM_LOW_LIMIT;
	radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
	rc_pub.publish(radio_pwm);
	//funzione richiamata per armare il veicolo
	mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
    
  //provo a armare
  arming_client.call(arm_cmd);

  if(arm_cmd.response.success)
  {
    //imposto il valore di thtrolle piu del minimo per non farlo disarmare
    radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
    radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
    radio_pwm.channels[RC_THROTTLE] = PWM_LOW_LIMIT + 100;
    radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
    rc_pub.publish(radio_pwm);
  }
  return arm_cmd.response.success;
	

}
/********************************************************************************************/
/*                                                                                         */
/*    DISARMA VEHICLE                                                                         */
/*                                                                                         */
/*******************************************************************************************/
bool disarm_vehicle()
{

	//devo impostare il pwm del throttle al valore minimo.
	mavros_msgs::OverrideRCIn radio_pwm;
	radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
	radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
	radio_pwm.channels[RC_THROTTLE] = PWM_LOW_LIMIT;
	radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
	rc_pub.publish(radio_pwm);
	//funzione richiamata per disarmare il veicolo
	mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
    
    //provo a disarmare
    arming_client.call(disarm_cmd);
    return disarm_cmd.response.success;
	

}
/********************************************************************************************/
/*                                                                                         */
/*    DISARMA VEHICLE                                                                         */
/*                                                                                         */
/*******************************************************************************************/
bool takeoff_vehicle()
{
	//se non è armato non posso fare il takeoff
	if(!current_state.armed)
	{
		init_takeoff = false;
		ROS_INFO("PLEASE ARM BEFORE TAKEOFF");
		return false;
	}
	//all'inizio devo inizializzare il takeoff
	if(!init_takeoff)
	{
		init_takeoff = true;
		//takeoff_pressure = current_pressure;
		return false;	
	}
	
/*
	//devo impostare il pwm del throttle al valore minimo.
	mavros_msgs::OverrideRCIn radio_pwm;
	radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
	radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
	radio_pwm.channels[RC_THROTTLE] = PWM_LOW_LIMIT;
	radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
	rc_pub.publish(radio_pwm);
	//funzione richiamata per disarmare il veicolo
	mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
    
    //provo a disarmare
    arming_client.call(disarm_cmd);
    return disarm_cmd.response.success;*/
	

}
/********************************************************************************************/
/*                                                                                         */
/*    CLEAR RADIO OVERRIDE                                                                  */
/*                                                                                         */
/*******************************************************************************************/
void clear_radio_override()
{

	//devo impostare il pwm del throttle al valore minimo.
	mavros_msgs::OverrideRCIn radio_pwm;
	radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
	radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
	radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
	radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
	radio_pwm.channels[4] = NO_OVERRIDE;
	radio_pwm.channels[5] = NO_OVERRIDE;
	radio_pwm.channels[6] = NO_OVERRIDE;
	radio_pwm.channels[7] = NO_OVERRIDE;
	rc_pub.publish(radio_pwm);

}
/********************************************************************************************/
/*                                                                                         */
/*    PID CONTROLLERS                                                                      */
/*                                                                                         */
/*******************************************************************************************/
PIDController::PIDController()
{
}

void PIDController::init(double Kp, double Ki, double Kd, double Ts, double Nd,double limit_max, double limit_min)
{
	double* vet_a = new double[3];
	double* vet_b = new double[3];

  	//denominatore
  	vet_a[0] = 1;
  	vet_a[1] = -2+ Nd*Ts;
  	vet_a[2] = 1 - Nd*Ts;
  	//numeratore
  	vet_b[0] = Kd*Nd  + Kp ;
  	vet_b[1] = -2*Kd*Nd  - 2*Kp  + Ki*Ts + Nd*Kp*Ts;
  	vet_b[2] = Kd*Nd + Kp - Ki*Ts - Nd*Kp*Ts + Ki*Nd*Ts*Ts;
  	length_a = 3;
  	length_b = 3;

  	a = new double[length_a];
  	b = new double[length_b];
  	y = new double[length_a];
  	u = new double[length_b];
  	limit_up = limit_max;
  	limit_down = limit_min;
  
  	for (int i = 0; i < length_a; i++)
  	{
    	a[i] = vet_a[i];
    
    	y[i] = 0.0;
  	}

  	for (int i = 0; i < length_b; i++)
  	{
    
    	b[i] = vet_b[i];
    
    	u[i] = 0.0;
    
  	}
}

double PIDController::update(double errore)
{
  	//aggiorno vettore ingressi
  	for (int i = length_b - 1; i > 0; i--)
  	{ 
    	//sposto tutti i valori di una posizione all'interno dell'array
    	u[i] = u[i - 1];

  	}
  	//inserisco in testa
  	u[0] = errore;

  	double uscita = 0.0;
  	//implemento y(k) = b0*u(k) +.....+ bn*u(k-n)
  	for (int i = 0; i < length_b; i++)
  	{
    	uscita = uscita + b[i] * u[i];
  	}
    
    for (int i = length_a - 1; i > 0; i--)
  	{
    	//sposto tutti i valori di una posizione all'interno dell'array
    	y[i] = y[i - 1];

  	}
  	//implemento y(k) = -a1*y(k-1) -.......-an*y(k-n)
  	for (int i = 1; i < length_a; i++)
  	{
    	uscita = uscita - a[i] * y[i];
  	}
  
  	//saturazioni
  	if(uscita > 0 && uscita > limit_up)
  	{
    	uscita = limit_up;
  	}
  	else if(uscita < 0 && uscita < limit_down)
  	{
    	uscita = limit_down;
  	}      

  	//insierisco in testa
  	y[0] = uscita;


  	return uscita;
}
/********************************************************************************************/
/*                                                                                         */
/*    INIT                                                                     */
/*                                                                                         */
/*******************************************************************************************/
void init_global_variables()
{
 	//inizializzo richiesta di comando
    current_cmd_req = NO_REQ;
    init_takeoff = false;
    init_pressure = 0;
    alt_from_barometer = 0;

    //inizializzo i controllori
    pid_controllers.roll.init(Kp_roll,Ki_roll,Kd_roll,Ts_roll, Nd_roll,limit_max_roll,limit_min_roll);
    pid_controllers.pitch.init(Kp_pitch,Ki_pitch,Kd_pitch,Ts_pitch, Nd_pitch,limit_max_pitch,limit_min_pitch);
    pid_controllers.yaw.init(Kp_yaw,Ki_yaw,Kd_yaw,Ts_yaw, Nd_yaw,limit_max_yaw,limit_min_yaw);
    pid_controllers.altitude.init(Kp_alt,Ki_alt,Kd_alt,Ts_alt, Nd_alt,limit_max_alt,limit_min_alt);

    marker_visibile = false;
	//Pose global_camera_pose;
    //..
	//altezza da raggiungere in takeoff
	alt_takeoff_target = 1.0;

  //inizializzo waypoint
  current_waypoint.position.x = 0;
  current_waypoint.position.y = 0;
  current_waypoint.position.z = 0;
  current_waypoint.orientation.x = 0;
  current_waypoint.orientation.y = 0;
  current_waypoint.orientation.z = 0;
  current_waypoint.orientation.w = 0;
  

    
}
/********************************************************************************************/
/*                                                                                         */
/*    QUATERNION_2_EULER                                                                    */
/*                                                                                         */
/*******************************************************************************************/
void quaternion_2_euler(double xquat, double yquat, double zquat, double wquat, double& roll, double& pitch, double& yaw)
{
  //Trasformo le corrdinate SVO in coordinate frame camera.
  double r11 = wquat*wquat + xquat*xquat - yquat*yquat - zquat*zquat;
  double r12 = 2*(xquat*yquat - wquat*zquat);
  double r13 = 2*(zquat*xquat + wquat*yquat);
  double r21 =  2*(xquat*yquat + wquat*zquat);
  double r22 = wquat*wquat - xquat*xquat + yquat*yquat - zquat*zquat;
  double r23 = 2*(yquat*zquat - wquat*xquat);
  double r31 = 2*(zquat*xquat - wquat*yquat);
  double r32 = 2*(yquat*zquat + wquat*xquat);
  double r33 = wquat*wquat - xquat*xquat - yquat*yquat + zquat*zquat;
  //Scrivo la trasposta:
  double rt11,rt12,rt13,rt21,rt22,rt23,rt31,rt32,rt33;
  rt11 = r11;
  rt12 = r21;
  rt13 = r31;
  rt21 = r12;
  rt22 = r22;
  rt23 = r32;
  rt31 = r13;
  rt32 = r23;
  rt33 = r33;
  //calcolo angoli di eulero
  roll = atan2(rt23,rt33);
  pitch = -asin(rt13);
	yaw = atan2(rt12,rt11);
}
 /********************************************************************************************/
/*                                                                                         */
/*    QUATERNION_2_EULER                                                                    */
/*                                                                                         */
/*******************************************************************************************/
void update_PID()
{

  //controllo di altezza
  double z = global_camera_pose.position.z;
  double z_des = current_waypoint.position.z;
  //calcolo errore
  double e_z = z_des - z ;
  double u_z = pid_controllers.altitude.update(e_z); 

  //devo mappare l'ingresso in un comando ai servo
  //...






}