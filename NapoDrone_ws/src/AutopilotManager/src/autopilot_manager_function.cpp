#include "autopilot_manager_node.h"

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
      ROS_INFO("RICEVUTO PARAMETRO ALTEZZA");         
			break;
    /*ROLL***************************************************************************************/
    case K_ROLL:
      pid_controllers.roll.set_K(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO K_ROLL");         
      break;
    case B_ROLL:
      pid_controllers.roll.set_b(msg->param); 
      pid_controllers.roll.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO B_ROLL");         
      break;
    case TI_ROLL:
      pid_controllers.roll.set_Ti(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TI_ROLL");         
      break;
    case TS_ROLL:
      pid_controllers.roll.set_Ts(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TS_ROLL");         
      break;
    case TD_ROLL:
      pid_controllers.roll.set_Td(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TD_ROLL");         
      break;
    case ND_ROLL:
      pid_controllers.roll.set_Nd(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO ND_ROLL");         
      break;
    case LUP_ROLL:
      pid_controllers.roll.set_saturazione_max(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO LUP_ROLL");         
      break;
    case LDOWN_ROLL:
      pid_controllers.roll.set_saturazione_min(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO LDOWN_roll");         
      break;
    /*PITCH***************************************************************************************/
    case K_PITCH:
      pid_controllers.pitch.set_K(msg->param); 
      pid_controllers.pitch.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO K_PITCH");         
      break;
    case B_PITCH:
      pid_controllers.pitch.set_b(msg->param); 
      pid_controllers.pitch.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO B_PITCH");         
      break;
    case TI_PITCH:
      pid_controllers.pitch.set_Ti(msg->param); 
      pid_controllers.pitch.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TI_PITCH");         
      break;
    case TS_PITCH:
      pid_controllers.pitch.set_Ts(msg->param);  
      pid_controllers.pitch.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TS_PITCH");         
      break;
    case TD_PITCH:
      pid_controllers.pitch.set_Td(msg->param); 
      pid_controllers.pitch.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TD_PITCH");         
      break;
    case ND_PITCH:
      pid_controllers.pitch.set_Nd(msg->param); 
      pid_controllers.pitch.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO ND_PITCH");         
      break;
    case LUP_PITCH:
      pid_controllers.pitch.set_saturazione_max(msg->param); 
      pid_controllers.pitch.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO LUP_PITCH");         
      break;
    case LDOWN_PITCH:
      pid_controllers.pitch.set_saturazione_min(msg->param); 
      pid_controllers.pitch.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO LDOWN_PITCH");         
      break;
    /*YAW***************************************************************************************/
    case K_YAW:
      pid_controllers.yaw.set_K(msg->param); 
      pid_controllers.yaw.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO K_YAW");         
      break;
    case B_YAW:
      pid_controllers.yaw.set_b(msg->param); 
      pid_controllers.yaw.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO B_YAW");         
      break;
    case TI_YAW:
      pid_controllers.yaw.set_Ti(msg->param); 
      pid_controllers.yaw.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TI_YAW");         
      break;
    case TS_YAW:
      pid_controllers.yaw.set_Ts(msg->param); 
      pid_controllers.yaw.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TS_YAW");         
      break;
    case TD_YAW:
      pid_controllers.yaw.set_Td(msg->param); 
      pid_controllers.yaw.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TD_YAW");         
      break;
    case ND_YAW:
      pid_controllers.yaw.set_Nd(msg->param); 
      pid_controllers.yaw.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO ND_YAW");         
      break;
    case LUP_YAW:
      pid_controllers.yaw.set_saturazione_max(msg->param); 
      pid_controllers.yaw.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO LUP_YAW");         
      break;
    case LDOWN_YAW:
      pid_controllers.yaw.set_saturazione_min(msg->param);  
      pid_controllers.yaw.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO LDOWN_YAW");         
      break;
    /*ALT***************************************************************************************/
    case K_ALT:
      pid_controllers.altitude.set_K(msg->param); 
      pid_controllers.altitude.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO K_ALT");         
      break;
    case B_ALT:
      pid_controllers.altitude.set_b(msg->param); 
      pid_controllers.altitude.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO B_ALT");         
      break;
    case TI_ALT:
      pid_controllers.altitude.set_Ti(msg->param); 
      pid_controllers.altitude.init_PID();
      ROS_INFO("RICEVUTO PARAMETRO TI_ALT");         
      break;
    case TS_ALT:
      pid_controllers.altitude.set_Ts(msg->param);  
      pid_controllers.altitude.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TS_ALT");         
      break;
    case TD_ALT:
      pid_controllers.altitude.set_Td(msg->param); 
      pid_controllers.altitude.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO TD_ALT");         
      break;
    case ND_ALT:
      pid_controllers.altitude.set_Nd(msg->param); 
      pid_controllers.altitude.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO ND_ALT");         
      break;
    case LUP_ALT:
      pid_controllers.altitude.set_saturazione_max(msg->param); 
      pid_controllers.altitude.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO LUP_ALT");         
      break;
    case LDOWN_ALT:
      pid_controllers.altitude.set_saturazione_min(msg->param); 
      pid_controllers.altitude.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO LDOWN_ALT");         
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

void PIDController::init_PID()
{
  I_k = 0;
  y_k = 0;
  D_k = 0;
}

double PIDController::update_PID(double y, double y_des)
{
  //calcolo dell'errore
  double e = y_des - y;
  
  //P
  double P = K * (b*y_des - y);

  //I
  double I_k_1 = 0;
  if(Ti == 0)
    I_k_1 = 0;
  else
    I_k_1 = I_k + (K*Ts)/Ti * e;
  //memorizzo integrale
  I_k = I_k_1;

  //D
  double D_k_1 = 0;
  if((Nd*Ts+Td) == 0)
    D_k_1 = 0;
  else
    D_k_1 = Td/(Nd*Ts+Td) * D_k - (K*Td*Nd)/(Nd*Ts+Td) * (y - y_k);
  //memorizzo derivativo
  y_k = y; 
  D_k = D_k_1;


  return (P + I_k_1 + D_k_1);
}
double PIDController::map_control_2_radio(double u)
{

  double m = (saturazione_min - saturazione_max)/2;
  double q = PWM_MEDIUM;
  double y = m*u + q;

  if(y < saturazione_min)
    y = saturazione_min;
  if(y > saturazione_max)
    y = saturazione_max;
    
  
  cout << "PWM CALCOLATO: " << y << endl; 

  return y;
} 

void PIDController::set_K(double param)
{
  K = param;
}
void PIDController::set_b(double param)
{
  b = param;
}
void PIDController::set_Ti(double param)
{
  Ti = param;
}
void PIDController::set_Td(double param)
{
  Td = param;
}
void PIDController::set_Ts(double param)
{
  Ts = param;
}
void PIDController::set_Nd(double param)
{
  Nd = param;
}
void PIDController::set_saturazione_max(double param)
{
  saturazione_max = param;
}
void PIDController::set_saturazione_min(double param)
{
  saturazione_min = param;
}


double PIDController::get_K()
{
  return K;
}
double PIDController::get_b()
{
  return b;
}
double PIDController::get_Ti()
{
  return Ti;
}
double PIDController::get_Td()
{
  return Td;
}
double PIDController::get_Ts()
{
  return Ts;
}
double PIDController::get_Nd()
{
  return Nd;
}
double PIDController::get_saturazione_max()
{
  return saturazione_max;
}
double PIDController::get_saturazione_min()
{
  return saturazione_min;
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
  pid_controllers.roll.init_PID();
  pid_controllers.pitch.init_PID();
  pid_controllers.yaw.init_PID();
  pid_controllers.altitude.init_PID();

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
/*    UPDATE_PID CONTROL                                                                   */
/*                                                                                         */
/*******************************************************************************************/
void update_control()
{

  //3)controllo di HEADING
  double rz = global_camera_pose.orientation.z;
  double rz_des = current_waypoint.orientation.z;
  //calcolo il controllo da attuare
  double u_rz = pid_controllers.yaw.update_PID(rz, rz_des);
  //devo mappare l'ingresso in un comando ai servo
  double yaw_commad = pid_controllers.yaw.map_control_2_radio(u_rz);


 //publish control to radio
  mavros_msgs::OverrideRCIn radio_pwm;
  radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
  radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = yaw_commad;
  rc_pub.publish(radio_pwm);
}
 /********************************************************************************************/
/*                                                                                         */
/*    LEGGI PID FILE                                                                       */
/*                                                                                         */
/*******************************************************************************************/
bool leggi_PID_file(std::string PID_file)
{
  //lettura del file PID
  const char* filename = PID_file.c_str();
  FILE* fd;
  fd = fopen(filename,"rb");
  if( fd == NULL ) {
    ROS_ERROR("File PID NOT FOUND ");
    return false;
  }
  double param;
  int i = 0;
  double vect_pid[4*8];
  while(!feof(fd)) 
  {
    fscanf(fd,"%lf",&param );
    vect_pid[i] = param;
    i++;
  }
  fclose(fd);

  //ricopio valori nel file
  //roll
  pid_controllers.roll.set_K(vect_pid[0]);
  pid_controllers.roll.set_b(vect_pid[1]);
  pid_controllers.roll.set_Ti(vect_pid[2]);
  pid_controllers.roll.set_Ts(vect_pid[3]);
  pid_controllers.roll.set_Td(vect_pid[4]);
  pid_controllers.roll.set_Nd(vect_pid[5]);
  pid_controllers.roll.set_saturazione_max(vect_pid[6]);
  pid_controllers.roll.set_saturazione_min(vect_pid[7]);
  //pitch
  pid_controllers.pitch.set_K(vect_pid[8]);
  pid_controllers.pitch.set_b(vect_pid[9]);
  pid_controllers.pitch.set_Ti(vect_pid[10]);
  pid_controllers.pitch.set_Ts(vect_pid[11]);
  pid_controllers.pitch.set_Td(vect_pid[12]);
  pid_controllers.pitch.set_Nd(vect_pid[13]);
  pid_controllers.pitch.set_saturazione_max(vect_pid[14]);
  pid_controllers.pitch.set_saturazione_min(vect_pid[15]);
  //yaw
  pid_controllers.yaw.set_K(vect_pid[16]);
  pid_controllers.yaw.set_b(vect_pid[17]);
  pid_controllers.yaw.set_Ti(vect_pid[18]);
  pid_controllers.yaw.set_Ts(vect_pid[19]);
  pid_controllers.yaw.set_Td(vect_pid[20]);
  pid_controllers.yaw.set_Nd(vect_pid[21]);
  pid_controllers.yaw.set_saturazione_max(vect_pid[22]);
  pid_controllers.yaw.set_saturazione_min(vect_pid[23]);
  //alt
  pid_controllers.altitude.set_K(vect_pid[24]);
  pid_controllers.altitude.set_b(vect_pid[25]);
  pid_controllers.altitude.set_Ti(vect_pid[26]);
  pid_controllers.altitude.set_Ts(vect_pid[27]);
  pid_controllers.altitude.set_Td(vect_pid[28]);
  pid_controllers.altitude.set_Nd(vect_pid[29]);
  pid_controllers.altitude.set_saturazione_max(vect_pid[30]);
  pid_controllers.altitude.set_saturazione_min(vect_pid[31]);

  ROS_INFO_STREAM("File pid loaded successfully");
  return true;

}

 /********************************************************************************************/
/*                                                                                         */
/*    SCRIVI FILE PID                                                                      */
/*                                                                                         */
/*******************************************************************************************/
bool scrivi_PID_file(std::string PID_file)
{
  //scrittura del file PIDs
  const char* filename = PID_file.c_str();
  FILE* fd;
  fd = fopen(filename,"w");
  fprintf(fd, "%lf\n", pid_controllers.roll.get_K());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_b());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Ti());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Ts());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Td());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Nd());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_saturazione_max());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_saturazione_min());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_K());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_b());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Ti());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Ts());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Td());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Nd());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_saturazione_max());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_saturazione_min());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_K());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_b());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Ti());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Ts());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Td());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Nd());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_saturazione_max());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_saturazione_min());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_K());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_b());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Ti());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Ts());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Td());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Nd());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_saturazione_max());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_saturazione_min());
  fclose(fd);

  ROS_INFO_STREAM("File pid saved successfully");
  return true;

}
