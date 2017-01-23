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

          //una volta connesso avvio lo stream dati dal drone
          mavros_msgs::StreamRate srv_rate;
          srv_rate.request.stream_id = 0;
          srv_rate.request.message_rate = 1000;//stream_rate
          srv_rate.request.on_off = 1;

          set_stream_rate_client.call(srv_rate);
          ROS_INFO("STREAM DATI AVVIATO");
    
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
    case HOLD_POSITION:
        current_cmd_req = HOLD_POSITION;
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
    case KP_ROLL:
      pid_controllers.roll.set_Kp(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KP_ROLL");         
      break;
    case B_ROLL:
      pid_controllers.roll.set_b(msg->param); 
      pid_controllers.roll.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO B_ROLL");         
      break;
    case KI_ROLL:
      pid_controllers.roll.set_Ki(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KI_ROLL");         
      break;
    case KY_ROLL:
      pid_controllers.roll.set_Ky(msg->param); 
      pid_controllers.roll.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KY_ROLL");         
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
    case KP_PITCH:
      pid_controllers.pitch.set_Kp(msg->param); 
      pid_controllers.pitch.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KP_PITCH");         
      break;
    case B_PITCH:
      pid_controllers.pitch.set_b(msg->param); 
      pid_controllers.pitch.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO B_PITCH");         
      break;
    case KI_PITCH:
      pid_controllers.pitch.set_Ki(msg->param); 
      pid_controllers.pitch.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KI_PITCH");         
      break;
    case KY_PITCH:
      pid_controllers.pitch.set_Ky(msg->param);  
      pid_controllers.pitch.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KY_PITCH");         
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
    case KP_YAW:
      pid_controllers.yaw.set_Kp(msg->param); 
      pid_controllers.yaw.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KP_YAW");         
      break;
    case B_YAW:
      pid_controllers.yaw.set_b(msg->param); 
      pid_controllers.yaw.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO B_YAW");         
      break;
    case KI_YAW:
      pid_controllers.yaw.set_Ki(msg->param); 
      pid_controllers.yaw.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KI_YAW");         
      break;
    case KY_YAW:
      pid_controllers.yaw.set_Ky(msg->param); 
      pid_controllers.yaw.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KY_YAW");         
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
    case KP_ALT:
      pid_controllers.altitude.set_Kp(msg->param); 
      pid_controllers.altitude.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KP_ALT");         
      break;
    case B_ALT:
      pid_controllers.altitude.set_b(msg->param); 
      pid_controllers.altitude.init_PID(); 
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO B_ALT");         
      break;
    case KI_ALT:
      pid_controllers.altitude.set_Ki(msg->param); 
      pid_controllers.altitude.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KI_ALT");         
      break;
    case KY_ALT:
      pid_controllers.altitude.set_Ky(msg->param);  
      pid_controllers.altitude.init_PID();
      scrivi_PID_file(PID_file); 
      ROS_INFO("RICEVUTO PARAMETRO KY_ALT");         
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
/*    CALBACK PER WAYPOINT                                                                */
/*                                                                                         */
/*******************************************************************************************/
void waypoint_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
  //ricevuto un nuovo waypoint
  current_waypoint_world = *msg;
  ROS_INFO("RICEVUTO NUOVO WAYPOINT"); 
  waypoint_recv = 1;
  manual_mode = false;
  hold_position_var = 0;


}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK STIMA DI POSIZIONE     	                                                    */
/*                                                                                         */
/*******************************************************************************************/
void poses_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	 
    
		//ho una stima buona della posizione del drone
		P_world_body_world.position.x = msg->pose.position.x;
		P_world_body_world.position.y = msg->pose.position.y;
	  P_world_body_world.position.z = msg->pose.position.z;
		//transformo il quaternione in un angoli di eulero
		quaternion_2_euler(msg->pose.orientation.x,msg->pose.orientation.y,
			msg->pose.orientation.z,msg->pose.orientation.w, 
			P_world_body_world.orientation.x,P_world_body_world.orientation.y,P_world_body_world.orientation.z);

    //prendo il tempo 
    gettimeofday(&pose_time, NULL); 

    //alzo un flag che ho una nuova stima
    new_pose_recv = 1;

		
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
	radio_pwm.channels[RC_THROTTLE] = PWM_LOW_LIMIT_THROTTLE;
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
    radio_pwm.channels[RC_THROTTLE] = PWM_LOW_LIMIT_THROTTLE + 100;
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
  radio_pwm.channels[RC_THROTTLE] = PWM_LOW_LIMIT_THROTTLE;
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
/*    TAKEOFF VEHICLE                                                                      */
/*                                                                                         */
/********************************************************************************************/
bool takeoff_vehicle()
{
	//se non è armato non posso fare il takeoff
	if(!current_state.armed)
	{
		init_takeoff = false;
    stato_takeoff = 0;
		ROS_INFO("PLEASE ARM BEFORE TAKEOFF");
		return false;
	}


	//all'inizio devo inizializzare il takeoff
	if(!init_takeoff)
	{
		init_takeoff = true;
    stato_takeoff = 1;
    //prendo il tempo in cui è arrivta la richiesta di takeoff
    gettimeofday(&takeoff_time, NULL);
		//inizializzo l'altezza desiderata di takeoff
    //current_waypoint_world.position.x  DA INIZIALIZZARE
    //current_waypoint_world.position.y  DA INIZIALIZZARE
    current_waypoint_world.position.z = alt_takeoff_target;
    current_waypoint_world.orientation.z = 0;
    //metto in automatico
    waypoint_recv = 1;
    manual_mode = 0;
    ROS_INFO("RICHIESTA DI TAKEOFF RICEVUTA: ALT %f HEADIND %f", alt_takeoff_target, 0.0);

		return true;	
	}
  else
  {
    ROS_INFO("COMANDO DI TAKEOFF IN ESECUZIONE");
    return true;
  }

	
}
/********************************************************************************************/
/*                                                                                         */
/*    CLEAR RADIO OVERRIDE                                                                  */
/*                                                                                         */
/*******************************************************************************************/
void clear_radio_override()
{
  //setto manual mode 
  manual_mode = true;
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
  e_k = 0;
  double Ts = 1.0/double(loop_rate);
  cout << "PID INIT "<< "KP: " << Kp << " KI: " << Ki << " KD: " << Td/(Nd*Ts+Td) << " KY: "<< (Ky*Td*Nd)/(Nd*Ts+Td)<< endl;
}

double PIDController::update_PID(double y, double y_des, double pwm_medium)
{
  //calcolo dell'errore
  double e = y_des - y;
  
  //P========================================================
  double P = Kp * (b*y_des - y);

  //I========================================================
  double I_k_1 = 0;
  //se l'errore ha cambiato segno scarico l'integrale
  if(e * e_k < 0)
    I_k = 0;  
  I_k_1 = I_k + Ki * e;
  //memorizzo integrale
  I_k = I_k_1;


  //D=========================================================
  double Ts = 1.0/double(loop_rate);
  double D_k_1 = 0;
  //se il denominatore è uguale a 0, oppure è l aprima iterazione di controllo non calcolo l'azione derivativa
  if((Nd*Ts+Td) == 0 || e_k == 0)
    D_k_1 = 0;
  else
    D_k_1 = Td/(Nd*Ts+Td) * D_k + (Ky*Td*Nd)/(Nd*Ts+Td) * (e - e_k);
    //D_k_1 = Td/(Nd*Ts+Td) * D_k - (Ky*Td*Nd)/(Nd*Ts+Td) * (y - y_k);
  
  //memorizzo derivativo
  e_k = e;
  y_k = y; 
  D_k = D_k_1;


  //aziobne di contorllo==========================================
  double u = (P + I_k_1 + D_k_1);

  //mappo l'azione di controllo nel pwm
  double m = (saturazione_min - saturazione_max)/2;
  double q = pwm_medium;
  double pwm = m*u + q;

  if(pwm < saturazione_min)
  {
    pwm = saturazione_min;
    //azione antiwindup: devo scaricare l'integrale dell'ultimo valore inserito
    //I_k = I_k - Ki *e; 
    //variante 1
    I_k = 0;

  }
  if(pwm > saturazione_max)
  {
    pwm = saturazione_max;
    //azione antiwindup: devo scaricare l'integrale dell'ultimo valore inserito
    //I_k = I_k - Ki *e;
    //variante 1
    I_k = 0;
  }

  //se ho saturazione devo scaricare l'integrale (Tecnica AntiWindUp)
  //double u_att = (pwm-q)/m;
  //double e_t = u_att - u;
  //double Tt = 1; //costante di tempo per scarico intefgrale
  //double AntiWindUp = e_t / Tt ;
  //I_k = I_k + e_t;

  
  return pwm;
}
double PIDController::map_control_2_radio(double u, double pwm_medium)
{

  double m = (saturazione_min - saturazione_max)/2;
  double q = pwm_medium;
  double y = m*u + q;

  if(y < saturazione_min)
    y = saturazione_min;
  if(y > saturazione_max)
    y = saturazione_max;
    
  
  cout << "PWM CALCOLATO: " << y << endl; 

  return y;
} 

void PIDController::set_Kp(double param)
{
  Kp = param;
}
void PIDController::set_b(double param)
{
  b = param;
}
void PIDController::set_Ki(double param)
{
  Ki = param;
}
void PIDController::set_Td(double param)
{
  Td = param;
}
void PIDController::set_Ky(double param)
{
  Ky = param;
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


double PIDController::get_Kp()
{
  return Kp;
}
double PIDController::get_b()
{
  return b;
}
double PIDController::get_Ki()
{
  return Ki;
}
double PIDController::get_Td()
{
  return Td;
}
double PIDController::get_Ky()
{
  return Ky;
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
  stato_takeoff = 0;
  hold_position_var = 0;
  init_pressure = 0;
  alt_from_barometer = 0;
  new_pose_recv = 0;
  secs_0 =ros::Time::now().toSec();
  std::string str_path  = "/home/robot/MCU_ArCaRa/log/quota.txt";
  FILE* fd;
  fd = fopen(str_path.c_str(), "w");
  fclose(fd);

  str_path  = "/home/robot/MCU_ArCaRa/log/control.txt";
  fd = fopen(str_path.c_str(), "w");
  fclose(fd);
  
  //inizializzo i controllori
  pid_controllers.roll.init_PID();
  pid_controllers.pitch.init_PID();
  pid_controllers.yaw.init_PID();
  pid_controllers.altitude.init_PID();

  //manual mode all'inizio abilitata
  manual_mode = true;

  marker_visibile = false;
	
   //..
	

  //inizializzo waypoint
  current_waypoint_world.position.x = 0;
  current_waypoint_world.position.y = 0;
  current_waypoint_world.position.z = 0;
  current_waypoint_world.orientation.x = 0;
  current_waypoint_world.orientation.y = 0;
  current_waypoint_world.orientation.z = 0;
  current_waypoint_world.orientation.w = 0;
      
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
  

  //1)-2)controllo di X-Y
  //devo ruotare [x_des_w, y_des_w] in body con RZ(yaw)
  double yaw = -P_world_body_world.orientation.z;
  double x_des_b = cos(yaw)*current_waypoint_world.position.x + sin(yaw)*current_waypoint_world.position.y;
  double y_des_b = -sin(yaw)*current_waypoint_world.position.x + cos(yaw)*current_waypoint_world.position.y; 
  double x_b = cos(yaw)*P_world_body_world.position.x + sin(yaw)*P_world_body_world.position.y;
  double y_b = -sin(yaw)*P_world_body_world.position.x + cos(yaw)*P_world_body_world.position.y;



  //1A) CONTROLLO DI ROLL
  //cout << "PID ROLL" << endl;
  double roll_command = pid_controllers.roll.update_PID(-y_b, -y_des_b, PWM_MEDIUM_ROLL);
  ROS_INFO("posizione y in body %f", y_b);
  ROS_INFO("posizione y  desiderata in body %f", y_des_b);
  ROS_INFO("pwm di rool %f", roll_command);
  cout <<"=====================================================================" << endl;
  //2A)CONTROLLO DI PITCH
  //cout << "PID PITCH" << endl;
  double pitch_command = pid_controllers.pitch.update_PID(x_b, x_des_b, PWM_MEDIUM_PITCH);
  ROS_INFO("posizione x in body %f", x_b);
  ROS_INFO("posizione x  desiderata in body %f", x_des_b);
  ROS_INFO("pwm di pitch %f", pitch_command);
  cout <<"=====================================================================" << endl;



  //3)controllo di HEADING
  double rz = P_world_body_world.orientation.z;
  double rz_des = current_waypoint_world.orientation.z;
  //calcolo il controllo da attuare
  double yaw_command = pid_controllers.yaw.update_PID(rz, rz_des, PWM_MEDIUM_YAW);
  //devo mappare l'ingresso in un comando ai servo

  ROS_INFO("heading corrrente %f", rz);
  ROS_INFO("heading desiderato %f", rz_des);
  ROS_INFO("pwm di yaw %f", yaw_command);
  cout <<"=====================================================================" << endl;

  //4)controllo di quota
  double alt_des = current_waypoint_world.position.z;
  double alt_curr = P_world_body_world.position.z;
  double throttle_command = pid_controllers.altitude.update_PID(alt_curr, alt_des, PWM_MEDIUM_THROTTLE);

  ROS_INFO("altezza corrrente %f", alt_curr);
  ROS_INFO("altezza desiderata %f", alt_des);
  ROS_INFO("pwm di throttle %f", throttle_command);
  cout <<"=====================================================================" << endl;

  //END)publish control to radio
  mavros_msgs::OverrideRCIn radio_pwm;
  
  /*if(stato_takeoff == 1)
  {    //radio_pwm.channels[RC_ROLL] = roll_command;
    //radio_pwm.channels[RC_PITCH] = pitch_command;
    radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
    radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
    radio_pwm.channels[RC_THROTTLE] = throttle_command;
    radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  }
  else
  {
    //se sono nella fase di takeoof, scelgo di venire su senza controllo di altro
    //radio_pwm.channels[RC_ROLL] = roll_command;
    //radio_pwm.channels[RC_PITCH] = pitch_command;
    radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
    radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
    radio_pwm.channels[RC_THROTTLE] = throttle_command;
    radio_pwm.channels[RC_YAW] = yaw_command;

  }*/
  //quando premo hold position faccio fare tutto in automatico
  if(hold_position_var)
  {
    radio_pwm.channels[RC_ROLL] = roll_command;
    radio_pwm.channels[RC_PITCH] = pitch_command;
    radio_pwm.channels[RC_THROTTLE] = throttle_command;
    radio_pwm.channels[RC_YAW] = yaw_command;
  }else
  {
    radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
    radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
    radio_pwm.channels[RC_THROTTLE] = throttle_command;
    radio_pwm.channels[RC_YAW] = yaw_command;
  }

  rc_pub.publish(radio_pwm);
  
  //scrittura su file
  if(true)
  {

      double secs = ros::Time::now().toSec();
      std::string str_path  = "/home/robot/MCU_ArCaRa/log/control.txt";
      FILE* fd;
      fd = fopen(str_path.c_str(), "a");
      fprintf(fd, "%f", secs -  secs_0);
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", P_world_body_world.position.x); //2
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", P_world_body_world.position.y); //3
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", P_world_body_world.position.z); //4 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", P_world_body_world.orientation.z); //8
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", current_waypoint_world.position.x); //9
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", current_waypoint_world.position.y); //10 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", current_waypoint_world.position.z); //11 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", current_waypoint_world.orientation.z); //12 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", x_b); //14 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", x_des_b); //15 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", y_b); //16 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", y_des_b); //17 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", roll_command); //18 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", pitch_command); //19
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", yaw_command); //20
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", throttle_command); //21 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%i\n", hold_position_var); //22
      fclose(fd); 


  }

     


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
  pid_controllers.roll.set_Kp(vect_pid[0]);
  pid_controllers.roll.set_b(vect_pid[1]);
  pid_controllers.roll.set_Ki(vect_pid[2]);
  pid_controllers.roll.set_Ky(vect_pid[3]);
  pid_controllers.roll.set_Td(vect_pid[4]);
  pid_controllers.roll.set_Nd(vect_pid[5]);
  pid_controllers.roll.set_saturazione_max(vect_pid[6]);
  pid_controllers.roll.set_saturazione_min(vect_pid[7]);
  //pitch
  pid_controllers.pitch.set_Kp(vect_pid[8]);
  pid_controllers.pitch.set_b(vect_pid[9]);
  pid_controllers.pitch.set_Ki(vect_pid[10]);
  pid_controllers.pitch.set_Ky(vect_pid[11]);
  pid_controllers.pitch.set_Td(vect_pid[12]);
  pid_controllers.pitch.set_Nd(vect_pid[13]);
  pid_controllers.pitch.set_saturazione_max(vect_pid[14]);
  pid_controllers.pitch.set_saturazione_min(vect_pid[15]);
  //yaw
  pid_controllers.yaw.set_Kp(vect_pid[16]);
  pid_controllers.yaw.set_b(vect_pid[17]);
  pid_controllers.yaw.set_Ki(vect_pid[18]);
  pid_controllers.yaw.set_Ky(vect_pid[19]);
  pid_controllers.yaw.set_Td(vect_pid[20]);
  pid_controllers.yaw.set_Nd(vect_pid[21]);
  pid_controllers.yaw.set_saturazione_max(vect_pid[22]);
  pid_controllers.yaw.set_saturazione_min(vect_pid[23]);
  //alt
  pid_controllers.altitude.set_Kp(vect_pid[24]);
  pid_controllers.altitude.set_b(vect_pid[25]);
  pid_controllers.altitude.set_Ki(vect_pid[26]);
  pid_controllers.altitude.set_Ky(vect_pid[27]);
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
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Kp());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_b());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Ki());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Ky());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Td());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_Nd());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_saturazione_max());
  fprintf(fd, "%lf\n", pid_controllers.roll.get_saturazione_min());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Kp());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_b());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Ki());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Ky());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Td());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_Nd());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_saturazione_max());
  fprintf(fd, "%lf\n", pid_controllers.pitch.get_saturazione_min());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Kp());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_b());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Ki());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Ky());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Td());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_Nd());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_saturazione_max());
  fprintf(fd, "%lf\n", pid_controllers.yaw.get_saturazione_min());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Kp());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_b());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Ki());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Ky());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Td());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_Nd());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_saturazione_max());
  fprintf(fd, "%lf\n", pid_controllers.altitude.get_saturazione_min());
  fclose(fd);

  ROS_INFO_STREAM("File pid saved successfully");
  return true;

}
 /********************************************************************************************/
/*                                                                                         */
/*    ROLL PITCH YAW PWM MEDIUM                                                            */
/*                                                                                         */
/*******************************************************************************************/
void warning_stop(double pwm_throttle)
{
    //rilascia gli stick di roll e pitch e yaw
    mavros_msgs::OverrideRCIn radio_pwm;
    radio_pwm.channels[RC_ROLL] = PWM_MEDIUM_ROLL;
    radio_pwm.channels[RC_PITCH] = PWM_MEDIUM_PITCH;
    radio_pwm.channels[RC_THROTTLE] = pwm_throttle; //TODO METTERE UN VALORE QUA
    radio_pwm.channels[RC_YAW] = PWM_MEDIUM_YAW;
    rc_pub.publish(radio_pwm);

}
 /********************************************************************************************/
/*                                                                                         */
/*    HOLD POSITION                                                                        */
/*                                                                                         */
/*******************************************************************************************/
void hold_position()
{
  cout << "HOLD POSITION" << endl;
  //il waypoint sarà la posizione corrente
  current_waypoint_world.position.x = P_world_body_world.position.x;
  current_waypoint_world.position.y = P_world_body_world.position.y;
  current_waypoint_world.position.z = P_world_body_world.position.z;
  current_waypoint_world.orientation.z = P_world_body_world.orientation.z;
  hold_position_var = 1;
  waypoint_recv = 1;
  manual_mode = 0;

  return;
}
 /********************************************************************************************/
/*                                                                                         */
/*    STEP TEST                                                                            */
/*                                                                                         */
/*******************************************************************************************/
void step_test()
{
  mavros_msgs::OverrideRCIn radio_pwm;

  //gradino di Pitch
  cout << "Gradino di -Pitch" << endl;
  //publish control to radio
  radio_pwm.channels[RC_ROLL] = PWM_MEDIUM_PITCH - 50;
  radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  rc_pub.publish(radio_pwm);
  sleep(5);

  cout << "No Step" << endl;
  //publish control to radio
  radio_pwm.channels[RC_ROLL] = NO_OVERRIDE ;
  radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  rc_pub.publish(radio_pwm);
  sleep(5);

  //gradino di Pitch
  cout << "Gradino di +Pitch" << endl;
  //publish control to radio
  radio_pwm.channels[RC_ROLL] = PWM_MEDIUM_PITCH + 50;
  radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  rc_pub.publish(radio_pwm);
  sleep(5);

  cout << "No Step" << endl;
  //publish control to radio
  radio_pwm.channels[RC_ROLL] = NO_OVERRIDE ;
  radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  rc_pub.publish(radio_pwm);
  sleep(5);
/****************************************************************************/
  //gradino di Roll
  cout << "Gradino di -Roll" << endl;
  //publish control to radio
  radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
  radio_pwm.channels[RC_PITCH] = PWM_MEDIUM_ROLL - 50;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  rc_pub.publish(radio_pwm);
  sleep(5);

  cout << "No Step" << endl;
  //publish control to radio
  radio_pwm.channels[RC_ROLL] = NO_OVERRIDE ;
  radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  rc_pub.publish(radio_pwm);
  sleep(5);

  //gradino di Roll
  cout << "Gradino di +Roll" << endl;
  //publish control to radio
  radio_pwm.channels[RC_ROLL] = NO_OVERRIDE;
  radio_pwm.channels[RC_PITCH] = PWM_MEDIUM_ROLL + 50;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  rc_pub.publish(radio_pwm);
  sleep(5);

  cout << "No Step" << endl;
  //publish control to radio
  radio_pwm.channels[RC_ROLL] = NO_OVERRIDE ;
  radio_pwm.channels[RC_PITCH] = NO_OVERRIDE;
  radio_pwm.channels[RC_THROTTLE] = NO_OVERRIDE;
  radio_pwm.channels[RC_YAW] = NO_OVERRIDE;
  rc_pub.publish(radio_pwm);
  sleep(5);







  return;
}