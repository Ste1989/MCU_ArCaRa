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
    
}