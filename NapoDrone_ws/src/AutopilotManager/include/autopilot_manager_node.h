
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include "std_msgs/Int32.h"
#include <mavros_msgs/StreamRate.h>
#include <sensor_msgs/FluidPressure.h>
#include <stdio.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <serial_manager/Param.h>
#include <sys/time.h>


using namespace std;

//Il gradiente barico verticale è legato alla variazione di pressione con l'aumentare o il diminuire della quota.
// La sua variazione è fissata in 27 piedi (8,23 m) per ogni hPa (o millibar) 
//di differenza di pressione in aria standard
//quindi la proporzione da impostare sarà:
//8.23 : 100 = x : variazione_registrata

//stato del drone
mavros_msgs::State current_state;
//pressione 
double init_pressure;
double current_pressure;
double alt_from_barometer; 

//cvarianili per la gestione del tempo
timeval  current_time, control_time, pose_time;
double elapsed_time_control, elapsed_time_pose;
char waypoint_recv;
std::string PID_file;
bool init_takeoff;
std::string init_flight_mode;
int loop_rate;
int stream_rate;
bool marker_visibile;
bool manual_mode;
char new_pose_recv;
double secs_0;
//altezza di takeoff da raggiungere
double alt_takeoff_target;
//struttura per la memorizzazione della posa della camera nel frame world
struct NapodronePose
{
    geometry_msgs::Point position;
    geometry_msgs::Point orientation;

};
NapodronePose P_world_body_world;

//memorizzo waypoint
geometry_msgs::Pose current_waypoint_world;

//ros topic subscriber
ros::Subscriber state_sub;
ros::Subscriber cmd_sub;
ros::Subscriber mode_sub;
ros::Subscriber param_sub;
ros::Subscriber pressure_sub;
ros::Subscriber aruco_poses_sub;
ros::Subscriber waypoint_sub;
//ros topic publisher
ros::Publisher rc_pub;
ros::Publisher state_pub;
//ros topic service
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient set_stream_rate_client;


//RC_MAP è la mappa dei segnali radio
//[0] : ROOL
//[1] : PITCH
//[2] : THRLOTTE
//[3] : YAW
//[4]
#define RC_YAW 3
#define RC_ROLL 0
#define RC_PITCH 1
#define RC_THROTTLE 2
#define PWM_LOW_LIMIT_ROLL 1118
#define PWM_HIGH_LIMIT_ROLL 1918
#define PWM_MEDIUM_ROLL 1522
#define PWM_LOW_LIMIT_PITCH 1118
#define PWM_HIGH_LIMIT_PITCH 1918
#define PWM_MEDIUM_PITCH 1518
#define PWM_LOW_LIMIT_YAW 1118
#define PWM_HIGH_LIMIT_YAW 1918
#define PWM_MEDIUM_YAW 1514
#define PWM_LOW_LIMIT_THROTTLE 1118
#define PWM_HIGH_LIMIT_THROTTLE 1918
#define PWM_MEDIUM_THROTTLE 1516
#define NO_OVERRIDE 0



//ENUMERATORI
/*possibili stati autopilota *************************************/
typedef enum{
    CONNECTING,
    CONNECTED,
    ARMABLE,
    NOT_ARMABLE,
    ARMED,
    TAKE_OFF,
    LANDED,
    DISCONNECTED,
    HOVER,
    LANDING,
    RTL_STATUS,
    EMERGENCY_STOP_STATUS,
}status_px4;


/*possibili richieste di flight mode *************************************/
typedef enum{
    NO_MODE,
    STABILIZE,
    ALT_HOLD,
    LOITER,
    AUTO,
    ACRO,
    SPORT,
    DRIFT,
    GUIDED,
    CIRCLE,
    POS_HOLD,
    BRAKE,
    FOLLOW_ME,
    SIMPLE_SUPER,
} mode_request;

mode_request current_mode_req;
/*possibili richiesta di comandi**********************************/
typedef enum{
    NO_REQ,
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    RTL,
    EMERGENCY_STOP,
    CLEAR_RADIO_OVERRIDE,
    HOLD_POSITION,
} cmd_request;

cmd_request current_cmd_req;
/*possibili richieste di cambio parametri*****************************/
typedef enum{
    NO_PARAM,
    ALT_TAKEOFF, 

    KP_ROLL,
    B_ROLL,
    KI_ROLL, 
    KY_ROLL, 
    TD_ROLL, 
    ND_ROLL, 
    LUP_ROLL,  
    LDOWN_ROLL, 

    KP_PITCH,
    B_PITCH,
    KI_PITCH, 
    KY_PITCH, 
    TD_PITCH, 
    ND_PITCH, 
    LUP_PITCH,  
    LDOWN_PITCH, 

    KP_YAW,
    B_YAW,
    KI_YAW, 
    KY_YAW, 
    TD_YAW, 
    ND_YAW, 
    LUP_YAW,  
    LDOWN_YAW, 

    KP_ALT,
    B_ALT,
    KI_ALT, 
    KY_ALT, 
    TD_ALT, 
    ND_ALT, 
    LUP_ALT,  
    LDOWN_ALT,
} param_request;


/***********************************************************************************************/
/*                                                                                             */
/*                PID                                                                          */
/*                                                                                             */
/***********************************************************************************************/
class PIDController {
  private:
    //parametri del controllore
    double Kp;
    double b;
    double Ki;
    double Td;
    double Ky;
    double Nd;
    double saturazione_max;
    double saturazione_min;
    //parametri che memorizzano lo stato del controllore
    double I_k;
    double y_k;
    double D_k;
    
  public:
    PIDController();
    void init_PID();
    double update_PID(double y, double y_des, double pwm_medium);
    double map_control_2_radio(double u, double pwm_medium);
    void set_Kp(double param);
    void set_b(double param);
    void set_Ki(double param);
    void set_Td(double param);
    void set_Ky(double param);
    void set_Nd(double param);
    void set_saturazione_max(double param);
    void set_saturazione_min(double param);
    double get_Kp();
    double get_b();
    double get_Ki();
    double get_Td();
    double get_Ky();
    double get_Nd();
    double get_saturazione_max();
    double get_saturazione_min();

    
};

struct Controllers 
{
  PIDController roll;
  PIDController pitch;
  PIDController yaw;
  //PIDController velocity_x;
  //PIDController velocity_y;
  PIDController altitude;
} pid_controllers;


//FUNZIONI////////////////////////////////////////////////////////////////////////////////////
void init_global_variables();
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr& msg);
void cmd_cb(const std_msgs::Int32::ConstPtr& msg);
void mode_cb(const std_msgs::Int32::ConstPtr& msg);
void param_cb(const serial_manager::Param::ConstPtr& msg);
void waypoint_cb(const geometry_msgs::Pose::ConstPtr& msg);
void poses_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
bool arm_vehicle();
bool disarm_vehicle();
bool takeoff_vehicle();
void clear_radio_override();
void quaternion_2_euler(double xquat, double yquat, double zquat, double wquat, double& roll, double& pitch, double& yaw);
void update_control();
bool leggi_PID_file(std::string PID_file);
bool scrivi_PID_file(std::string PID_file);
void warning_stop(double pwm_throttle);
void hold_position();
void step_test();

