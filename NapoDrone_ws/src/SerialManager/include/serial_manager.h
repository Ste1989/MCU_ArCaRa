//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "serial_manager/Param.h"
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <queue>
#include <sys/time.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/BatteryStatus.h>
/*************************************************************/
//
//modulo che preposto alla ricezione e alla decodifica dei messaggi
//ricevuti da PC  sulla seriale 2 (Xbee)
//
/************************************************************/
// PACCHETTI IN RICEZIONE
//1-comandi
//     _________________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_CMD   | PAYLOAD x1 BYETS  |
//     __________________________________________________________________
//1-param
//     ___________________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_PARAM   | PAYLOAD x4        |
//     _____________________________________________#include <geometry_msgs/Point.h>______________________
//
//3-copter flight mode
//     ___________________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_MODE   | PAYLOAD x1        |
//     ___________________________________________________________________


// PACCHETTI IN TRASMISSIONE
//1-stato dell'autopilota
//     _________________________________________________________________
//     | HEADER_CMD_B |HEADER_CMD_A|  PAYLOAD_STATUS   | PAYLOAD x1 BYETS|
//     __________________________________________________________________
//
//2- ack comando
//     __________________________________________________________
//     | HEADER_CMD_B |HEADER_CMD_A|  PAYLOAD_CMD   | PAYLOAD_ACK|
//     __________________________________________________________
//
//3- ack param
//     _____________________________________________________________
//     | HEADER_CMD_B |HEADER_CMD_A|  PAYLOAD_PARAM   | PAYLOAD_ACK|
//     ____________________________________________________________
//
//4- ack flight mode
//     ____________________________________________________________
//     | HEADER_CMD_B |HEADER_CMD_A|  PAYLOAD_MODE   | PAYLOAD_ACK|
//     ____________________________________________________________
//5- pos packet
//     _____________________________________________________________________________________
//     | HEADER_CMD_B |HEADER_CMD_A|  PAYLOAD_POS   | X_POS x4 | Y_POS x4| Z_POS x4| YAWx4  |
//     _____________________________________________________________________________________
/*define bytes di header e ack*/
#define HEADER_BYTES  2
#define HEADER_A  (int)0xAB
#define HEADER_B (int)0x1B
#define PAYLOAD_PING (int)0x91
#define PAYLOAD_ACK (int)0x90
/**************************************************************************************************/
/*********************************************MACCHINA A STATI*************************************/
/**************************************************************************************************/
typedef enum{
    HEADER_1,
    HEADER_2,
    PAYLOAD_1_1,
    PAYLOAD_1_2,

    PAYLOAD_2_2,
    PAYLOAD_2_3,
    PAYLOAD_2_4,
    PAYLOAD_2_5,
    PAYLOAD_2_6,

    PAYLOAD_3_2,

    PAYLOAD_4_2,
    PAYLOAD_4_3,
    PAYLOAD_4_4,
    PAYLOAD_4_5,
    PAYLOAD_4_6,


    PAYLOAD_5_2,
} waiting_msg;

//variabile per la memorizzazione dello stato della macchina a stati
waiting_msg state_msg;
/**************************************************************************************************/
/******************PACCHETTO DI WAYPOINT************************************************************/
/**************************************************************************************************/
#define NBYTES_PAYLOAD_WAYPOINT 5 //17
#define PAYLOAD_WAYPOINT  (int)0xF0
//lista di waypoint
#define X_POINT (int)0x10
#define Y_POINT (int)0x11
#define Z_POINT (int)0x12
#define RZ_POINT (int)0x13
/*possibili punti *************************************/
typedef enum{
    NO_WAY,
    X_WAYP,
    Y_WAYP,
    Z_WAYP,
    RZ_WAYP,
} way_request;
//variabile per la memorizzazione dello richiesta effettuata
way_request way_msg;


/**************************************************************************************************/
/******************PACCHETTO DI COMANDO PINZA******************************************************/
/**************************************************************************************************/
#define NBYTES_PAYLOAD_GRIPPER 2
#define PAYLOAD_GRIPPER  (int)0xF4
//lista dei comandi
#define GRIPPER_CLOSE (int)0x80
#define GRIPPER_OPEN (int)0x81
#define GRIPPER_NO_REQ (int)0xFF
/*possibili richiesta di comandi**********************************/
typedef enum{
    NO_GRIPPER_REQ,
    CLOSE,
    OPEN,
} gripper_request;
//variabile per la memorizzazione della richiesta effettuata
gripper_request gripper_msg;

/**************************************************************************************************/
/******************PACCHETTO DI COMANDO************************************************************/
/**************************************************************************************************/
#define NBYTES_PAYLOAD_CMD 2
#define PAYLOAD_CMD (int)0xF2
//lista comandi
#define CMD_ARM (int)0x80
#define CMD_DISARM (int)0x81
#define CMD_TAKEOFF (int)0x82
#define CMD_LAND (int)0x83
#define CMD_RTL (int)0x84
#define CMD_EMERGENCYSTOP (int)0x85
#define CMD_CLEAR_RADIO_OVERRIDE (int)0x86
#define CMD_HOLD_POSITION (int)0x87
#define CMD_SEND_WAYPOINT (int)0x88
#define CMD_NO_REQ (int)0xFF
/*possibili richiesta di comandi**********************************/
typedef enum{
    NO_REQ,
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    RTL,
    EMERGENCY_STOP,
    CLEARRADIOOVERRIDE,
    HOLD_POSITION,
} cmd_request;
//variabile per la memorizzazione dello richiesta effettuata
cmd_request cmd_msg;
cmd_request cmd_msg_last;


/**************************************************************************************************/
/******************PACCHETTO PARAMETRI************************************************************/
/**************************************************************************************************/
#define NBYTES_PAYLOAD_PARAM 5
#define PAYLOAD_PARAM (int)0xF3
//lista parametri che posso inviare
#define PARAM_ALT_TAKEOFF (int)0x10
#define PARAM_KP_ROLL (int)0x20
#define PARAM_B_ROLL (int)0x21
#define PARAM_KI_ROLL (int)0x22
#define PARAM_KY_ROLL (int)0x23
#define PARAM_TD_ROLL (int)0x24
#define PARAM_ND_ROLL (int)0x25
#define PARAM_LUP_ROLL (int)0x26
#define PARAM_LDOWN_ROLL (int)0x27
#define PARAM_KP_PITCH (int)0x30
#define PARAM_B_PITCH (int)0x31
#define PARAM_KI_PITCH (int)0x32
#define PARAM_KY_PITCH (int)0x33
#define PARAM_TD_PITCH (int)0x34
#define PARAM_ND_PITCH (int)0x35
#define PARAM_LUP_PITCH (int)0x36
#define PARAM_LDOWN_PITCH (int)0x37
#define PARAM_KP_YAW (int)0x40
#define PARAM_B_YAW (int)0x41
#define PARAM_KI_YAW (int)0x42
#define PARAM_KY_YAW (int)0x43
#define PARAM_TD_YAW (int)0x44
#define PARAM_ND_YAW (int)0x45
#define PARAM_LUP_YAW (int)0x46
#define PARAM_LDOWN_YAW (int)0x47
#define PARAM_KP_ALT (int)0x50
#define PARAM_B_ALT (int)0x51
#define PARAM_KI_ALT (int)0x52
#define PARAM_KY_ALT (int)0x53
#define PARAM_TD_ALT (int)0x54
#define PARAM_ND_ALT (int)0x55
#define PARAM_LUP_ALT (int)0x56
#define PARAM_LDOWN_ALT (int)0x57

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
param_request param_msg;

/**************************************************************************************************/
/******************PACCHETTO FLIGHT MODE**********************************************************/
/**************************************************************************************************/

#define NBYTES_PAYLOAD_MODE 2
#define PAYLOAD_MODE (int)0xF1
//lista parametri che posso inviare
#define MODE_STABILIZE (int)0x50
#define MODE_ALT_HOLD (int)0x51
#define MODE_LOITER (int)0x52
#define MODE_AUTO (int)0x53
#define MODE_ACRO (int)0x54
#define MODE_SPORT (int)0x55
#define MODE_DRIFT (int)0x56
#define MODE_GUIDED (int)0x57
#define MODE_CIRCLE (int)0x58
#define MODE_POS_HOLD (int)0x59
#define MODE_BRAKE (int)0x5A
#define MODE_FOLLOW_ME (int)0x5B
#define MODE_SIMPLE_SUPER (int)0x5C
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
//variabile per la memorizzazione dello richiesta effettuata
mode_request mode_msg;


/**************************************************************************************************/
/******************STATO AUTOPILOTA**********************************************************/
/**************************************************************************************************/
#define PAYLOAD_PX4 (int)0xE0
#define PAYLOAD_POSE (int)0xE1
#define PAYLOAD_BATTERY (int)0xE2
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
//dichiaro la variabile globale che mantiene lo stato dell'autopilota
status_px4 current_status_px4;



/************BUFFER DI RICEZIONE E TRASMISSIONE***************************************************/
//buffer di ricezione
std::queue<unsigned char> coda_recv_seriale;
//bufferi di trasmissione
std::queue<unsigned char> coda_send_seriale;

/**********************TOPIC ROS********************************************************************/
//ros Subscriber
ros::Subscriber status_topic;
ros::Subscriber pose_topic;
ros::Subscriber battery_topic;
//ros topic request
ros::Publisher req_topic;
ros::Publisher param_topic;
ros::Publisher mode_topic;
ros::Publisher waypoint_topic;
ros::Publisher gripper_topic;

/******************GLOBAL VAR***********************************************************************/
int count = 0;
double param = 0.0;
double waypoint_data = 0.0;
int new_packet = 0;
char new_packet_pose = 0;
char new_packet_battery = 0;
int pose_el_time, ack_el_time,battery_el_time;
double PI = 3.14159;
using std::cout;
using std::endl;
//strutture dati emporali
timeval new_pkt_time, current_time, ping_time, stream_pose_time, stream_battery_time;
double elapsed_time_pkt_received, elapsed_time_ping, elapsed_time_pose, elapsed_time_battery;
//struttura per la memorizzazione della posa della camera nel frame world
struct NapodronePose
{
    geometry_msgs::Point position;
    geometry_msgs::Point orientation;

};
NapodronePose P_world_body_world;

bool stream_pose;
char new_waypoint = 0;
geometry_msgs::Pose waypoint_recv;

/*batteria*/
mavros_msgs::BatteryStatus battery_status;



/***************************FUNZIONI************************************************************************/
void parser_mess(unsigned char buffer);
double decode_payload();
void decode_packet();
void encode_payload(double payload);
void write_to_serial(int* serial);
void read_from_serial(int* serial);
void check_send_request();
void Status_Pixhawk_Callback(const std_msgs::Int32::ConstPtr& msg);
void Pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void Battery_cb(const mavros_msgs::BatteryStatus::ConstPtr& msg);
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
int serial_init(int* fd,const char* seriale_dev);
void quaternion_2_euler(double xquat, double yquat, double zquat, double wquat, double& roll, double& pitch, double& yaw);
