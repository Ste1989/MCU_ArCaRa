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
#include <aruco_mapping/ArucoMarker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
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
    PAYLOAD_4_7,
    PAYLOAD_4_8,
    PAYLOAD_4_9,
    PAYLOAD_4_10,
    PAYLOAD_4_11,
    PAYLOAD_4_12,
    PAYLOAD_4_13,
    PAYLOAD_4_14,
    PAYLOAD_4_15,
    PAYLOAD_4_16,
    PAYLOAD_4_17,
} waiting_msg;

//variabile per la memorizzazione dello stato della macchina a stati
waiting_msg state_msg;
/**************************************************************************************************/
/******************PACCHETTO DI WAYPOINT************************************************************/
/**************************************************************************************************/
#define NBYTES_PAYLOAD_WAYPOINT 17
#define PAYLOAD_WAYPOINT  (int)0xF0
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
#define PARAM_KP_YAW (int)0x40
#define PARAM_KI_YAW (int)0x41
#define PARAM_KD_YAW (int)0x42
#define PARAM_TS_YAW (int)0x43
#define PARAM_ND_YAW (int)0x44
#define PARAM_LUP_YAW (int)0x45
#define PARAM_LDOWN_YAW (int)0x46

/*possibili richieste di cambio parametri*****************************/
typedef enum{
    NO_PARAM,
    ALT_TAKEOFF, 

    KP_ROLL,
    KI_ROLL,
    KD_ROLL, 
    TS_ROLL, 
    ND_ROLL, 
    LUP_ROLL,  
    LDOWN_ROLL, 

    KP_PITCH, 
    KI_PITCH, 
    KD_PITCH,  
    TS_PITCH,  
    ND_PITCH,  
    LUP_PITCH,  
    LDOWN_PITCH,

    KP_YAW, 
    KI_YAW, 
    KD_YAW, 
    TS_YAW, 
    ND_YAW, 
    LUP_YAW, 
    LDOWN_YAW,
    
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
//ros topic request
ros::Publisher req_topic;
ros::Publisher param_topic;
ros::Publisher mode_topic;
ros::Publisher waypoint_topic;

/******************GLOBAL VAR***********************************************************************/
int count = 0;
double param = 0.0;
char new_packet = 0;
char new_packet_pose = 0;
int pose_el_time, ack_el_time;
double PI = 3.14159;
using std::cout;
using std::endl;
//strutture dati emporali
timeval new_pkt_time, current_time, ping_time, stream_pose_time;
double elapsed_time_pkt_received, elapsed_time_ping, elapsed_time_pose;
//struttura per la memorizzazione della posa della camera nel frame world
struct global_pose
{
    geometry_msgs::Point position;
    geometry_msgs::Point orientation;

};
global_pose global_camera_pose;
bool stream_pose;
char new_waypoint = 0;
geometry_msgs::Pose waypoint_recv;



/***************************FUNZIONI************************************************************************/
void parser_mess(unsigned char buffer);
double decode_payload();
void decode_packet();
void encode_payload(double payload);
void write_to_serial(int* serial);
void read_from_serial(int* serial);
void check_send_request();
void Status_Pixhawk_Callback(const std_msgs::Int32::ConstPtr& msg);
void Pose_cb(const aruco_mapping::ArucoMarker::ConstPtr& msg);
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
int serial_init(int* fd,const char* seriale_dev);
void quaternion_2_euler(double xquat, double yquat, double zquat, double wquat, double& roll, double& pitch, double& yaw);