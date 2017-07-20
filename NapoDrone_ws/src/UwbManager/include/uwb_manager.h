//=================================================================================================
//
//   nodo ROS per leggere  su seriale gli ultra wide band
//
//=================================================================================================
#include "ros/ros.h"
#include <errno.h>
#include <unistd.h>
#include <strings.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <queue>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "uwb_manager/RangeUwb.h"
#include "autopilot_manager/init_time.h"


using std::cout;
using std::endl;
/*************************************************************/
//
//modulo che preposto alla ricezione e alla decodifica dei messaggi
//ricevuti da UWB sulla seriale
//
/************************************************************/
// PACCHETTI IN RICEZIONE
//1-comandi

//     __________________________________________________________
//     | HEADER_A |HEADER_B|  HEADER_POSE    | PAYLOAD_RANGE x4 | 
//     _________________________________________________________

//     _________________________________________________________________________________________________________
//     | HEADER_A |HEADER_B|  HEADER_POSE    | PAYLOAD_RANGE | PAYLOAD_SOLUTION_1 | PAYLOAD_SOLUTION_2 |
//     _________________________________________________________________________________________________________


/*********************************************MACCHINA A STATI*************************************/
/**************************************************************************************************/
typedef enum{
    HEADER_1,
    HEADER_2,
    PAYLOAD_1_1,
    PAYLOAD_1_2,
    PAYLOAD_1_3,
    PAYLOAD_1_4,
    PAYLOAD_1_5,
    PAYLOAD_1_6,
    PAYLOAD_1_7,
    PAYLOAD_1_8,
    PAYLOAD_1_9,
    PAYLOAD_1_10,
    PAYLOAD_1_11,
    PAYLOAD_1_12,
    PAYLOAD_1_13,
    PAYLOAD_1_14,
    PAYLOAD_1_15,
    PAYLOAD_1_16,
    PAYLOAD_1_17,
    PAYLOAD_1_18,
    PAYLOAD_1_19,
    PAYLOAD_1_20,
    PAYLOAD_1_21,
    PAYLOAD_1_22,
    PAYLOAD_1_23,
    PAYLOAD_1_24,
    PAYLOAD_1_25,
    PAYLOAD_1_26,
    PAYLOAD_1_27,
    PAYLOAD_1_28,
    PAYLOAD_1_29,
    PAYLOAD_1_30,
    PAYLOAD_1_31,
    PAYLOAD_1_32,
    PAYLOAD_1_33,
    PAYLOAD_1_34,
    PAYLOAD_1_35,
    PAYLOAD_1_36,
    PAYLOAD_1_37,
    PAYLOAD_1_38,
    PAYLOAD_1_39,
    PAYLOAD_1_40,
    PAYLOAD_1_41,
    PAYLOAD_1_42,
    PAYLOAD_1_43,



} waiting_msg;

//variabile per la memorizzazione dello stato della macchina a stati
waiting_msg state_msg;
/**************************************************************************************************/
/******************PACCHETTO UWB*******************************************************************/
/**************************************************************************************************/

#define HEADER_A_UWB  (int)0x1A
#define HEADER_B_UWB  (int)0x1B
#define PAYLOAD_UWB  (int)0x2C








/************BUFFER DI RICEZIONE E TRASMISSIONE***************************************************/
//buffer di ricezione
std::queue<unsigned char> coda_recv_seriale;

/**********************TOPIC ROS********************************************************************/
//ros topic publisher
ros::Publisher uwb_topic;
//client
ros::ServiceClient get_time_sec0;
/******************GLOBAL VAR***********************************************************************/
int new_packet = 0;
int idx_msg_range = 0;
timeval current_time, recv_time;
double elapsed_time_recv;
bool enable_log ;
std::string log_range_uwb_path;
FILE* file;
double secs_0;
/*****************************************************************************************************/
double range_uwb[4];
struct trian_position{
	double x;
	double y;
	double z;

};
trian_position trian_solution_1, trian_solution_2;

/***************************FUNZIONI************************************************************************/
void parser_mess(unsigned char buffer);
double decode_payload();
void decode_packet();
void read_from_serial(int* serial);
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
int serial_init(int* fd,const char* seriale_dev);
void init_global_var();

