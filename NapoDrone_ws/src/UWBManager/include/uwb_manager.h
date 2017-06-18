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
//     _________________________________________________________________________________________________________
//     | HEADER_A |HEADER_B|  HEADER_POSE    | B | B | PAYLOAD_RANGE | PAYLOAD_SOLUTION_1 | PAYLOAD_SOLUTION_2 |
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
#define HEADER_POSE_UWB (int)0x0B







/************BUFFER DI RICEZIONE E TRASMISSIONE***************************************************/
//buffer di ricezione
std::queue<unsigned char> coda_recv_seriale;

/**********************TOPIC ROS********************************************************************/
//ros topic publisher
ros::Publisher uwb_topic;
/******************GLOBAL VAR***********************************************************************/
int new_packet = 0;

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

