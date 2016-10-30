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
#include  <sys/time.h>

/*************************************************************/
//
//modulo che preposto alla ricezione e alla decodifica dei messaggi
//ricevuti da PC  sulla seriale 2 (Xbee)
//
/************************************************************/
//1-comandi
//     _________________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_CMD   | PAYLOAD x1 BYETS  |
//     __________________________________________________________________
//1-param
//     ___________________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_PARAM   | PAYLOAD x4        |
//     ___________________________________________________________________
//3-copter guided mode
//     ___________________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_MODE   | PAYLOAD x1        |
//     ___________________________________________________________________

#define HEADER_BYTES  2
#define HEADER_CMD_A  (int)0xAB
#define HEADER_CMD_B (int)0x1B

#define NBYTES_PAYLOAD_CMD 2
#define PAYLOAD_CMD (int)0xF2
//lista comandi
#define PAYLOAD_ARM (int)0x80
#define PAYLOAD_DISARM (int)0x81
#define PAYLOAD_TAKEOFF (int)0x82
#define PAYLOAD_LAND (int)0x83
#define PAYLOAD_RTL (int)0x84
#define PAYLOAD_EMERGENCYSTOP (int)0x85

#define NBYTES_PAYLOAD_PARAM 5
#define PAYLOAD_PARAM (int)0xF3
//lista parametri che posso inviare
#define PARAM_ALT_TAKEOFF (int)0x10




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



int count = 0;
double param = 0.0;

//dichiaro i possibili stati dell'autopilota
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

typedef enum{
    NO_REQ,
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    RTL,
    EMERGENCY_STOP,
} cmd_request;
//variabile per la memorizzazione dello richiesta effettuata
cmd_request cmd_msg;
cmd_request cmd_msg_last;

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



typedef enum{
    NO_PARAM,
    ALT_TAKEOFF,
} param_request;
param_request param_msg;

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
} waiting_msg;

//variabile per la memorizzazione dello stato della macchina a stati
waiting_msg state_msg;
//buffer di ricezione
std::queue<unsigned char> coda_recv_seriale;
//bufferi di trasmissione
std::queue<unsigned char> coda_send_seriale;

//ros Subscriber
ros::Subscriber status_topic;


//ros topic request
ros::Publisher req_topic;
ros::Publisher param_topic;
ros::Publisher mode_topic;
//++++++++++++++++++++
char new_packet = 0;

double PI = 3.14159;
using std::cout;
using std::endl;
//strutture dati emporali
timeval time_1, time_2;
double elapsed_time;

/*****************************************************************/
/*                                                               */
/*                 PARSING DEL PACKET                            */
/*****************************************************************/
//funzione che riceve e decodifica i pacchetti in arrivo da PC
void parser_mess(unsigned char buffer){


    //implementazione della macchina a stati
    switch(state_msg){
        case HEADER_1:

            if(buffer == HEADER_CMD_A)
            {
                state_msg=HEADER_2;
            }else
            {
                state_msg=HEADER_1;
            }
            break;

        case HEADER_2:
            if(buffer == HEADER_CMD_B)
            {
                state_msg=PAYLOAD_1_1;
                //è stato riconosciuto un header-->è in arrivo un nuovo pacchetto
                //ma non è ancora stato ricevuto e decodificato tutto
            }
            else
            {
                state_msg=HEADER_1;
            }
            break;

        case PAYLOAD_1_1:
            coda_recv_seriale.push(buffer);
            if(buffer == PAYLOAD_CMD)
                state_msg=PAYLOAD_1_2;
            else if(buffer == PAYLOAD_PARAM)
                state_msg=PAYLOAD_2_2;
            else if(buffer == PAYLOAD_MODE)
                state_msg = PAYLOAD_3_2;
            break;


            /*********************************************************/
            //PCCHETTO CONTENTENTE UN COMANDO
        case PAYLOAD_1_2:
            coda_recv_seriale.push(buffer);
            //notifico che c'è un nuovo pacchetto da decodificare
            new_packet = 1;
            state_msg=HEADER_1;
            break;

            /*********************************************************/
            //PCCHETTO CONTENTENTE UN PARAMETRO (INT 32)
        case PAYLOAD_2_2:
            coda_recv_seriale.push(buffer);
            state_msg=PAYLOAD_2_3;
            break;

        case PAYLOAD_2_3:
            coda_recv_seriale.push(buffer);
            state_msg=PAYLOAD_2_4;
            break;

        case PAYLOAD_2_4:
            coda_recv_seriale.push(buffer);
            state_msg=PAYLOAD_2_5;
            break;

        case PAYLOAD_2_5:
            coda_recv_seriale.push(buffer);
            state_msg=PAYLOAD_2_6;
            break;

        case PAYLOAD_2_6:
            coda_recv_seriale.push(buffer);
            new_packet = 1;
            state_msg=HEADER_1;
            break;
            /*********************************************************/
            //PCCHETTO CONTENTENTE UN MODO
        case PAYLOAD_3_2:
            coda_recv_seriale.push(buffer);
            //notifico che c'è un nuovo pacchetto da decodificare
            new_packet = 1;
            state_msg=HEADER_1;
            break;



    }

    return;
}

/*****************************************************************/
/*                                                               */
/*                 DECODE PAYLOAD                                */
/*****************************************************************/
double decode_payload()
{
    //in coda_recv_seriale ho 4 bytes da decodificare
    int decode;
    decode = (long int)coda_recv_seriale.front();
    coda_recv_seriale.pop();
    decode = decode | (0x0000FF00 &(((long int)coda_recv_seriale.front())<<8));
    coda_recv_seriale.pop();
    decode = decode | (0x00FF0000 &(((long int)coda_recv_seriale.front())<<16));
    coda_recv_seriale.pop();
    decode = decode | (0xFF000000 &(((long int)coda_recv_seriale.front())<<24));
    coda_recv_seriale.pop();

    double param_ = (double)decode / 100;
    cout << "ricevuto parametro : " << param_ << endl;
    return param_;
}

/*****************************************************************/
/*                                                               */
/*                 DECODE PACKET                                 */
/*****************************************************************/
void decode_packet()
{
 label:
    while(!coda_recv_seriale.empty())
    {

        if(coda_recv_seriale.front() == PAYLOAD_CMD && coda_recv_seriale.size() >= NBYTES_PAYLOAD_CMD)
        {
            coda_recv_seriale.pop();
            //è un pacchetto di comando, vedo che tipo di comando
            switch(coda_recv_seriale.front()){
                case PAYLOAD_ARM:
                    cmd_msg = ARM;
                    coda_recv_seriale.pop();
                    break;
                case PAYLOAD_DISARM:
                    cmd_msg = DISARM;
                    coda_recv_seriale.pop();
                    break;
                case PAYLOAD_TAKEOFF:
                    cmd_msg = TAKEOFF;
                    coda_recv_seriale.pop();
                    break;
                case PAYLOAD_LAND:
                    cmd_msg = LAND;
                    coda_recv_seriale.pop();
                    break;
                case PAYLOAD_RTL:
                    cmd_msg = RTL;
                    coda_recv_seriale.pop();
                    break;
                case PAYLOAD_EMERGENCYSTOP:
                    cmd_msg = EMERGENCY_STOP;
                    coda_recv_seriale.pop();
                    break;

            }
            goto label;
        }

        if(coda_recv_seriale.front() == PAYLOAD_PARAM && coda_recv_seriale.size() >= NBYTES_PAYLOAD_PARAM)
        {
            coda_recv_seriale.pop();
            //in coda_recv_seriale ho 4 bytes che devono essere convertiti in un intero
            //il primo bytes mi dice che parametro si tratta
            switch(coda_recv_seriale.front())
            {
                case PARAM_ALT_TAKEOFF:

                    coda_recv_seriale.pop();
                    param = decode_payload();
                    param_msg = ALT_TAKEOFF;
                    break;

            }
            goto label;
        }

        if(coda_recv_seriale.front() == PAYLOAD_MODE && coda_recv_seriale.size() >= NBYTES_PAYLOAD_MODE)
        {
            coda_recv_seriale.pop();
            switch(coda_recv_seriale.front())
            {
                case MODE_ACRO:
                    mode_msg = ACRO;
                    coda_recv_seriale.pop();
                    cout << "ACRO" << endl;
                    break;
                case MODE_ALT_HOLD:
                    mode_msg = ALT_HOLD;
                    coda_recv_seriale.pop();
                    cout << "ALT HOLD" << endl;
                    break;
                case MODE_AUTO:
                    mode_msg = AUTO;
                    coda_recv_seriale.pop();
                    cout << "AUTO" << endl;
                    break;
                case MODE_BRAKE:
                    mode_msg = BRAKE;
                    coda_recv_seriale.pop();
                    cout << "BRAKE" << endl;
                    break;
                case MODE_CIRCLE:
                    mode_msg = CIRCLE;
                    coda_recv_seriale.pop();
                    cout << "CIRCLE" << endl;
                    break;
                case MODE_DRIFT:
                    mode_msg = DRIFT;
                    coda_recv_seriale.pop();
                    cout << "DRIFT" << endl;
                    break;
                case MODE_FOLLOW_ME:
                    mode_msg = FOLLOW_ME;
                    coda_recv_seriale.pop();
                    cout << "FOLLOW ME" << endl;
                    break;
                case MODE_GUIDED:
                    mode_msg = GUIDED;
                    coda_recv_seriale.pop();
                    cout << "GUIDED" << endl;
                    break;
                case MODE_LOITER:
                    mode_msg = LOITER;
                    coda_recv_seriale.pop();
                    cout << "LOITER" << endl;
                    break;
                case MODE_POS_HOLD:
                    mode_msg = POS_HOLD;
                    coda_recv_seriale.pop();
                    cout << "POS HOLD" << endl;
                    break;
                case MODE_SIMPLE_SUPER:
                    mode_msg = SIMPLE_SUPER;
                    coda_recv_seriale.pop();
                    cout << "SIMPLE SUP" << endl;
                    break;
                case MODE_SPORT:
                    mode_msg = SPORT;
                    coda_recv_seriale.pop();
                    cout << "SPORT" << endl;
                    break;
                case MODE_STABILIZE:
                    mode_msg = STABILIZE;
                    coda_recv_seriale.pop();
                    cout << "STAB" << endl;
                    break;

            }
            goto label;

        }
        if(coda_recv_seriale.size() > 0)
        {
            cout << "PACCHETTO NON RICONOSCIUTO" << endl;
            coda_recv_seriale.pop();
        }






    }


    //pacchetto è stato analizzato: resetto new_packet
    new_packet = 0;
}
/*****************************************************************/
/*                                                               */
/*                  ENCODE PACKET                                */
/*****************************************************************/
void encode_packet(std::queue<unsigned char> coda_seriale,int payload1, int payload2)
{

//vedere dalla tesi delle macchinine come fare



}

/*****************************************************************/
/*                                                               */
/*                 WRITE SERIALE                                 */
/*****************************************************************/
void write_to_serial(int* serial)
{

    while( !coda_send_seriale.empty() )
    {
        write(*serial,&coda_send_seriale.front(), 1);
        coda_send_seriale.pop();
    }




}

/*****************************************************************/
/*                                                               */
/*                 READ SERIALE                                  */
/*****************************************************************/
void read_from_serial(int* serial)
{

    static int recv1 = 0;
    static int recv2 = 0;
    int bytes = 0;
    int sum_bytes = 0;
    unsigned char buf[1024];

    // Read data from the COM-port
    bytes= read(*serial, buf, sizeof buf);

    for(int a = 0 ; a < bytes ; a++)
    {
        parser_mess(buf[a]);
    }

    if(new_packet == 1)
    {
        decode_packet();
        //aggiorno il tempo
        gettimeofday(&time_1, NULL);
    }


}
/*****************************************************************/
/*                                                               */
/*                 CHECK_SEND_REQUEST                            */
/*****************************************************************/
void check_send_request()
{
    /*COMANDI*********************************************/
    if(cmd_msg != NO_REQ && cmd_msg != cmd_msg_last)
    {
        //preparo la struttura dati
        std_msgs::Int32 msg;
        //riempio la struttura dati
        msg.data = cmd_msg;
        //pubblico sul topc
        req_topic.publish(msg);

        //preparo il pacchetto di ack da mandare su seriale
        /*coda_send_seriale.push('C');
        coda_send_seriale.push('M');
        coda_send_seriale.push('D');
        coda_send_seriale.push(' ');
        coda_send_seriale.push('A');
        coda_send_seriale.push('C');
        coda_send_seriale.push('K');
        coda_send_seriale.push('.');*/

        cmd_msg_last = cmd_msg;
        //resetto cmd_msg
        cmd_msg = NO_REQ;

    }

    /********PARAMETRI**********************************************/
    if(param_msg != NO_PARAM)
    {
        //preparo la struttura dati
        serial_manager::Param msg;
        //riempio la struttura dati
        msg.header = 1;
        msg.param = param;
        //pubblico sul topc
        param_topic.publish(msg);

        param_msg = NO_PARAM;

    }
     /********MODO GUIDA**********************************************/
    if(mode_msg != NO_MODE)
    {
        //preparo la struttura dati
        std_msgs::Int32 msg;
        //riempio la struttura dati
        msg.data = mode_msg;
        //pubblico sul topc
        mode_topic.publish(msg);

        mode_msg = NO_MODE;

    }
}
/*****************************************************************/
/*                                                               */
/*                 CALLBACK PX4 STATUS                           */
/*****************************************************************/
void  Status_Pixhawk_Callback(const std_msgs::Int32::ConstPtr& msg)
{
    //routine che legge lo stato del drone
    cout << "ricevuto" << endl;
    switch(msg->data){
        case 0:
            current_status_px4 = CONNECTING;
            //preparo il pacchetto di ack da mandare su seriale
            coda_send_seriale.push('C');
            coda_send_seriale.push('O');
            coda_send_seriale.push('N');
            coda_send_seriale.push('N');
            coda_send_seriale.push('E');
            coda_send_seriale.push('C');
            coda_send_seriale.push('T');
            coda_send_seriale.push('I');
            coda_send_seriale.push('N');
            coda_send_seriale.push('G');
            coda_send_seriale.push('\n');
            break;
        case 1:
            current_status_px4 = CONNECTED;
            //preparo il pacchetto di ack da mandare su seriale
            coda_send_seriale.push('C');
            coda_send_seriale.push('O');
            coda_send_seriale.push('N');
            coda_send_seriale.push('N');
            coda_send_seriale.push('E');
            coda_send_seriale.push('C');
            coda_send_seriale.push('T');
            coda_send_seriale.push('E');
            coda_send_seriale.push('D');
            coda_send_seriale.push('\n');
            break;
        case 2:
            current_status_px4 = ARMABLE;
            coda_send_seriale.push('A');
            coda_send_seriale.push('R');
            coda_send_seriale.push('M');
            coda_send_seriale.push('A');
            coda_send_seriale.push('B');
            coda_send_seriale.push('L');
            coda_send_seriale.push('E');
            coda_send_seriale.push('\n');
            break;
        case 3:
            current_status_px4 = NOT_ARMABLE;
            coda_send_seriale.push('N');
            coda_send_seriale.push('O');
            coda_send_seriale.push('T');
            coda_send_seriale.push(' ');
            coda_send_seriale.push('A');
            coda_send_seriale.push('R');
            coda_send_seriale.push('M');
            coda_send_seriale.push('\n');
            break;
        case 4:
            current_status_px4 = ARMED;
            coda_send_seriale.push('A');
            coda_send_seriale.push('R');
            coda_send_seriale.push('M');
            coda_send_seriale.push('E');
            coda_send_seriale.push('D');
            coda_send_seriale.push('\n');
            break;
        case 5:
            current_status_px4 = TAKE_OFF;
            coda_send_seriale.push('T');
            coda_send_seriale.push('A');
            coda_send_seriale.push('K');
            coda_send_seriale.push('E');
            coda_send_seriale.push(' ');
            coda_send_seriale.push('O');
            coda_send_seriale.push('F');
            coda_send_seriale.push('F');
            coda_send_seriale.push('\n');
            break;
        case 6:
            current_status_px4 = LANDED;
            coda_send_seriale.push('L');
            coda_send_seriale.push('A');
            coda_send_seriale.push('N');
            coda_send_seriale.push('D');
            coda_send_seriale.push('E');
            coda_send_seriale.push('D');
            coda_send_seriale.push('\n');
            break;
        case 7:
            current_status_px4 = DISCONNECTED;
            coda_send_seriale.push('D');
            coda_send_seriale.push('I');
            coda_send_seriale.push('S');
            coda_send_seriale.push('C');
            coda_send_seriale.push('O');
            coda_send_seriale.push('N');
            coda_send_seriale.push('\n');
            break;
        case 8:
            current_status_px4 = HOVER;
            coda_send_seriale.push('H');
            coda_send_seriale.push('O');
            coda_send_seriale.push('V');
            coda_send_seriale.push('E');
            coda_send_seriale.push('R');
            coda_send_seriale.push('\n');
            break;
        case 9:
            current_status_px4 = LANDING;
            coda_send_seriale.push('L');
            coda_send_seriale.push('A');
            coda_send_seriale.push('N');
            coda_send_seriale.push('D');
            coda_send_seriale.push('I');
            coda_send_seriale.push('N');
            coda_send_seriale.push('G');
            coda_send_seriale.push('\n');
            break;
        case 10:
            current_status_px4 = RTL_STATUS;
            coda_send_seriale.push('R');
            coda_send_seriale.push('T');
            coda_send_seriale.push('L');
            coda_send_seriale.push('\n');
            break;
        case 11:
            current_status_px4 = EMERGENCY_STOP_STATUS;
            coda_send_seriale.push('E');
            coda_send_seriale.push('M');
            coda_send_seriale.push('E');
            coda_send_seriale.push('R');
            coda_send_seriale.push(' ');
            coda_send_seriale.push('S');
            coda_send_seriale.push('T');
            coda_send_seriale.push('O');
            coda_send_seriale.push('P');
            coda_send_seriale.push('\n');
            break;

    }

}
/*****************************************************************/
/*                                                               */
/*                 INIT SERIALE                                  */
/*****************************************************************/
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        printf("error %d from tcgetattr", errno) ;
        //error_message("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}


int serial_init(int* fd,const char* seriale_dev)
{


    /* apro la porta seriale*/
    const char * portname = seriale_dev;

     *fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (*fd < 0)
    {
        printf("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }
    /*imposto baud rate*/
    set_interface_attribs (*fd, B57600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (*fd, 0);


    /*inizializzazione macchina a stati*/
    //inizializzazone della macchina a stati con il primo stato
    state_msg = HEADER_1;
    //nessun nuovo pacchetto
    new_packet = 0;
    //nessua richiesta di comando
    cmd_msg = NO_REQ;
    cmd_msg_last = NO_REQ;
    //nessun parametro da inviare
    param_msg = NO_PARAM;

    return 1;
}
/*****************************************************************/
/*                                                               */
/*                 MAIN                                          */
/*****************************************************************/
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
  */
    ros::init(argc, argv, "Serial_Manager");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
    ros::NodeHandle n;
    req_topic = n.advertise<std_msgs::Int32>("napodrone/cmd_request", 1);
    param_topic = n.advertise<serial_manager::Param>("napodrone/param_request", 1);
    mode_topic = n.advertise<std_msgs::Int32>("napodrone/mode_request", 1);
    status_topic = n.subscribe<std_msgs::Int32>("napodrone/px4_status",10, &Status_Pixhawk_Callback);

    //leggo i parametri specificati nel launch file
    std::string seriale_dev;
    n.param<std::string>("/SerialManager/dev", seriale_dev, "/dev/ttyUSB0");


    int serial;
    // init della seriale
    int result = serial_init(&serial, seriale_dev.c_str());
    //inizializzo time_1
    gettimeofday(&time_1, NULL);
    //ros::Rate loop_rate(100); // 100 Hz
    if (result == 1)
    {
        while(ros::ok())
        {
            read_from_serial(&serial);

            //leggo il tempo e calcolo quanto è passato dall'ultimo pacchetto ricevuto
            gettimeofday(&time_2, NULL);
            elapsed_time = (time_2.tv_sec - time_1.tv_sec) * 1000;
            elapsed_time += (time_2.tv_usec - time_1.tv_usec) / 1000;
 /*           cout << elapsed_time << "ms.\n";

           if(elapsed_time > 2000)
            {
                //comunicazione persa
                coda_send_seriale.push('C');
                coda_send_seriale.push('O');
                coda_send_seriale.push('M');
                coda_send_seriale.push(' ');
                coda_send_seriale.push('L');
                coda_send_seriale.push('O');
                coda_send_seriale.push('S');
                coda_send_seriale.push('T');
                coda_send_seriale.push('.');
            }*/


            //controllo se vi è una richiesta di comando
            check_send_request();


            //vedi se arrivato qualcosa sulle callback
            ros::spinOnce();

            //funzione per scrivere su seriale
           write_to_serial(&serial);


            //loop_rate.sleep();
        }

    }


  return 0;
}
