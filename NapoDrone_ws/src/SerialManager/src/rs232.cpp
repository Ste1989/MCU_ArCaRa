//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
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
//numero di Header che deve avere il pacchetto che ricevo
//1-comando contenente il comando di start
//     _________________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_CMD   | PAYLOAD_ARM       |
//     _________________________________________________________________
//2-comando contenente il comando di strat e stop
//     _______________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_CMD   | PAYLOAD_TAKEOFF   |
//     ________________________________________________________________
//     _______________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_CMD   | PAYLOAD_DISARM    |
//     ________________________________________________________________
//     _______________________________________________________________
//     | HEADER_CMD_A |HEADER_CMD_B|  PAYLOAD_CMD   | PAYLOAD_LAND    |
//     ________________________________________________________________
#define HEADER_BYTES  2
#define NBYTES_PAYLOAD_CMD 2
#define HEADER_CMD_A  (int)0xAB
#define HEADER_CMD_B (int)0x1B
#define PAYLOAD_CMD (int)0xF2
#define PAYLOAD_ARM (int)0x80
#define PAYLOAD_DISARM (int)0x81
#define PAYLOAD_TAKEOFF (int)0x82
#define PAYLOAD_LAND (int)0x83
#define PAYLOAD_RTL (int)0x84
#define PAYLOAD_EMERGENCYSTOP (int)0x85

int count = 0;
//numero di interi (32 bit) che ha il pacchetto che ricevo (2 nel progetto di avizzano)
#define PAYLOAD_NBYTES   2


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
    HEADER_1,
    HEADER_2,
    PAYLOAD_1_1,
    PAYLOAD_1_2,
} waiting_msg;

//variabile per la memorizzazione dello stato della macchina a stati
waiting_msg state_msg;
//buffer di ricezione
std::queue<unsigned char> coda_recv_seriale;
//bufferi di trasmissione
std::queue<unsigned char> coda_send_seriale;

//ros topic status
ros::Subscriber status_topic;

//ros topic request
ros::Publisher req_topic;
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
            state_msg=PAYLOAD_1_2;
            break;


        case PAYLOAD_1_2:
            coda_recv_seriale.push(buffer);
            //notifico che c'è un nuovo pacchetto da decodificare
            new_packet = 1;
            count ++;
            cout << "RICEVUI "<<count << endl;
            state_msg=HEADER_1;
            break;
    }

    return;
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
    return 1;
}
/*****************************************************************/
/*                                                               */
/*                 DECODE PACKET                                 */
/*****************************************************************/
void decode_packet()
{

    while(!coda_recv_seriale.empty())
    {
        if(coda_recv_seriale.front() == PAYLOAD_CMD && coda_recv_seriale.size() >= NBYTES_PAYLOAD_CMD)
        {
            coda_recv_seriale.pop();
            //è un pacchetto di comando, vedo che tipo di comando
            switch(coda_recv_seriale.front()){
                case PAYLOAD_ARM:
                    cmd_msg = ARM;
                    break;
                case PAYLOAD_DISARM:
                    cmd_msg = DISARM;
                    break;
                case PAYLOAD_TAKEOFF:
                    cmd_msg = TAKEOFF;
                    break;
                case PAYLOAD_LAND:
                    cmd_msg = LAND;
                    break;
                case PAYLOAD_RTL:
                    cmd_msg = RTL;
                    break;
                case PAYLOAD_EMERGENCYSTOP:
                    cmd_msg = EMERGENCY_STOP;
                    break;

            }

        }else
        {
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
    if(cmd_msg != NO_REQ && cmd_msg != cmd_msg_last)
    {
        //preparo la struttura dati
        std_msgs::Int32 msg;
        //riempio la struttura dati
        msg.data = cmd_msg;
        //pubblico sul topc
        req_topic.publish(msg);

        //preparo il pacchetto di ack da mandare su seriale
        coda_send_seriale.push('c');
        coda_send_seriale.push('m');
        coda_send_seriale.push('d');
        coda_send_seriale.push(' ');
        coda_send_seriale.push('a');
        coda_send_seriale.push('c');
        coda_send_seriale.push('k');

        cmd_msg_last = cmd_msg;
        //resetto cmd_msg
        cmd_msg = NO_REQ;

    }
}
/*****************************************************************/
/*                                                               */
/*                 CALLBACK PX4 STATUS                           */
/*****************************************************************/
void  Status_Pixhawk_Callback(const std_msgs::Int32::ConstPtr& msg)
{
    //routine che legge lo stato del drone

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
            break;
        case 4:
            current_status_px4 = ARMED;
            coda_send_seriale.push('A');
            coda_send_seriale.push('R');
            coda_send_seriale.push('M');
            coda_send_seriale.push('E');
            coda_send_seriale.push('D');
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
            break;
        case 6:
            current_status_px4 = LANDED;
            coda_send_seriale.push('L');
            coda_send_seriale.push('A');
            coda_send_seriale.push('N');
            coda_send_seriale.push('D');
            coda_send_seriale.push('E');
            coda_send_seriale.push('D');
            break;
        case 7:
            current_status_px4 = DISCONNECTED;
            coda_send_seriale.push('D');
            coda_send_seriale.push('I');
            coda_send_seriale.push('S');
            coda_send_seriale.push('C');
            coda_send_seriale.push('O');
            coda_send_seriale.push('N');
            break;
        case 8:
            current_status_px4 = HOVER;
            coda_send_seriale.push('H');
            coda_send_seriale.push('O');
            coda_send_seriale.push('V');
            coda_send_seriale.push('E');
            coda_send_seriale.push('R');
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
            break;
        case 10:
            current_status_px4 = RTL_STATUS;
            coda_send_seriale.push('R');
            coda_send_seriale.push('T');
            coda_send_seriale.push('L');
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
            break;

    }

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
    status_topic = n.subscribe<std_msgs::Int32>("napodrone/px4_status", 1000, Status_Pixhawk_Callback);

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
            cout << elapsed_time << "ms.\n";

            if(elapsed_time > 2000)
            {
                //comunicazione persa
                coda_send_seriale.push('c');
                coda_send_seriale.push('o');
                coda_send_seriale.push('o');
                coda_send_seriale.push('m');
                coda_send_seriale.push(' ');
                coda_send_seriale.push('l');
                coda_send_seriale.push('o');
                coda_send_seriale.push('s');
                coda_send_seriale.push('s');
            }


            //controllo se vi è una richiesta di comando
            check_send_request();

            //funzione per scrivere su seriale
           write_to_serial(&serial);


            //loop_rate.sleep();
        }

    }


  return 0;
}
