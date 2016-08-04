//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================

#include "ros/ros.h"
#include "std_msgs/String.h"
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

//numero di Header che deve avere il pacchetto che ricevo
#define HEADER_BYTES  2
#define HEADER_A  (int)0xAB
#define HEADER_B  (int)0xAC
//numero di interi (32 bit) che ha il pacchetto che ricevo (2 nel progetto di avizzano)
#define PAYLOAD_NBYTES   2

typedef enum{
    HEADER_A_1,
    HEADER_A_2,
    PAYLOAD_1_1,
    PAYLOAD_1_2,
} waiting_msg;

//variabile per la memorizzazione dello stato della macchina a stati
waiting_msg state_msg;



/*************************************************************/
//
//modulo che preposto alla ricezione e alla decodifica dei messaggi
//ricevuti da PC  sulla seriale 2 (Xbee)
//
/************************************************************/
//
//
//
//1-comando contenente il comando di strat e stop
//     ______________________
//     | HEADER_A | FF | FE |
//     ______________________
//2-comando contenente il comando di strat e stop
//     ______________________
//     | HEADER_A | FF | FE |
//     ______________________






//header in ricezione
unsigned char header_recv[HEADER_BYTES+1] = {171, 172,0};
//header in invio
unsigned char header_send[HEADER_BYTES+1] = {255, 255,0};
//buffer di ricezione
std::queue<unsigned char> coda_seriale;
//bufferi di invio
std::queue<unsigned char> coda_send_seriale;
//++++++++++++++++++++
int start = 0;
std::string str;
double PI = 3.14159;
using std::cout;
using std::endl;
timeval start_, stop_;
double elsapsed_time;
//++++++++++++++++++++
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


int serial_init(int* fd)
{


    /* apro la porta seriale*/
    const char * portname = "/dev/ttyUSB0";

     *fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (*fd < 0)
    {
        printf("error %d opening %s: %s", errno, portname, strerror (errno));
        return -1;
    }
    /*imposto baud rate*/
    set_interface_attribs (*fd, B57600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (*fd, 0);

    return 1;
}
/*****************************************************************/
/*                                                               */
/*                 DECODE PACKET                                 */
/*****************************************************************/
void decode_packet(std::queue<unsigned char> coda_seriale, int* dato1,int* dato2)
{

int decode;
decode = (int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();

        
*dato1 = (int)decode;

////////////////////////////////////////////
decode = (int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();

        
*dato2 = (int)decode;



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
void write_to_serial(int* serial, unsigned char* index, int payload1, int payload2)
{


    //encode packet
    encode_packet(coda_send_seriale,payload1, payload2);
    //invio l'header
    write(*serial,index, 1);

    //invio il payload
    // for (int i = 7; i >= 0; i--)
    // {
    //     serial.Write(&payload[i], 1);
    // }
    for (int i = 0; i <= 7; i++)
    {
      //  write(*serial, &payload[i], 1);
    }

}
/*****************************************************************/
/*                                                               */
/*                 PARSING DEL PACKET                            */
/*****************************************************************/
//funzione che riceve e decodifica i pacchetti in arrivo da PC
void parser_mess(unsigned char buffer){


    cout << (int)buffer << endl;

    //implementazione della macchina a stati
    switch(state_msg){
        case HEADER_A_1:

            if(buffer == HEADER_A)
            {    cout << "ok" << endl;
                state_msg=HEADER_A_2;
            }
            break;

        case HEADER_A_2:
            if(buffer == HEADER_B)
            {
                state_msg=PAYLOAD_1_1;
                cout << "ok" << endl;
                //è stato riconosciuto un header-->è in arrivo un nuovo pacchetto
                //ma non è ancora stato ricevuto e decodificato tutto
            }
            break;

        case PAYLOAD_1_1:
            cout << "ok" << endl;
            state_msg=PAYLOAD_1_2;
            break;
        case PAYLOAD_1_2:
              cout << "PACCHETTO ARRIVATO" << endl;
            state_msg=HEADER_A_1;
            break;




    }

    return;
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

   // while(!coda_seriale.empty())
    //{
     //   unsigned char byte  = coda_seriale.front();
      //  parser_mess(byte);
       // coda_seriale.pop()
   // }





if( bytes < 0)
{
    for(int a = 0 ; a < bytes ; a++)
    {
        coda_seriale.push(buf[a]);
    }


    //ricerca dell header
    unsigned char h1 = 0;
    unsigned char h2 = 0;
    if(start == 0)
    {
        //devo trovare il primo header
        h1 = coda_seriale.front();
        cout << h1<< endl;

        while(h1 != header_recv[0] && !coda_seriale.empty())
        {

            coda_seriale.pop();
            h1 = coda_seriale.front();

        }


        cout << " qui2"<< endl;
        h2 = coda_seriale.front();
        cout << h2<< endl;
        while(h2 != header_recv[1] && !coda_seriale.empty())
        {   cout << " qui3"<< endl;
            coda_seriale.pop();
            h2 = coda_seriale.front();
        }
         if(!coda_seriale.empty())
        {   cout << " qui4"<< endl;
            //ho trovato l'header
            coda_seriale.pop();
            start = 1;

        }
    }

    //se ho trovato l'header e ho ricevuto un numero di bytes pari alla dimensione del pacchetto, lo decodifico
    if(start == 1 && coda_seriale.size() >= PAYLOAD_NBYTES)
    {
      cout << " HO RICEVUTO UN HEADER"<< endl;
      //ho trovato l'header e ho tutto il pacchetto
      //stampo il tempo di ogni quanto lo ricevo
      gettimeofday(&stop_, NULL);
       elsapsed_time = (stop_.tv_sec - start_.tv_sec) * 1000;
      elsapsed_time += (stop_.tv_usec - start_.tv_usec) / 1000;
      double sec = ros::Time::now().toSec();//*1000000;

      //decodifico il pacchetto
      //decode_packet( coda_seriale, &recv1, &recv2);

      //resetto start e tempo di ricezione
      start = 0;
      gettimeofday(&start_, NULL);




      cout << "PACCHETTO ARRIVATO" << endl;
      cout << elsapsed_time << "ms.\n";
      cout << (int)coda_seriale.front()<< endl;
      coda_seriale.pop();
      cout << (int)coda_seriale.front()<< endl;
      coda_seriale.pop();
      cout << endl;
    }
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

int serial;
// init della seriale
int result = serial_init(&serial);
//inizializzazone della macchina a stati con il primo stato
state_msg = HEADER_A_1;
if (result == 1)
{
    while(ros::ok())
   {
       read_from_serial(&serial);
       int dato1, dato2;
       //write_to_serial(&serial, header_send, dato1, dato2);

       //loop_rate.sleep()
   }

}
// ros::Rate loop_rate(100); // 100 Hz

  return 0;
}
