//=================================================================================================
//
//   In questro file si effettua la lettura di alcuni topic
//
//=================================================================================================
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
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

//numero di interi (32 bit) che ha il pacchetto che ricevo (52 +1 terminatore)
#define PAYLOAD_NBYTES   53
  

//++++++++++++++++++++
int start = 0;
std::string str;
double PI = 3.14159;
using std::cout;
using std::endl;
FILE *fd1 ;
timeval start_, stop_;
double elsapsed_time;
//++++++++++++++++++++

int
set_interface_attribs (int fd, int speed, int parity)
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

void
set_blocking (int fd, int should_block)
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
/*****************************************************************/
/*                                                               */
/*                 PRINT SU FILE                                  */
/*****************************************************************/
void printIMU(double time_,  double roll,double pitch,double yaw,double acc_x,double acc_y,double acc_z,double w_x,double w_y,double w_z,double mag_x,double mag_y,double mag_z)
{   
   
    fd1=fopen("iNemo_data.txt", "a");
    fprintf(fd1, "%f", time_);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", roll);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", pitch);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", yaw);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", acc_x);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", acc_y);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", acc_z);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", w_x);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", w_y);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", w_z);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", mag_x);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f", mag_y);
    fprintf(fd1, "%s", "  ");
    fprintf(fd1, "%f\n", mag_z);
    fclose(fd1);
}
/*****************************************************************/
/*                                                               */
/*                 DECODE PACKE                                  */
/*****************************************************************/
void decode_packet(std::queue<unsigned char> coda_seriale, double* roll,double* pitch,double* yaw,double* acc_x,double* acc_y,double* acc_z,double* w_x,double* w_y,double* w_z,double* mag_x,double* mag_y,double* mag_z)
{

int decode;
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*roll = (double)decode / 1000 * 180 /PI;

////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*pitch = (double)decode / 1000 * 180 /PI;
/////////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*yaw = (double)decode / 1000 * 180 /PI;

/////////////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*acc_x = (double)decode / 1000 ;

////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*acc_y = (double)decode / 1000 ;
/////////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*acc_z = (double)decode / 1000;
        
/////////////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*w_x = (double)decode / 1000 ;

////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*w_y = (double)decode / 1000 ;
/////////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*w_z = (double)decode / 1000;
/////////////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*mag_x = (double)decode / 1000 ;

////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*mag_y = (double)decode / 1000 ;
/////////////////////////////////////////////////
decode = (long int)coda_seriale.front();
coda_seriale.pop();
decode = decode | (0x0000FF00 &(((long int)coda_seriale.front())<<8));
coda_seriale.pop();
decode = decode | (0x00FF0000 &(((long int)coda_seriale.front())<<16));
coda_seriale.pop();
decode = decode | (0xFF000000 &(((long int)coda_seriale.front())<<24));
coda_seriale.pop();
        
*mag_z = (double)decode / 1000;

	       
//Svuoto gli ultimi 4 byte
coda_seriale.pop();
coda_seriale.pop();
coda_seriale.pop();
coda_seriale.pop();



}


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
ros::init(argc, argv, "ros_serial");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
ros::NodeHandle n;

std::queue<unsigned char> coda_seriale;

const char * portname = "/dev/ttyUSB0";

int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
if (fd < 0)
{
    printf("error %d opening %s: %s", errno, portname, strerror (errno));
    return -1;
}

set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
set_blocking (fd, 0);



fd1=fopen("iNemo_data.txt", "w");
fclose(fd1);

// ros::Rate loop_rate(100); // 100 Hz
 while(ros::ok())
{

    static double roll = 0;
    static double pitch = 0;
    static double yaw = 0;
    static double acc_x = 0;
    static double acc_y = 0;
    static double acc_z = 0;
    static double w_x = 0;
    static double w_y = 0;
    static double w_z = 0;
    static double mag_x = 0;
    static double mag_y = 0;
    static double mag_z = 0;
    static double duty = 0;


    int bytes = 0;
    int sum_bytes = 0;
    unsigned char buf[1024];
    
    // Read data from the COM-port
    bytes= read (fd, buf, sizeof buf);

    for(int a = 0 ; a < bytes ; a++)
    {
        coda_seriale.push(buf[a]);
    }


    //ricerca dell header
    unsigned char header[HEADER_BYTES+1] = {171, 172,0};

    unsigned char h1 = 0;
    unsigned char h2 = 0;
    if(start == 0){
        h1 = coda_seriale.front();
        while(h1 != header[0] && !coda_seriale.empty())
        {
            coda_seriale.pop();
            h1 = coda_seriale.front();
        }
        h2 = coda_seriale.front();
        while(h2 != header[1] && !coda_seriale.empty())
        {
            coda_seriale.pop();
            h2 = coda_seriale.front();
        }
         if(!coda_seriale.empty())
        {
            //ho trovato l'header 
            coda_seriale.pop();
            start = 1;
            
        }
    }

    if(start == 1 && coda_seriale.size() >= PAYLOAD_NBYTES)
    {
      //ho trovato l'header e ho tutto il pacchetto
      gettimeofday(&stop_, NULL);
	   elsapsed_time = (stop_.tv_sec - start_.tv_sec) * 1000;
      elsapsed_time += (stop_.tv_usec - start_.tv_usec) / 1000;
      cout << elsapsed_time << "ms.\n";
      double sec = ros::Time::now().toSec();//*1000000;
	    decode_packet( coda_seriale, &roll, &pitch, &yaw, &acc_x, &acc_y, &acc_z, &w_x, &w_y, &w_z, &mag_x, &mag_y, &mag_z);
      printIMU( sec, roll,pitch, yaw, acc_x, acc_y, acc_z, w_x, w_y, w_z, mag_x, mag_y, mag_z);
      /*cout << "PACCHETTO ARRIVATO" << endl;
      cout << "rool: " << roll << endl;  
      cout << "pitch: " << pitch << endl;  
      cout << "yaw: " << yaw << endl;  
      cout << "accx: " << acc_x << endl;  
      cout << "accy: " << acc_y << endl;  
      cout << "accz: " << acc_z << endl;  
      cout << "wx: " << w_x << endl;  
      cout << "wy: " << w_y << endl;  
      cout << "wz: " << w_z << endl;  
      cout << "magx: " << mag_x << endl;  
      cout << "magy: " << mag_y << endl; 
      cout << "magz: " << mag_z << endl; 
      cout << endl;*/
	    //resetto start
      start = 0;
 	    gettimeofday(&start_, NULL);
    }


    //loop_rate.sleep()
	}

  return 0;
}
