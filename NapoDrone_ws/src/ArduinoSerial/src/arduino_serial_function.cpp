//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================
//A,RANGE.
//B,RANGE.
#include "arduino_serial.h"

/*****************************************************************/
/*                                                               */
/*                 PARSING DEL PACKET                            */
/*****************************************************************/
//funzione che riceve e decodifica i pacchetti in arrivo da PC
void parser_mess(unsigned char buffer)
{    sensor_msgs::Imu msg;
     //implementazione della macchina a stati
    switch(state_msg){
        case HEADER_1:

            if(buffer == HEADER_A)
            {
                state_msg=PAYLOAD_1_1;
                
            }else
            {
                state_msg=HEADER_1;
            }
            break;

        case PAYLOAD_1_1:
            switch(buffer)
            {
                case END_PACKET:
                    
                    range_recv[index_range] = decode_payload();
                    cout << range_recv[index_range] << endl;
                    msg.angular_velocity.x = range_recv[0];
                    msg.angular_velocity.y = range_recv[1];
                    msg.angular_velocity.z = range_recv[2];
                    msg.linear_acceleration.x = range_recv[3];
                    range_pub.publish(msg);

                    new_packet ++;
                    state_msg=HEADER_1;
                    index_range = 0;
                    break;

                case HEADER_A:
                
                    for(int i = 0; i< coda_recv_seriale.size(); i++)
                        coda_recv_seriale.pop();
                    index_range = 0;

                    break;

                case SEPARATORE:
                    
                    //ho ricevuto un range, lo devo codificare
                    range_recv[index_range] = decode_payload();
                    cout << range_recv[index_range] << endl;
                    index_range ++;
                    break;

                default:
                
                    coda_recv_seriale.push(buffer);
                    break;
            }
            break;
    }
      
}
/*****************************************************************/
/*                                                               */
/*                 service calib                             */
/*****************************************************************/
bool service_calib(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  cout << "OK!" << endl;
  //devo comunicare a arduino di fare autocalibrazione
  return true;
}
/*****************************************************************/
/*                                                               */
/*                 service_cb                               */
/*****************************************************************/
void service_cb(const std_msgs::Int16::ConstPtr& msg)
{
    if(msg->data == 1 && start_calibration == false)
    {
        //faccio una richiesta a arduino di calibrazione
        start_calibration = true;
        calibrazione();
    }
}
/*****************************************************************/
/*                                                               */
/*                 DECODE PAYLOAD                                */
/*****************************************************************/
int decode_payload()
{
    int a = 0;
    int n = coda_recv_seriale.size()-1 ;
    
    for(int i=0; i<n+1; i++)
    {   
        
        a = (int)(coda_recv_seriale.front()-'0') * pow(10,n-i) + a;
        coda_recv_seriale.pop();
    } 
    
    return a;
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
    //prova con for...
    /*if(coda_recv_seriale.size() > 0)
    {   cout << "dimensione coda: " << coda_recv_seriale.size() << endl;
        decode_packet();
    }*/
    //while(new_packet)
//    {
        //cout << "pacchetti da elaborare: " << new_packet << endl;
        //decode_payload();
  //  }



}

/*****************************************************************/
/*                                                               */
/*                 INIT GLOBL VAR                                */
/*****************************************************************/
void init_global_var()
{
    state_msg=HEADER_1;
    new_packet = 0;
    index_range = 0;
}
/*****************************************************************/
/*                                                               */
/*                 WRITE SERIALE                                 */
/*****************************************************************/
void write_to_serial(int* serial)
{   

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
    set_interface_attribs (*fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (*fd, 0);




    return 1;
}