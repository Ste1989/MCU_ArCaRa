//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================

#include "bluetooth_manager.h"

/*************************************************************/
//
//  Ricezione della posa da ekf
//
/************************************************************/


void ekf_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //salvo la stima precedente arrivata
    xt_ = xt;
    yt_ = yt;
    zt_ = zt;
    //salvo la stima attuale
    xt = msg->pose.position.x;
    yt = msg->pose.position.y;
    zt = msg->pose.position.z;

    //devo prendere la stima arrivata in precedenza e calcolare il treno di delta
    double delta_x = (xt - xt_)/num_campioni_delta;
    double delta_y = (yt - yt_)/num_campioni_delta;
    double delta_z = (zt - zt_)/num_campioni_delta;

    double x = xt_;
    double y = yt_;
    double z = zt_;

    for(int i = 0; i < num_campioni_delta-1; i++)
    {
        x = x + delta_x;
        y = y + delta_y;
        z = z + delta_z;

        coda_position_x.push(x);
        coda_position_y.push(y);
        coda_position_z.push(z);
    }
   
    //inserisco la misura vera
    coda_position_x.push(xt);
    coda_position_y.push(yt);
    coda_position_z.push(zt);


}


/*****************************************************************/
/*                                                               */
/*                 INIT GLOBL VAR                                */
/*****************************************************************/
void init_global_var()
{
    xt_ = 0;
    yt_ = 0;
    zt_ = 0;
    //salvo la stima attuale
    xt = 0;
    yt = 0;
    zt = 0;

}
/*****************************************************************/
/*                                                               */
/*                 WRITE SERIALE                                 */
/*****************************************************************/
void write_to_serial(int* serial)
{
    if(!coda_position_x.empty())
    {

        //devo convertire in double e poi inviare come stringa
        int x = (int)(coda_position_x.front()*1000);
        coda_position_x.pop();
        int y = (int)(coda_position_y.front()*1000);
        coda_position_y.pop();
        int z = (int)(coda_position_z.front()*1000);
        coda_position_z.pop();
        double x1 = ((double)x)/1000.0;
        double y1 = ((double)y)/1000.0;
        double z1 = ((double)z)/1000.0;
        std::stringstream ss;
        ss << x1;
        std::string str = ss.str();
        unsigned char* uc = (unsigned char*)str.c_str();
        int n_written = write( *serial, uc, sizeof(uc) -1 );
        //write(*serial,&coda_send_seriale.front(), 1);
        //coda_send_seriale.pop();
    //}

    //    std::cout << "sono qui" << std::endl;
    }

    
    

    
   

    //    std::cout << "sono qui" << std::endl;
    

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
    set_interface_attribs (*fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (*fd, 0);




    return 1;
}