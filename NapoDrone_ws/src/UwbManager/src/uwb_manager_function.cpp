//=================================================================================================
//
//   nodo ROS per leggere e scrivere su seriale
//
//=================================================================================================

#include "uwb_manager.h"

/*************************************************************/
//
//modulo che preposto alla ricezione e alla decodifica dei messaggi
//ricevuti da uwb  sulla seriale 2 
//
/************************************************************/


/*****************************************************************/
/*                                                               */
/*                 PARSING DEL PACKET                            */
/*****************************************************************/
//funzione che riceve e decodifica i pacchetti in arrivo da PC
void parser_mess(unsigned char buffer){

    //implementazione della macchina a stati
    switch(state_msg){
        case HEADER_1:

            if(buffer == HEADER_A_UWB)
            {
                state_msg=HEADER_2;
            }else
            {
                state_msg=HEADER_1;
            }
            break;

        case HEADER_2:
            if(buffer == HEADER_B_UWB)
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
            if(buffer == PAYLOAD_UWB)
                state_msg=PAYLOAD_1_2;
            break;

            
        
        //arriva il range ancora 1 -byte 1
        case PAYLOAD_1_2:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_3;
            break;
   
            //arriva il range ancora 1 -byte 2
        case PAYLOAD_1_3:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_4;
            break;
  
            //arriva il range ancora 1 -byte 3
        case PAYLOAD_1_4:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_5;
            break;
    
            //arriva il range ancora 1 -byte 4
        case PAYLOAD_1_5:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_6;
            break;
      
      /////////////////////////////////////////////////////////////
        //arriva il range ancora 2 -byte 1
        case PAYLOAD_1_6:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_7;
            break;
   
            //arriva il range ancora 2 -byte 2
        case PAYLOAD_1_7:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_8;
            break;
  
            //arriva il range ancora 2 -byte 3
        case PAYLOAD_1_8:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_9;
            break;
    
            //arriva il range ancora 2 -byte 4
        case PAYLOAD_1_9:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_10;
            break;


      ///////////////////PAYLOAD_1_41,//////////////////////////////////////////
        //arriva il range ancora 3 -byte 1
        case PAYLOAD_1_10:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_11;
            break;
   
            //arriva il range ancora 3 -byte 2
        case PAYLOAD_1_11:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_12;
            break;
  
            //arriva il range ancora 3 -byte 3
        case PAYLOAD_1_12:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_13;
            break;
    
            //arriva il range ancora 3 -byte 4
        case PAYLOAD_1_13:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_14;
            break; 

        /////////////////////////////////////////////////////////////
        //arriva il range ancora 4 -byte 1
        case PAYLOAD_1_14:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_15;
            break;
   
            //arriva il range ancora 4 -byte 2
        case PAYLOAD_1_15:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_16;
            break;
  
            //arriva il range ancora 4 -byte 3
        case PAYLOAD_1_16:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_17;
            break;
    
            //arriva il range ancora 4 -byte 4
        case PAYLOAD_1_17:
            coda_recv_seriale.push(buffer);
            //state_msg = PAYLOAD_1_20;
            //notifico che è arrivato un nuovo pacchetto 
            new_packet ++;
            state_msg = HEADER_1;
            break; 


        ////////////////////////////NON UTILIZZATO/////////////////////////////////
        //arriva  x solution 1 byte 1
        case PAYLOAD_1_20:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_21;
            break;
   
            //arriva  x solution 1 byte 2
        case PAYLOAD_1_21:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_22;
            break;
  
            //arriva  x solution 1 byte 3
        case PAYLOAD_1_22:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_23;
            break;
    
            //arriva  x solution 1 byte 4
        case PAYLOAD_1_23:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_24;
            break; 
    /////////////////////////////////////////////////////////////
        //arriva  y solution 1 byte 1
        case PAYLOAD_1_24:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_25;
            break;
   
            //arriva  y solution 1 byte 2
        case PAYLOAD_1_25:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_26;
            break;
  
            //arriva  y solution 1 byte 3
        case PAYLOAD_1_26:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_27;
            break;
    
            //arriva  y solution 1 byte 4
        case PAYLOAD_1_27:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_28;
            break; 
        /////////////////////////////////////////////////////////////
        //arriva  z solution 1 byte 1
        case PAYLOAD_1_28:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_29;
            break;
   
            //arriva  z solution 1 byte 2
        case PAYLOAD_1_29:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_30;
            break;
  
            //arriva  z solution 1 byte 3
        case PAYLOAD_1_30:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_31;
            break;
    
            //arriva  z solution 1 byte 4
        case PAYLOAD_1_31:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_32;
            break; 
    /////////////////////////////////////////////////////////////
        //arriva  x solution 2 byte 1
        case PAYLOAD_1_32:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_33;
            break;
   
            //arriva  x solution 2 byte 2
        case PAYLOAD_1_33:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_34;
            break;
  
            //arriva  x sPAYLOAD_1_41,olution 2 byte 3
        case PAYLOAD_1_34:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_35;
            break;
    
            //arriva  x solution 2 byte 4
        case PAYLOAD_1_35:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_36;
            break; 
        /////////////////////////////////////////////////////////////
        //arriva  y solution 2 byte 1
        case PAYLOAD_1_36:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_37;
            break;
   
            //arriva  y solution 2 byte 2
        case PAYLOAD_1_37:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_38;
            break;
  
            //arriva  y solution 2 byte 3
        case PAYLOAD_1_38:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_39;
            break;
    
            //arriva  y solution 2 byte 4
        case PAYLOAD_1_39:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_40;
            break; 
        /////////////////////////////////////////////////////////////
        //arriva  z solution 2 byte 1
        case PAYLOAD_1_40:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_41;
            break;
   
            //arriva  z solution 2 byte 2
        case PAYLOAD_1_41:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_42;
            break;
  
            //arriva  z solution 2 byte 3
        case PAYLOAD_1_42:
            coda_recv_seriale.push(buffer);
            state_msg = PAYLOAD_1_43;
            break;
    
            //arriva  z solution 2 byte 4
        case PAYLOAD_1_43:
            coda_recv_seriale.push(buffer);
            //notifico che è arrivato un nuovo pacchetto 
            new_packet ++;
            cout << " qui no" << endl;
            state_msg = HEADER_1;
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

    double param_ = (double)decode / 1000;
    cout << "ricevuto payload : " << param_ << endl;
    return param_;
}
/*****************************************************************/
/*                                                               */
/*                 INIT GLOBAL V                                 */
/*****************************************************************/
void init_global_var()
{

    /*inizializzazione macchina a stati*/
    //inizializzazone della macchina a stati con il primo stato
    state_msg = HEADER_1;
    idx_msg_range = 0;
    new_packet = 0;
    log_range_uwb_path = "/home/sistema/MCU_ArCaRa/NapoDrone_ws/log/uwb_range.txt";
    if(enable_log)
    {
      //apro il file in scrittura
      file = fopen(log_range_uwb_path.c_str(), "w");
      fclose(file);
    }
    //per inizializzare secs_0 richiamo il client
    autopilot_manager::init_time srv_msg;
    bool res = get_time_sec0.call(srv_msg);

    if(res)
    {
        secs_0 = srv_msg.response.sec0;
    }
    else
    {   
        ROS_WARN("ATTENZIONE TEMPO NON INIZIALIZZATO CORRETTAMENTE");
        secs_0 = ros::Time::now().toSec();    
    }
    


}
/*****************************************************************/
/*                                                               */
/*                 DECODE PACKET                                 */
/*****************************************************************/
void decode_packet()
{
    //è arrivato un pacchetto contenente 
    //4 range
    //x-y-z soluzione 1
    //x-y-z soluzione 2
    range_uwb[0] = decode_payload();
    range_uwb[1] = decode_payload();
    range_uwb[2] = decode_payload();
    range_uwb[3] = decode_payload();
    //
    //trian_solution_1.x = decode_payload();
    //trian_solution_1.y = decode_payload();
    //trian_solution_1.z = decode_payload();
    //
    //trian_solution_2.x = decode_payload();
    //trian_solution_2.y = decode_payload();
    //trian_solution_2.z = decode_payload();
    new_packet --;
    idx_msg_range ++;
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
    while(new_packet)
    {

        cout << "pacchetti da elaborare: " << new_packet << endl;
        decode_packet();

        //pubblico su topic i valori ottenuti da pacchetto
        uwb_manager::RangeUwb msg;
        msg.header.seq = idx_msg_range;
        msg.header.stamp = ros::Time::now();
        msg.ancor1 = range_uwb[0];
        msg.ancor2 = range_uwb[1];
        msg.ancor3 = range_uwb[2];
        msg.ancor4 = range_uwb[3];
        uwb_topic.publish(msg);

        //scrittura su file 
        if(enable_log)
        {

            double secs = msg.header.stamp.sec  + ((double)(msg.header.stamp.nsec ))/1000000000; 
            file = fopen(log_range_uwb_path.c_str(), "a");
            fprintf(file, "%f", secs -  secs_0);
            fprintf(file, "%s", " ");
            fprintf(file, "%f", msg.ancor1); //2
            fprintf(file, "%s", " ");
            fprintf(file, "%f", msg.ancor2); //3
            fprintf(file, "%s", " ");
            fprintf(file, "%f", msg.ancor3); //4
            fprintf(file, "%s", " ");
            fprintf(file, "%f\n", msg.ancor4); //5
            fclose(file);


        }


        //calcolo tempo di controllo
        gettimeofday(&recv_time, NULL);
        elapsed_time_recv = (current_time.tv_sec - recv_time.tv_sec) * 1000;
        elapsed_time_recv += (current_time.tv_usec - recv_time.tv_usec) / 1000;
        cout << "freq :" << 1/(-elapsed_time_recv/1000) << endl;
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


        new_packet = 0;
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



    
    return 1;
}
