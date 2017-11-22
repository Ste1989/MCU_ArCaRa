#include "uwb_system_node.h"

/********************************************************************************************/
/*                                                                                         */
/*    MAIN                                                                                 */
/*                                                                                         */
/*******************************************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "uwb_system_node");
    ros::NodeHandle nh;

    
    //leggo i parametri dal lauch file
    nh.param<bool>("/UWBSystem_Node/debug", debug, false);
    nh.param<bool>("/UWBSystem_Node/log_file", log_file, false);
    nh.param<int>("/UWBSystem_Node/freq_filter", freq_filter, 100);
    //coordinate delle ancore
    //anchor0 X-Y-Z
    nh.param<double>("/UWBSystem_Node/anchor0_X", anchor0(0), 0.0);
    nh.param<double>("/UWBSystem_Node/anchor0_Y", anchor0(1), 0.0);
    nh.param<double>("/UWBSystem_Node/anchor0_Z", anchor0(2), 0.0);
    //anchor1 X-Y-Z
    nh.param<double>("/UWBSystem_Node/anchor1_X", anchor1(0), 0.0);
    nh.param<double>("/UWBSystem_Node/anchor1_Y", anchor1(1), 0.0);
    nh.param<double>("/UWBSystem_Node/anchor1_Z", anchor1(2), 0.0);
    //anchor2 X-Y-Z
    nh.param<double>("/UWBSystem_Node/anchor2_X", anchor2(0), 0.0);
    nh.param<double>("/UWBSystem_Node/anchor2_Y", anchor2(1), 0.0);
    nh.param<double>("/UWBSystem_Node/anchor2_Z", anchor2(2), 0.0);
    //anchor3 X-Y-Z
    nh.param<double>("/UWBSystem_Node/anchor3_X", anchor3(0), 0.0);
    nh.param<double>("/UWBSystem_Node/anchor3_Y", anchor3(1), 0.0);
    nh.param<double>("/UWBSystem_Node/anchor3_Z", anchor3(2), 0.0);

    R = MatrixXd::Zero(4,4);
    P = MatrixXd::Zero(6,6);
    Q = MatrixXd::Zero(6,6);
    K = MatrixXd::Zero(6,4);
    //R
    nh.param<double>("/UWBSystem_Node/R1", R(0,0), 0.0);
    nh.param<double>("/UWBSystem_Node/R2", R(1,1), 0.0);
    nh.param<double>("/UWBSystem_Node/R3", R(2,2), 0.0);
    nh.param<double>("/UWBSystem_Node/R4", R(3,3), 0.0);
    //P
    nh.param<double>("/UWBSystem_Node/P1", P(0,0), 0.0);
    nh.param<double>("/UWBSystem_Node/P2", P(1,1), 0.0);
    nh.param<double>("/UWBSystem_Node/P3", P(2,2), 0.0);
    nh.param<double>("/UWBSystem_Node/P4", P(3,3), 0.0);
    nh.param<double>("/UWBSystem_Node/P5", P(4,4), 0.0);
    nh.param<double>("/UWBSystem_Node/P6", P(5,5), 0.0);
    //Q
    nh.param<double>("/UWBSystem_Node/Q1", Q(0,0), 0.0);
    nh.param<double>("/UWBSystem_Node/Q2", Q(1,1), 0.0);
    nh.param<double>("/UWBSystem_Node/Q3", Q(2,2), 0.0);
    nh.param<double>("/UWBSystem_Node/Q4", Q(3,3), 0.0);
    nh.param<double>("/UWBSystem_Node/Q5", Q(4,4), 0.0);
    nh.param<double>("/UWBSystem_Node/Q6", Q(5,5), 0.0);

    //sottoscrizione al topic dei range
    rangeUWB_sub = nh.subscribe<uwb_manager::RangeUwb>("/uwb/range", 1, rangeUWB_cb);
    
    //service client
    get_time_sec0 = nh.serviceClient<autopilot_manager::init_time>("/get_time_t0");
    
    //init variabili globali
    init_global_var();
    

    //se debug è uguale true leggo il file uwb_range.txt
   
    if(debug)
    {
        leggi_file_debug();
    
    }
    timeval time_0;
    //frequenza a cui far girare il nodo
    ros::Rate loop_rate(100);
    gettimeofday(&filter_time, NULL);
    if(debug)
    {
        gettimeofday(&time_0, NULL);
        
    }
    while(ros::ok())
    {
        //loop rate
        //loop_rate.sleep();
        //calolo tempo attuale
        gettimeofday(&current_time, NULL);
        //calcolo tempo di controllo
        elapsed_time_filter = (current_time.tv_sec - filter_time.tv_sec) * 1000;
        elapsed_time_filter += (current_time.tv_usec - filter_time.tv_usec) / 1000;
        //elapsed_time_filter = 500 ---> 0.5 s
        
      
        cout << (current_time.tv_sec-time_0.tv_sec) << endl;// + current_time.tv_usec /1000 << endl;

        //A)leggo i topic
        ros::spinOnce();

        //EKF_solo_range
        if(elapsed_time_filter >= time_ms)
        {
            if(elapsed_time_filter > time_ms)
                ROS_WARN("FREQUENZA FILTRO %f ", 1/(elapsed_time_filter/1000));
         

            //sottocampiono il segnale dei range
            VectorXd range(4);
            range(0) = sum_range_dt.anchor0/new_range_packet;
            range(1) = sum_range_dt.anchor1/new_range_packet;
            range(2) = sum_range_dt.anchor2/new_range_packet;
            range(3) = sum_range_dt.anchor3/new_range_packet;

            //update filtro
            VectorXd position_estimated(3);
            EKF_solo_range(range,  dt_filter, position_estimated);

            //resetto elapsed_time_filter
            gettimeofday(&filter_time, NULL);
            //resetto variabili globali
            new_range_packet = 0;
            sum_range_dt.anchor0 = 0;
            sum_range_dt.anchor1 = 0;
            sum_range_dt.anchor2 = 0;
            sum_range_dt.anchor3 = 0;
        }

        
        

    }


    return 0;
}

