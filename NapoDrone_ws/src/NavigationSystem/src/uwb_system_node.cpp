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

    int freq_filter;
    //leggo i parametri dal lauch file
    nh.param<bool>("/UWBSystem_Node/log_file", log_file, 1);
    nh.param<int>("/UWBSystem_Node/freq_filter", freq_filter, 100);
    //coordinate delle ancore
    //anchor0 X-Y-Z
    nh.param<double>("/UWBSystem_Node/anchor0_X", anchor0[0], 0.0);
    nh.param<double>("/UWBSystem_Node/anchor0_Y", anchor0[1], 0.0);
    nh.param<double>("/UWBSystem_Node/anchor0_Z", anchor0[2], 0.0);
    //anchor1 X-Y-Z
    nh.param<double>("/UWBSystem_Node/anchor1_X", anchor1[0], 0.0);
    nh.param<double>("/UWBSystem_Node/anchor1_Y", anchor1[1], 0.0);
    nh.param<double>("/UWBSystem_Node/anchor1_Z", anchor1[2], 0.0);
    //anchor2 X-Y-Z
    nh.param<double>("/UWBSystem_Node/anchor2_X", anchor2[0], 0.0);
    nh.param<double>("/UWBSystem_Node/anchor2_Y", anchor2[1], 0.0);
    nh.param<double>("/UWBSystem_Node/anchor2_Z", anchor2[2], 0.0);
    //anchor3 X-Y-Z
    nh.param<double>("/UWBSystem_Node/anchor3_X", anchor3[0], 0.0);
    nh.param<double>("/UWBSystem_Node/anchor3_Y", anchor3[1], 0.0);
    nh.param<double>("/UWBSystem_Node/anchor3_Z", anchor3[2], 0.0);
    //sottoscrizione al topic dei range
    rangeUWB_sub = nh.subscribe<uwb_manager::RangeUwb>("/uwb/range", 1, rangeUWB_cb);
    
    //service client
    get_time_sec0 = nh.serviceClient<autopilot_manager::init_time>("/get_time_t0");
    
    init_global_var();
    
    double time_ms = (1000.0/freq_filter);
    //frequenza a cui far girare il nodo
    ros::Rate loop_rate(100);
    gettimeofday(&filter_time, NULL);
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
        
        //A)leggo i topic
        ros::spinOnce();

        //EKF_solo_range
        if(elapsed_time_filter >= time_ms)
        {
            if(elapsed_time_filter > time_ms)
                ROS_WARN("FREQUENZA FILTRO %f ", 1/(elapsed_time_filter/1000));
         

            //sottocampiono il segnale dei range
            double range0 = sum_range_dt.anchor0/new_range_packet;
            double range1 = sum_range_dt.anchor1/new_range_packet;
            double range2 = sum_range_dt.anchor2/new_range_packet;
            double range3 = sum_range_dt.anchor3/new_range_packet;

            //update filtro
            EKF_solo_range(range0,range1,range2,range3, time_ms/1000);


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

