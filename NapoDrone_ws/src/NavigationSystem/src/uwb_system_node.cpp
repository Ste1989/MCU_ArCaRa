
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
    nh.param<bool>("/UWBSystem_Node/log_file", log_file, 1);
    nh.param<int>("/UWBSystem_Node/freq_ros_node", freq_ros_node, 100);

    //sottoscrizione al topic dei range
    rangeUWB_sub = nh.subscribe<uwb_manager::RangeUwb>("/uwb/range", 1, rangeUWB_cb);
    
    //service client
    get_time_sec0 = nh.serviceClient<autopilot_manager::init_time>("/get_time_t0");
    
    init_global_var();
    

    //frequenza a cui far girare il nodo
    ros::Rate loop_rate(freq_ros_node);

    while(ros::ok())
    {
       

        //A)leggo i topic
        ros::spinOnce();

        //EKF_solo_range


        //loop rate
        loop_rate.sleep();
        

    }


    return 0;
}

