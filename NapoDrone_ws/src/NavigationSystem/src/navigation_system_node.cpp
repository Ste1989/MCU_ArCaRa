
#include "navigation_system_node.h"

/********************************************************************************************/
/*                                                                                         */
/*    MAIN                                                                                 */
/*                                                                                         */
/*******************************************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    //enable callback
    bool enable_imu, enable_mag, enable_attitude;
    //leggo i parametri dal lauch file
     nh.param<bool>("/NavigationSystem/imu_enable", enable_imu, false);
     nh.param<bool>("/NavigationSystem/mag_enable", enable_mag, false);
     nh.param<bool>("/NavigationSystem/attitude_enable", enable_attitude, false);
     nh.param<int>("/NavigationSystem/log_file", log_file, 0);

    //imu del drone
    if(enable_imu)
        imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 1, imu_cb);
    //magnetometro del drone
    if(enable_mag)
        mag_sub = nh.subscribe<sensor_msgs::MagneticField>("/mavros/imu/mag", 1, mag_cb);
    //assetto del drone
    if(enable_attitude)
        attitude_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, attitude_cb);
    
    //service client
    get_time_sec0 = nh.serviceClient<autopilot_manager::init_time>("/get_time_t0");
    
    init_global_var();
    
    while(ros::ok())
    {
       
        //A)leggo i topic
        ros::spinOnce();
        

    }


    return 0;
}

