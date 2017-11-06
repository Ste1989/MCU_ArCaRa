#include "uwb_system_node.h"

/******************************************************************************************/
/*                                                                                        */
/*                  INIT                                                                  */
/*                                                                                        */
/******************************************************************************************/
void init_global_var()
{

  //new packet from the callback
  new_range_packet = 0;


  //tempo 0
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

  //path log file
  if(log_file)
  { 
    log_uwb_path  = "/home/robot/MCU_ArCaRa/NapoDrone_ws/log/EKF_soloRange.txt";
    fd = fopen(log_uwb_path.c_str(), "w");
    fclose(fd);
  }

}
/******************************************************************************************/
/*                                                                                        */
/*                  RANGE CALLBACK                                                        */
/*                                                                                        */
/******************************************************************************************/
void rangeUWB_cb(const uwb_manager::RangeUwb::ConstPtr& msg)
{
  //imu_state = *msg;
  new_range_packet = 1;


  
  //gettimeofday(&current_time, NULL);
  //elapsed_time_imu = (current_time.tv_sec - imu_time.tv_sec) * 1000;
  //elapsed_time_imu += (current_time.tv_usec - imu_time.tv_usec) / 1000;
  //cout << "freq IMU:" << 1/(elapsed_time_imu/1000) << endl;
  //gettimeofday(&imu_time, NULL);

  //cout << "***********************************"<< endl;
  
}


 