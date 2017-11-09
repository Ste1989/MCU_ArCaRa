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

  //init
  sum_range_dt.anchor0 = 0.0;
  sum_range_dt.anchor1 = 0.0;
  sum_range_dt.anchor2 = 0.0;
  sum_range_dt.anchor3 = 0.0;

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
  uwb_manager::RangeUwb app = *msg;
  //rimozione eventuali outlier
  if(abs(app.anchor0) > 0.05 && abs(app.anchor1) > 0.05 && abs(app.anchor2) > 0.05 && abs(app.anchor3) > 0.05  )
  { 
    sum_range_dt.anchor0 = sum_range_dt.anchor0 + app.anchor0;
    sum_range_dt.anchor1 = sum_range_dt.anchor1 + app.anchor1;
    sum_range_dt.anchor2 = sum_range_dt.anchor2 + app.anchor2;
    sum_range_dt.anchor3 = sum_range_dt.anchor3 + app.anchor3;
    //incremento il numero di pacchetti arrivati nell'intervlo di tempo
    new_range_packet ++;
  }else
    ROS_WARN("ALMENO UN RANGE UGUALE a 0");
  
  //gettimeofday(&current_time, NULL);
  //elapsed_time_imu = (current_time.tv_sec - imu_time.tv_sec) * 1000;
  //elapsed_time_imu += (current_time.tv_usec - imu_time.tv_usec) / 1000;
  //cout << "freq IMU:" << 1/(elapsed_time_imu/1000) << endl;
  //gettimeofday(&imu_time, NULL);

  //cout << "***********************************"<< endl;
  
}
/******************************************************************************************/
/*                                                                                        */
/*                  EKF                                                                   */
/*                                                                                        */
/******************************************************************************************/

void EKF_solo_range(double range0,double range1,double range2, double range3,double dt)
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  
}