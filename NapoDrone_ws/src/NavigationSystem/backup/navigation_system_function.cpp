#include "navigation_system_node.h"

/******************************************************************************************/
/*                                                                                        */
/*                  INIT                                                                  */
/*                                                                                        */
/******************************************************************************************/
void init_global_var()
{

  //new packet from the callback
  new_imu_packet = 0;
  new_attitude_packet = 0;
  new_mag_packet = 0;


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
  log_imu_path  = "/home/robot/MCU_ArCaRa/NapoDrone_ws/log/imu.txt";
  log_mag_path  = "/home/robot/MCU_ArCaRa/NapoDrone_ws/log/mag.txt";
  log_attitude_path  = "/home/robot/MCU_ArCaRa/NapoDrone_ws/log/attitude.txt";

  switch(log_file)
  {
    case 1:
      fd = fopen(log_imu_path.c_str(), "w");
      fclose(fd);
      break;

    case 2:
      fd = fopen(log_mag_path.c_str(), "w");
      fclose(fd);
      break;

    case 3:
      fd = fopen(log_imu_path.c_str(), "w");
      fclose(fd);
      fd = fopen(log_mag_path.c_str(), "w");
      fclose(fd);
      break;

    case 4:
      fd = fopen(log_attitude_path.c_str(), "w");
      fclose(fd);
      break;

    case 5:
      fd = fopen(log_imu_path.c_str(), "w");
      fclose(fd);
      fd = fopen(log_mag_path.c_str(), "w");
      fclose(fd);
      fd = fopen(log_attitude_path.c_str(), "w");
      fclose(fd);
      break;

    default:
      break;

  }
 


}
/******************************************************************************************/
/*                                                                                        */
/*                  IMU CALLBACK                                                          */
/*                                                                                        */
/******************************************************************************************/
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_state = *msg;
  new_imu_packet = 1;


  
  gettimeofday(&current_time, NULL);
  elapsed_time_imu = (current_time.tv_sec - imu_time.tv_sec) * 1000;
  elapsed_time_imu += (current_time.tv_usec - imu_time.tv_usec) / 1000;
  cout << "freq IMU:" << 1/(elapsed_time_imu/1000) << endl;
  gettimeofday(&imu_time, NULL);

  cout << "***********************************"<< endl;
  if(log_file == 1 || log_file == 3 || log_file == 5 )
  {

    double secs = ros::Time::now().toSec(); //mettere questo tempo o quello con cui è arrivato?
    secs = imu_state.header.stamp.sec + ((double)imu_state.header.stamp.nsec)/1000000000;
    fd = fopen(log_imu_path.c_str(), "a");
    fprintf(fd, "%f", secs -  secs_0);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", imu_state.angular_velocity.x); //2
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", imu_state.angular_velocity.y); //3
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", imu_state.angular_velocity.z); //4
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", imu_state.linear_acceleration.x); //5
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", imu_state.linear_acceleration.y); //6
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f\n",imu_state.linear_acceleration.z); //7
    fclose(fd);

  }
}

/******************************************************************************************/
/*                                                                                        */
/*                  ASSETTO CALLBACK                                                      */
/*                                                                                        */
/******************************************************************************************/
void attitude_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  attitude_state = *msg;
  new_attitude_packet = 1;

    
  gettimeofday(&current_time, NULL);
  elapsed_time_attitude = (current_time.tv_sec - attitude_time.tv_sec) * 1000;
  elapsed_time_attitude += (current_time.tv_usec - attitude_time.tv_usec) / 1000;
  cout << "freq ATT:" << 1/(elapsed_time_attitude/1000) << endl;
  gettimeofday(&attitude_time, NULL);

  if(log_file == 4 || log_file == 5 )
  {
    double secs = ros::Time::now().toSec();
    secs = attitude_state.header.stamp.sec + ((double)attitude_state.header.stamp.nsec)/1000000000;

    fd = fopen(log_attitude_path.c_str(), "a");
    fprintf(fd, "%f", secs -  secs_0);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", attitude_state.orientation.x); //2
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", attitude_state.orientation.y); //3
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", attitude_state.orientation.z); //4
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f\n", attitude_state.orientation.w); //5
    fclose(fd);

  }
}

/******************************************************************************************/
/*                                                                                        */
/*                  MAG CALLBACK                                                          */
/*                                                                                        */
/******************************************************************************************/
void mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg)
{
  mag_state = *msg;
  new_mag_packet = 1;

  gettimeofday(&current_time, NULL);
  elapsed_time_mag = (current_time.tv_sec - mag_time.tv_sec) * 1000;
  elapsed_time_mag += (current_time.tv_usec - mag_time.tv_usec) / 1000;
  cout << "freq MAG:" << 1/(elapsed_time_mag/1000) << endl;
  gettimeofday(&mag_time, NULL);

  if(log_file == 2 || log_file == 3 || log_file == 5 )
  {
    double secs = ros::Time::now().toSec();
    secs = mag_state.header.stamp.sec + ((double)mag_state.header.stamp.nsec)/1000000000;
    fd = fopen(log_mag_path.c_str(), "a");
    fprintf(fd, "%f", secs -  secs_0);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", mag_state.magnetic_field.x); //2
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", mag_state.magnetic_field.y); //3
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f\n", mag_state.magnetic_field.z); //4
    fclose(fd);

  }
}

 