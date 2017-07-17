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
  secs_0 = ros::Time::now().toSec();
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

  if(log_file == 1 || log_file == 3 || log_file == 5 )
  {

    double secs = ros::Time::now().toSec(); //mettere questo tempo o quello con cui è arrivato?
    double secs_cb = imu_state.header.stamp.sec  + ((double)(imu_state.header.stamp.sec ))/1000000000; 
    cout << "diff_temp_imu: " << secs- secs_cb << endl;
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

  if(log_file == 4 || log_file == 5 )
  {
    double secs = ros::Time::now().toSec();
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


  if(log_file == 2 || log_file == 3 || log_file == 5 )
  {
    double secs = ros::Time::now().toSec();
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

 