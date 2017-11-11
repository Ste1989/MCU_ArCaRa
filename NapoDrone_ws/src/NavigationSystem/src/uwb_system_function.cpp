#include "uwb_system_node.h"
#include <fstream>
/******************************************************************************************/
/*                                                                                        */
/*                  INIT                                                                  */
/*                                                                                        */
/******************************************************************************************/
void init_global_var()
{

  //EKF
  first_cycle_EKF = true;
  time_ms = (1000.0/freq_filter);
  dt_filter = time_ms/1000.0;
  A << 1,0,0,dt_filter,0,0,
       0,1,0,0,dt_filter,0,
       0,0,1,0,0,dt_filter,
       0,0,0,1,0,0,
       0,0,0,0,1,0,
       0,0,0,0,0,1;
  x_k << 0,0,0,0,0,0;
  anchor_pos << anchor0(0),anchor0(1),anchor0(2),
                anchor1(0),anchor1(1),anchor1(2),
                anchor2(0),anchor2(1),anchor2(2),
                anchor3(0),anchor3(1),anchor3(2);
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

void EKF_solo_range(VectorXd range,  double dt, VectorXd& position_estimated)
{
  if(first_cycle_EKF)
  {

    //inizializzo il filtro
    Vector3d pos_init;
    pos_init << 0,0,0;
    triangolazione_range(range, pos_init);
    x_k(0) = pos_init(0);
    x_k(1) = pos_init(1);
    x_k(2) = pos_init(2);
    first_cycle_EKF = false;
  }
  else
  {

    //costrusico la matrice delle distanze
    d_hat(0) = ((x_k(0)-anchor_pos(0,0))*(x_k(0)-anchor_pos(0,0)) + (x_k(1)-anchor_pos(0,1))*(x_k(1)-anchor_pos(0,1)) +(x_k(2)-anchor_pos(0,2))*(x_k(2)-anchor_pos(0,2)));
    d_hat(1) = ((x_k(0)-anchor_pos(1,0))*(x_k(0)-anchor_pos(1,0)) + (x_k(1)-anchor_pos(1,1))*(x_k(1)-anchor_pos(1,1)) +(x_k(2)-anchor_pos(1,2))*(x_k(2)-anchor_pos(1,2)));
    d_hat(2) = ((x_k(0)-anchor_pos(2,0))*(x_k(0)-anchor_pos(2,0)) + (x_k(1)-anchor_pos(2,1))*(x_k(1)-anchor_pos(2,1)) +(x_k(2)-anchor_pos(2,2))*(x_k(2)-anchor_pos(2,2)));
    d_hat(3) = ((x_k(0)-anchor_pos(3,0))*(x_k(0)-anchor_pos(3,0)) + (x_k(1)-anchor_pos(3,1))*(x_k(1)-anchor_pos(3,1)) +(x_k(2)-anchor_pos(3,2))*(x_k(2)-anchor_pos(3,2)));
    d_hat.abs().sqrt();
    for(int i=0;i<4;i++)
    {
      for(int j=0;j<3;j++)
        //jacobiano di misura
        H(i,j) = ( 1/d_hat(i) ) * ( x_k(j) - anchor_pos(i,j) );
    }

  //propago lo stato
  x_k = A*x_k;
  //propago la matrice di covarianza
  P = A*P*A.transpose() + Q;

  //Calcolo il guadagno di Kalman
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse(); //6x4

  VectorXd resid(4);
  //calcolo residuo
  resid(0) = range(0) - d_hat(0);
  resid(1) = range(1) - d_hat(1);
  resid(2) = range(2) - d_hat(2);
  resid(3) = range(3) - d_hat(3);

  //aggiorno lo stato e la covarianza della stima
  x_k = x_k + K*resid;
  P = (MatrixXd::Identity(6,6)-K*H)*P*(MatrixXd::Identity(6,6)-K*H).transpose() +K*R*K.transpose();


  }
  //output
  position_estimated(0) = x_k(0);
  position_estimated(1) = x_k(1);
  position_estimated(2) = x_k(2);
  return;
}

/******************************************************************************************/
/*                                                                                        */
/*                  triangolazione                                                        */
/*                                                                                        */
/******************************************************************************************/
void triangolazione_range(VectorXd range,  Vector3d& pos_triangolata)
{

  Vector3d diff_distanza;

  //distanze tra le ancora
  diff_distanza(0) = anchor1.norm() - anchor0.norm();
  diff_distanza(1) = anchor2.norm() - anchor0.norm();
  diff_distanza(2) = anchor3.norm() - anchor0.norm();

  MatrixXd A(3,3);
  A(0.0) = (anchor0(0) - anchor1(0)); 
  A(0,1) = (anchor0(1) - anchor1(1));
  A(0,2) = (anchor0(2) - anchor1(2));

  A(1,0) = (anchor0(0) - anchor2(0)); 
  A(1,1) = (anchor0(1) - anchor2(1));
  A(1,2) = (anchor0(2) - anchor2(2));

  A(2,0) = (anchor0(0) - anchor3(0)); 
  A(2,1) = (anchor0(1) - anchor3(1));
  A(2,2) = (anchor0(2) - anchor3(2));
  A = 2*A;
 
  Vector3d b;
  b(0) = ((range(1)*range(1))-(range(0)*range(0))-(diff_distanza(0)*diff_distanza(0))); 
  b(1) = ((range(2)*range(2))-(range(0)*range(0))-(diff_distanza(1)*diff_distanza(1)));
  b(2) = ((range(3)*range(3))-(range(0)*range(0))-(diff_distanza(2)*diff_distanza(2)));

  
  if (range(0)*range(0) + range(1)*range(1) +range(2)*range(2) +range(3)*range(3) > 0)
    pos_triangolata = A.inverse() * b;
  else
    pos_triangolata << 0 ,0 ,0 ;

}
/******************************************************************************************/
/*                                                                                        */
/*                  triangolazione                                                        */
/*                                                                                        */
/******************************************************************************************/
void leggi_file_debug()
{

  FILE* fd;
  float* num;
   FILE *fp;
  
   fp = fopen("/home/sistema/MCU_ArCaRa/NapoDrone_ws/log/uwb_manager","r");
   while(1) {
      fscanf(fd, "%g", num); 
     /* if( feof(fp) ) { 
         break ;
      }
      printf("%f", num);*/
   }
   fclose(fp);


}