#include "uwb_system_node.h"


/******************************************************************************************/
/*                                                                                        */
/*                  INIT                                                                  */
/*                                                                                        */
/******************************************************************************************/
void init_global_var()
{

  start_range_recv = 0;
  //EKF
  first_cycle_EKF = true;
  //time_ms = (1000.0/freq_filter);
  time_ms = (1.0/freq_filter);


  dt_filter = time_ms;

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

  range_rs(0) = 0.0;
  range_rs(1) = 0.0;
  range_rs(2) = 0.0;
  range_rs(3) = 0.0;

  //per debug
  index_range = 0;

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
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK STIMA DI PIMU                                                            */
/*                                                                                         */
/*******************************************************************************************/
void rangePOZ_cb(const sensor_msgs::Imu::ConstPtr& imu)
{
  
  
    double r0 = imu->angular_velocity.x/1000.0;  //anchor 1 -7 
    double r1 = imu->angular_velocity.y/1000.0; //anchor 2 -8
    double r2 = imu->angular_velocity.z/1000.0; //anchor 3 -9
    double r3 = imu->linear_acceleration.x/1000.0; //anchor 4 -10
    

    if(abs(r0) > 0.05 && abs(r1) > 0.05 && abs(r2) > 0.05 && abs(r3) > 0.05  )
    { 
      sum_range_dt.anchor0 = sum_range_dt.anchor0 + r0;
      sum_range_dt.anchor1 = sum_range_dt.anchor1 + r1;
      sum_range_dt.anchor2 = sum_range_dt.anchor2 + r2;
      sum_range_dt.anchor3 = sum_range_dt.anchor3 + r3;
      //incremento il numero di pacchetti arrivati nell'intervlo di tempo
      new_range_packet ++;
      start_range_recv = 1;
    }
    //else
      //ROS_WARN("ALMENO UN RANGE UGUALE a 0");



}
/******************************************************************************************/
/*                                                                                        */
/*                  INIT EKF                                                                   */
/*                                                                                        */
/******************************************************************************************/
void EKF_solo_range_init(VectorXd range)
{
/*
 range(0) = 6.316;
  range(1) = 3.13;
  range(2) = 8.232;
  range(3) = 5.149;
*/
  //inizializzo il filtro
  Vector3d pos_init;
  pos_init << 0,0,0;
 

  triangolazione_range(range, pos_init);
  x_k(0) = pos_init(0);
  x_k(1) = pos_init(1);
  x_k(2) = pos_init(2);
  

  cout << " POS TRIANG INIT: " << x_k(0) << " " <<x_k(1) << " " << x_k(2) << endl;  

}
/******************************************************************************************/
/*                                                                                        */
/*                  EKF                                                                   */
/*                                                                                        */
/******************************************************************************************/

void EKF_solo_range(VectorXd range,  double dt, VectorXd& position_estimated)
{
  



  //da tirare fuori dall EKF l init
  //costrusico la matrice delle distanze
  d_hat(0) = ((x_k(0)-anchor_pos(0,0))*(x_k(0)-anchor_pos(0,0)) + (x_k(1)-anchor_pos(0,1))*(x_k(1)-anchor_pos(0,1)) +(x_k(2)-anchor_pos(0,2))*(x_k(2)-anchor_pos(0,2)));
  d_hat(1) = ((x_k(0)-anchor_pos(1,0))*(x_k(0)-anchor_pos(1,0)) + (x_k(1)-anchor_pos(1,1))*(x_k(1)-anchor_pos(1,1)) +(x_k(2)-anchor_pos(1,2))*(x_k(2)-anchor_pos(1,2)));
  d_hat(2) = ((x_k(0)-anchor_pos(2,0))*(x_k(0)-anchor_pos(2,0)) + (x_k(1)-anchor_pos(2,1))*(x_k(1)-anchor_pos(2,1)) +(x_k(2)-anchor_pos(2,2))*(x_k(2)-anchor_pos(2,2)));
  d_hat(3) = ((x_k(0)-anchor_pos(3,0))*(x_k(0)-anchor_pos(3,0)) + (x_k(1)-anchor_pos(3,1))*(x_k(1)-anchor_pos(3,1)) +(x_k(2)-anchor_pos(3,2))*(x_k(2)-anchor_pos(3,2)));
  d_hat = d_hat.abs().sqrt();

  
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



  
  //output
  position_estimated(0) = x_k(0);
  position_estimated(1) = x_k(1);
  position_estimated(2) = x_k(2);

  double min = 1000000;
  int index = -1;
  for(int i = 0; i < 4; i++)
  {
    if(range(i) < min)
    {
      min = range(i);
      index = i;
    }
  }


  //calcolo Z a paretire dalle distanze dai range
  double d0t = (range(0))*(range(0));
  double d1t = (range(1))*(range(1));
  double d2t = (range(2))*(range(2));
  double d3t = (range(3))*(range(3));
  double x0t = (x_k(0)-anchor_pos(0,0))*(x_k(0)-anchor_pos(0,0));
  double x1t = (x_k(0)-anchor_pos(1,0))*(x_k(0)-anchor_pos(1,0));
  double x2t = (x_k(0)-anchor_pos(2,0))*(x_k(0)-anchor_pos(2,0));
  double x3t = (x_k(0)-anchor_pos(3,0))*(x_k(0)-anchor_pos(3,0));
  double y0t = (x_k(1)-anchor_pos(0,1))*(x_k(1)-anchor_pos(0,1));
  double y1t = (x_k(1)-anchor_pos(1,1))*(x_k(1)-anchor_pos(1,1));
  double y2t = (x_k(1)-anchor_pos(2,1))*(x_k(1)-anchor_pos(2,1));
  double y3t = (x_k(1)-anchor_pos(3,1))*(x_k(1)-anchor_pos(3,1));
  double z_stima[4];
   z_stima[0]=  anchor_pos(0,2) - sqrt(d0t - x0t -y0t);
   z_stima[1] =  anchor_pos(1,2) - sqrt(d1t - x1t -y1t);
   z_stima[2] =  anchor_pos(2,2) - sqrt(d2t - x2t -y2t);
   z_stima[3] =  anchor_pos(3,2) - sqrt(d3t - x3t -y3t);

  position_estimated(2) = z_stima[3];
  //d = x + y + z
 /* std::cout<< "**************" << std::endl;
  std::cout<< "range 0 " << range(0) << std::endl;
  std::cout<< "range 1 " << range(1) << std::endl;
  std::cout<< "range 2 " << range(2) << std::endl;
  std::cout<< "range 3 " << range(3) << std::endl;

  std::cout<< "**************" << std::endl;
  std::cout<< "X0 " << x_k(0) << std::endl;
  std::cout<< "X1 " << x_k(1) << std::endl;


  std::cout<< "**************" << std::endl;
  std::cout<< "Z0 " << z0 << std::endl;
  std::cout<< "Z1 " << z1 << std::endl;
  std::cout<< "Z2 " << z2 << std::endl;
  std::cout<< "Z3 " << z3 << std::endl;*/


  return;
}

/******************************************************************************************/
/*                                                                                        */
/*                  triangolazione                                                        */
/*                                                                                        */
/******************************************************************************************/
void triangolazione_range(VectorXd range,  Vector3d& pos_triangolata)
{
  int triang = 2;
if( triang == 1)
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

  else 
  {
    

    double a1 = 2*(anchor1(0)-anchor0(0));
    double b1 = 2*(anchor1(1)-anchor0(1)); 
    double c1 = 2*(anchor1(2)-anchor0(2));

    double a2 = 2*(anchor2(0)-anchor1(0));
    double b2 = 2*(anchor2(1)-anchor1(1)); 
    double c2 = 2*(anchor2(2)-anchor1(2));

    double a3 = 2*(anchor3(0)-anchor2(0));
    double b3 = 2*(anchor3(1)-anchor2(1)); 
    double c3 = 2*(anchor3(2)-anchor2(2));


    double blocco1 = c2 - c1*a2/a1;
    double blocco2 = b2 - b1*a2/a1;
    double blocco12 = c3 - c1*a3/a1;
    double blocco22 = b3 - b1*a3/a1;

    double d1 = range(0)*range(0)-range(1)*range(1) - anchor0.norm() * anchor0.norm()  + anchor1.norm() * anchor1.norm();
    double d2 = range(1)*range(1)-range(2)*range(2) - anchor1.norm() * anchor1.norm()  + anchor2.norm() * anchor2.norm();
    double d3 = range(2)*range(2)-range(3)*range(3) - anchor2.norm() * anchor2.norm()  + anchor3.norm() * anchor3.norm();

    double blocco3 = d2 - a2*d1/a1;
    double blocco32 = d3 - a3*d1/a1;

    if (range(0)*range(0) + range(1)*range(1) +range(2)*range(2) +range(3)*range(3) > 0)
    {
      pos_triangolata(2) = (blocco32 - blocco3*blocco22/blocco2)/blocco12/(1 - blocco1*blocco22/(blocco2*blocco12));
      pos_triangolata(1) = (blocco3 - pos_triangolata(2)*blocco1)/blocco2;
      pos_triangolata(0) = (d1 - b1*pos_triangolata(1) - c1*pos_triangolata(2))/a1;
      
    }
    else
      pos_triangolata << 0 ,0 ,0 ;


  }

}
/******************************************************************************************/
/*                                                                                        */
/*                  anchorRange_cb                                                        */
/*                                                                                        */
/******************************************************************************************/
void anchorRange_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
  double d01 = (double)msg->position.x/1000.0;
  double d02 = (double)msg->position.y/1000.0;
  double d12 = (double)msg->position.z/1000.0;
  double d03 = (double)msg->orientation.x/1000.0;
  double d13 = (double)msg->orientation.y/1000.0;
  double d23 = (double)msg->orientation.z/1000.0;
  
  //calcolo distanza sul piano X-Y
  double r01 = sqrt(d01*d01 - ((anchor1(2)-anchor0(2))*(anchor1(2)-anchor0(2))));
  double r02 = sqrt(d02*d02 - ((anchor2(2)-anchor0(2))*(anchor2(2)-anchor0(2))));
  double r12 = sqrt(d12*d12 - ((anchor2(2)-anchor1(2))*(anchor2(2)-anchor1(2))));
  double r03 = sqrt(d03*d03 - ((anchor3(2)-anchor0(2))*(anchor3(2)-anchor0(2))));
  double r13 = sqrt(d13*d13 - ((anchor3(2)-anchor1(2))*(anchor3(2)-anchor1(2))));
  double r23 = sqrt(d23*d23 - ((anchor3(2)-anchor2(2))*(anchor3(2)-anchor2(2))));

  
  double CARNOT = (r01*r01 + r02*r02 - r12*r12)/(2*r01*r02);
  if (CARNOT > 1)
  {
    ROS_WARN("CALIBRATION ERROR");
    return;
  }

  double alpha = acos(((r01*r01 + r02*r02 - r12*r12)/(2*r01*r02)));
  //print("angolo " , alpha *180 /3.14);

  anchor0(0) = 0;
  anchor0(1) = 0;
  
        
  anchor1(1) = 0;
  anchor1(0) = r01;
  

  anchor2(1) = r02 * sin(alpha);
  anchor2(0) = r02 * cos(alpha);
  

  double a11 = anchor1(0);
  double a20 = anchor2(1);
  double a21 = anchor2(0);

  double y3 = (a11*a11 + r03*r03 - r13*r13)/(2*a11);
  double x3 = (a20*a20 - r23*r23 + (y3 - a21)*(y3 - a21) + r03*r03 - y3*y3)/(2*a20);

  anchor3(0) = y3;
  anchor3(1) = x3;
  
  cout << "CIAO" << endl;
  scrivi_file_calib();

  anchor_calib = false;
}
/******************************************************************************************/
/*                                                                                        */
/*                  scrivi calibrizione                                                    */
/*                                                                                        */
/******************************************************************************************/
void scrivi_file_calib()
{
  log_uwb_path  = "/home/robot/MCU_ArCaRa/NapoDrone_ws/log/anchor_calib.txt";
  fd = fopen(log_uwb_path.c_str(), "a");
  fprintf(fd, "%f", anchor0(0));
  fprintf(fd, "%s", "  ");
  fprintf(fd, "%f", anchor0(1));
  fprintf(fd, "%s", "  ");
  fprintf(fd, "%f", anchor1(0));
  fprintf(fd, "%s", "  ");
  fprintf(fd, "%f", anchor1(1));
  fprintf(fd, "%s", "  ");
  fprintf(fd, "%f", anchor2(0));
  fprintf(fd, "%s", "  ");
  fprintf(fd, "%f", anchor2(1));
  fprintf(fd, "%s", "  ");
  fprintf(fd, "%f", anchor3(0));
  fprintf(fd, "%s", "  ");
  fprintf(fd, "%f\n", anchor3(1));
  fclose(fd);
}
/******************************************************************************************/
/*                                                                                        */
/*                  leggi calibrizione                                                    */
/*                                                                                        */
/******************************************************************************************/
void leggi_file_calibrazione()
{
  ifstream OpenFile("/home/robot/MCU_ArCaRa/NapoDrone_ws/log/anchor_calib.txt");
  
  double x0, x1, x2, x3, y0, y1, y2, y3;;
  while(!OpenFile.eof())
  {
    OpenFile >> x0 >> y0 >> x1 >> y1 >> x2 >> y2 >> x3 >> y3;
    
  }
  OpenFile.close();

  anchor0(0) = x0;
  anchor0(1) = y0;

  anchor1(0) = x1;
  anchor1(1) = y1;
  
  anchor2(0) = x2;
  anchor2(1) = y2;

  anchor3(0) = x3;
  anchor3(1) = y3;


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

  
  ifstream OpenFile("/home/robot/MCU_ArCaRa/NapoDrone_ws/log/pozyx_range.txt");
  Num_measure =0 ;
  double temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10, temp11;
  while(!OpenFile.eof())
  {
    OpenFile >> temp1 >> temp2 >> temp3 >> temp4 >> temp5>> temp6 >> temp7 >> temp8 >> temp9 >> temp10 >> temp11 ;
    /*cout << temp1 << endl;
    cout << temp2 << endl;
    cout << temp3 << endl;
    cout << temp4 << endl;
    cout << temp5 << endl;
    cout << temp6 << endl;
    cout << temp7 << endl;
    cout << temp8 << endl;
    cout << temp9 << endl;
    cout << temp10 << endl;
    cout << temp11 << endl;*/
    Num_measure ++;
    
  }
  OpenFile.close();
  Num_measure--;
  cout << "Numero di misure" << Num_measure << endl;

  //ifstream OpenFile("/home/sistema/MCU_ArCaRa/NapoDrone_ws/log/uwb_range.txt");
  time_log = (double *) malloc(sizeof(double) * Num_measure);
  range1_log = (double *) malloc(sizeof(double) * Num_measure);
  range2_log = (double *) malloc(sizeof(double) * Num_measure);
  range3_log = (double *) malloc(sizeof(double) * Num_measure);
  range4_log = (double *) malloc(sizeof(double) * Num_measure);

  double * app_1 = (double *) malloc(sizeof(double) * Num_measure);
  double *app_2 = (double *) malloc(sizeof(double) * Num_measure);
  double *app_3 = (double *) malloc(sizeof(double) * Num_measure);
  double *app_4 = (double *) malloc(sizeof(double) * Num_measure);
  double *app_5 = (double *) malloc(sizeof(double) * Num_measure);
  double *app_6 = (double *) malloc(sizeof(double) * Num_measure);
  double f1,f2,f3,f4,f5;
  int i =0 ;
  ifstream OpenFile1("/home/robot/MCU_ArCaRa/NapoDrone_ws/log/pozyx_range.txt");
  while(!OpenFile1.eof())
  {
    OpenFile1 >> time_log[i] >> app_1[i]   >> app_2[i]  >> app_3[i]  >> app_4[i]  >> range1_log[i] >> range2_log[i] >> range3_log[i] >> range4_log[i] >> app_5[i]  >> app_6[i];
    
    i++;
  }
  OpenFile1.close();
  
  /*for (i = 0; i < Num_measure ; i++)
  {

    cout << time_log[i]  << " " << range1_log[i] << " " << range2_log[i] << " " << range3_log[i] << " " << range4_log[i] << endl;
  }*/



  free(app_1);
  free(app_2);
  free(app_3);
  free(app_4);
  free(app_5);
  free(app_6);

}
/******************************************************************************************/
/*                                                                                        */
/*                  Resample data                                                         */
/*                                                                                        */
/******************************************************************************************/
void resample_data_range()
{
  
  //ricampiono i dati 

  //calolo quanti campioni ci vogliono
  //cout << (time_log[Num_measure-1] - time_log[0])  << endl;
  num_samples_rs = (int)((time_log[Num_measure-1] - time_log[0]) / time_ms)-1;
  


  time_log_rs = (double *) malloc(sizeof(double) * num_samples_rs);
  range1_log_rs = (double *) malloc(sizeof(double) * num_samples_rs);
  range2_log_rs = (double *) malloc(sizeof(double) * num_samples_rs);
  range3_log_rs = (double *) malloc(sizeof(double) * num_samples_rs);
  range4_log_rs = (double *) malloc(sizeof(double) * num_samples_rs);
  int index_range = 0;
  
  double range1, range2, range3, range4;
  for(int i = 0; i < num_samples_rs ; i++)
  {
    double time = time_ms * (i+1);
 
    for(int j = index_range; time_log[j] <= time ; j++)
    {
      //sommo i valori se non ho buchi
      if(abs(range1_log[index_range]) > 0.1 && abs(range2_log[index_range]) > 0.1 && abs(range3_log[index_range]) > 0.1 && abs(range4_log[index_range]) > 0.1)
      {
        sum_range_dt.anchor0 = sum_range_dt.anchor0 + range1_log[index_range]/1000.0;
        sum_range_dt.anchor1 = sum_range_dt.anchor1 + range2_log[index_range]/1000.0;
        sum_range_dt.anchor2 = sum_range_dt.anchor2 + range3_log[index_range]/1000.0;
        sum_range_dt.anchor3 = sum_range_dt.anchor3 + range4_log[index_range]/1000.0;
        range1 = range1_log[index_range];
        range2 = range2_log[index_range];
        range3 = range3_log[index_range];
        range4 = range4_log[index_range];
        
      }else
      {
        sum_range_dt.anchor0 = sum_range_dt.anchor0 + range1/1000.0;
        sum_range_dt.anchor1 = sum_range_dt.anchor1 + range2/1000.0;
        sum_range_dt.anchor2 = sum_range_dt.anchor2 + range3/1000.0;
        sum_range_dt.anchor3 = sum_range_dt.anchor3 + range4/1000.0;

      }


      new_range_packet ++;
      index_range ++;

    }
    
    //qua devo prepare il segnale ricampionato
    time_log_rs[i] = time ; /// qua devo sommare il tempo inziale begin_time
    range1_log_rs[i] = sum_range_dt.anchor0 / new_range_packet;
    range2_log_rs[i] = sum_range_dt.anchor1 / new_range_packet;
    range3_log_rs[i] = sum_range_dt.anchor2 / new_range_packet;
    range4_log_rs[i] = sum_range_dt.anchor3 / new_range_packet;


    sum_range_dt.anchor0 = 0;
    sum_range_dt.anchor1 = 0;
    sum_range_dt.anchor2 = 0;
    sum_range_dt.anchor3 = 0;
    new_range_packet = 0;

    
  }

  /*
   
  for (int k = 0; k < Num_measure ; k++)
  {

    cout << time_log[k]  << " " << range1_log[k] << " " << range2_log[k] << " " << range3_log[k] << " " << range4_log[k] << endl;
  }
  cout << "********************+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
  for (int k = 0; k < num_samples ; k++)
  {

    cout << time_log_rs[k]  << " " << range1_log_rs[k] << " " << range2_log_rs[k] << " " << range3_log_rs[k] << " " << range4_log_rs[k] << endl;
  }
*/
}