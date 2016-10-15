/**
*
*   Detect object for Intel R200
*
*HSV;
*La tonalità H viene misurata da un angolo intorno all'asse verticale:
*con il rosso a 0 gradi, il verde a 120 e il blu a 240.
*L'altezza del modello rappresenta la luminosità (L) con lo zero che rappresenta il nero e l'uno il bianco.
*La saturazione (S) invece va da zero, sull'asse del modello, a uno sulla sua superficie.
tabella : http://codicicolori.com/codici-colori-hsv , https://en.wikipedia.org/wiki/File:HueScale.svg
OPencv
H : 0-179 valore *2 = angolo
S : 0-255
V :0-255
*/

#include "vision_node.h"




/********************************************************************************
*
*    INIT VARIABILI GLOBALI
*
**************************************************************************************/
void init_global_var(  )
{

  cout << "*********************************************" << endl;
  cout << "PARAMETRI SETTATI PER SCATOLA                " << endl;
  cout << "*********************************************" << endl;
  iLowH = 160;
  iHighH = 179;
  iLowS = 50; //bottiglia = 90
  iHighS = 255;//165;
  iLowV = 90;
  iHighV = 255;
  size_erode_clos = 0;
  size_dil_clos = 3;  //bottiglia = 1
  size_erode_fill = 1;
  size_dil_fill = 1;//20;
  idx = 695;
  id_test = 0;
  id_img = 0;
  //binarizzazione
  bin_threshold = 0;
  bin_type = 1;



  //blob
  min_threshold_blob = 0;
  max_threshold_blob = 255;
  thresholdStep_blob = 10;
  min_dist_blob = 0;
  max_area_blob = 100000;
  min_area_blob = 3000;
  blob_color = 0;

  //contour
  area_min_contour = 50;

}

/********************************************************************************
*
*    READ ATTITUDE CALLBACK
*
**************************************************************************************/

void AttitudeCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  //read the curret attitude of the drone
  attitude_UAV.roll = msg->x;
  attitude_UAV.pitch = msg->y;
  attitude_UAV.yaw = msg->z;
}
/********************************************************************************
*
*    READ IMAGE REAL SENSE CALLBACK
*
**************************************************************************************/

void ImageRealSenseCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  //apro l'immagine a colori
  
  image_packet new_image;

  //Mat bgr_image = Mat::zeros( cv::Size(cam.frame_width,cam.frame_height), CV_8UC3 );
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  Mat bgr_image = cv_ptr->image;
  
  new_image.bgr_image = bgr_image;
  new_image.header_im = msg->header;
  obj_detection_function(new_image);
}


/********************************************************************************
*
*    MAIN
*
**************************************************************************************/
int main(int argc, char** argv)
{


  ros::init(argc, argv, "obj_detection");
  ros::NodeHandle nh;
  
  //ros::Rate loop_rate(100);
  /*Camera 	parameters*/
  camera cam;
  
  //leggo i parametri specificati nel launch file
  nh.param<std::string>("/VisionNode/img_path", params.img_path, "");
  nh.param<bool>("/VisionNode/real_time", params.real_time , false);
  nh.param<bool>("/VisionNode/save_img", params.save_img , false);
  nh.param<std::string>("/VisionNode/img_path_save", params.img_path_save, "");

  
  //mi devo sottorscrivere al topi della real sense r200
  image_r200_sub= nh.subscribe("/camera/color/image_raw", 1, ImageRealSenseCallback);
  
  //Subscriber
  attitude_sub = nh.subscribe("/napodrone/attitude", 1, AttitudeCallback);
  
  //Publisher
  features_rect_pub = nh.advertise<obj_detection::Features>("/obj_detection/features", 10);

  
  /*inizializzo
 variabili globali ***************************************************************/
  init_global_var();




  /*Se  sono in modalità debug ****************************************/
  
  if(!params.real_time)
  {
    
    //creo le finestre di controllo
    make_control_window();

  }
  

  /*******************************************************************************************************
                  LOOP PRINCIPALE
  *********************************************************************************************************/



  while (nh.ok()) 
  {    
    //valuta callback
    ros::spinOnce();
  }
}
