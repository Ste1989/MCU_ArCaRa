/**
*
*   Detect object
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

  iLowH = 160;
  iHighH = 179;
  iLowS = 90; 
  iHighS = 255;//165;
  iLowV = 90;
  iHighV = 255;
  size_erode_clos = 0;
  size_dil_clos = 1;//20;
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
  nh.param<int>("/VisionNode/device", cam.device, 1); 
  nh.param<double>("/VisionNode/width", cam.frame_width, 960); 
  nh.param<double>("/VisionNode/height", cam.frame_height, 540); 
  nh.param<double>("/VisionNode/fps", cam.fps, 15); 
  nh.param<double>("/VisionNode/brigthness", cam.brigthness, 0.219608); 
  nh.param<double>("/VisionNode/contrast", cam.contrast, 0.3333333); 
  nh.param<double>("/VisionNode/saturation", cam.saturation, 0.501961); 
  nh.param<double>("/VisionNode/hue", cam.hue, 0.5); 
  nh.param<double>("/VisionNode/gain", cam.gain, 0.125); 
  nh.param<double>("/VisionNode/u0", cam.u0, 0);
  nh.param<double>("/VisionNode/v0", cam.v0, 0);
  nh.param<double>("/VisionNode/fu", cam.fu, 0);
  nh.param<double>("/VisionNode/fv", cam.fv, 0);
  nh.param<double>("/VisionNode/k1", cam.k1, 0);
  nh.param<double>("/VisionNode/k2", cam.k2, 0);
  nh.param<double>("/VisionNode/p1", cam.p1, 0);
  nh.param<double>("/VisionNode/p2", cam.p2, 0);
  nh.param<std::string>("/VisionNode/img_path", params.img_path, "");
  nh.param<bool>("/VisionNode/real_time", params.real_time , false);
  nh.param<bool>("/VisionNode/save_img", params.save_img , false);
  nh.param<std::string>("/VisionNode/img_path_save", params.img_path_save, "");

 
 
  //Publisher
  //1
  image_transport::ImageTransport it(nh);
  image_pub = it.advertise("/camera/image_raw", 100);
  info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info",100);
  //2
  features_rect_pub = nh.advertise<std_msgs::Float64MultiArray>("/obj_detection/features", 10);

  //Subscriber
  attitude_sub = nh.subscribe("/napodrone/attitude", 1, AttitudeCallback);


  
  /*inizializzo
 variabili globali ***************************************************************/
  init_global_var();




  /*Se non sono in modalità debug, e ho una webcam, la apro****************************************/
  if(params.real_time)
  {
    /*Open the webcam*/
    
    cap.open(cam.device);		

     // Check if video device can be opened with the given index
    if(!cap.isOpened()) 
    {
      ROS_ERROR("No camera Found");
      return 1;
    }
    
    /*Set Camera Parameters*/
    char read_param = 1; 
    char set_param = 1;
    if(set_param)
    {
  	
      cap.set(CV_CAP_PROP_FRAME_WIDTH, cam.frame_width); 
      cap.set(CV_CAP_PROP_FRAME_HEIGHT, cam.frame_height);
      //cap.set(CV_CAP_PROP_FPS, fps);
      cap.set(CV_CAP_PROP_BRIGHTNESS,cam.brigthness);
      cap.set(CV_CAP_PROP_CONTRAST,cam.contrast);
      cap.set(CV_CAP_PROP_SATURATION,cam.saturation);
      //cap.set(CV_CAP_PROP_EXPOSURE,0.1); //not supported
      cap.set(CV_CAP_PROP_GAIN, cam.gain); //not supported
      //cap.set(CV_CAP_PROP_FOURCC, ??); //4 character code of the codec
      cap.set(CV_CAP_PROP_HUE, cam.hue);//Hue of the image
    }
    
    
    if(read_param)
    {
      cam.frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
      cam.frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
      //cam.fps = cap.get(CV_CAP_PROP_FPS);
      cam.brigthness = cap.get(CV_CAP_PROP_BRIGHTNESS);  
      cam.contrast = cap.get(CV_CAP_PROP_CONTRAST);
      cam.saturation = cap.get(CV_CAP_PROP_SATURATION);
      //exposure = cap.get(CV_CAP_PROP_EXPOSURE);
      cam.gain = cap.get(CV_CAP_PROP_GAIN); // not supported
      //fourcc = cap.get(CV_CAP_PROP_FOURCC);// 4 character code of the codec
      cam.hue = cap.get(CV_CAP_PROP_HUE); //Hue of the image
      /*STAMPO A VIDEO PARAMETRI CAMERa*/
      std::cout <<"width: "<< cam.frame_width << std::endl;
      std::cout <<"height: "<< cam.frame_height << std::endl;
      std::cout <<"fps: "<< cam.fps << std::endl;
      std::cout <<"brightness: "<< cam.brigthness << std::endl;
      std::cout <<"contrast: "<< cam.contrast << std::endl;
      std::cout <<"saturation: "<< cam.saturation << std::endl;
      std::cout <<"exposure: "<< cam.exposure << std::endl;
      //std::cout <<"fourcc: "<< fourcc << std::endl;
      std::cout <<"gain: "<< cam.gain << std::endl;
      std::cout <<"hue: "<< cam.hue << std::endl;
    }

  }else
  {
    //creo le finestre di controllo
    make_control_window();
  }


  
 
  

  /*******************************cilco principale***************************************************/

  

  int id_img = 0;
  /*if(params.real_time)
  {
    fd1=fopen("/home/odroid/time.txt", "w");
    fclose(fd1);
  }*/


/*******************************************************************************************************
                  LOOP
*********************************************************************************************************/



  while (nh.ok()) 
  {


    //caso in cui uso la WebCam
    if(params.real_time)
    {   
        Mat bgr_image = Mat::zeros( cv::Size(cam.frame_width,cam.frame_height), CV_8UC3 );
        //catturo l'immagine dalla webcam
        cap >> bgr_image;
        // Check if grabbed frame is actually full with some content
        if(bgr_image.empty()) 
        {
          ROS_ERROR("No camera Found");
        }
    
        //pubblico sul topic l'immagine che ho appena acquisito//////////////////////////////////////////////////
        sensor_msgs::CameraInfo info_image;
        info_image.height = bgr_image.size().height;
        info_image.width = bgr_image.size().width;
        sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg();
        image_pub.publish(msg_image);
        info_pub.publish(info_image);

        //elaboro l'immagine
        obj_detection(bgr_image);
      
      
    }
    //se non utilizzo la camera...
    else{
 
      stringstream ss_id;
      stringstream ss;
      ss << idx;
      string str = ss.str();
      ss_id << id_test;
      string num_test = ss_id.str();
      if(idx < 10)
        str = params.img_path + "test" + num_test+ "/left000"+ str+ ".jpg";
      if(idx>=10 && idx < 100)
        str = params.img_path + "test" + num_test+ "/left00"+ str+ ".jpg";
      if(idx>=100 && idx < 1000)
        str = params.img_path + "test" + num_test+ "/left0"+ str+ ".jpg";
      if(idx>=1000 && idx < 10000)
        str = params.img_path + "test" + num_test+ "/left"+ str+ ".jpg";
        
  	    
      //apro l'immagine a colori
      Mat bgr_image = Mat::zeros( cv::Size(cam.frame_width,cam.frame_height), CV_8UC3 );
      bgr_image = imread( str, CV_LOAD_IMAGE_COLOR );
      
      //la visualizzo a PC
      imshow( "original image", bgr_image );
      waitKey(30);
        

      //elaboro l'immagine
      obj_detection(bgr_image); 
      
    
    }
  
    //valuta callback
    ros::spinOnce();
  }
}
