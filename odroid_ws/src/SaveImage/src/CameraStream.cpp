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
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <time.h>

clock_t start,end;
clock_t start_cv;


ros::Publisher info_pub;
image_transport::Publisher pub;

using namespace cv;
using namespace std;

bool real_time, save_img;

cv::VideoCapture cap;

int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

int bin_threshold ;
int bin_type; ;


int iLowH ;
int iHighH ;
int iLowS ; 
int iHighS;
int iLowV;
int iHighV;
int size_erode_clos;
int size_dil_clos;
int size_erode_fill;
int size_dil_fill;
int idx ;
int id_test;

int min_area_blob;
int blob_color ;
int min_threshold_blob;
int max_threshold_blob ;
int min_dist_blob ;
int max_area_blob;
int thresholdStep_blob ;


//debug mode, per clibrazione di "copri pinza"
int x = 0;
int y = 0;
int h = 20;
int w = 20;

/********************************************************************************
*
*    MAIN
*
**************************************************************************************/
int main(int argc, char** argv)
{


  ros::init(argc, argv, "obj_detection");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  pub = it.advertise("/camera/image_raw", 100);
  info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info",100);
  //ros::Rate loop_rate(100);
  /*Camera 	parameters*/
  int device;
  double frame_width, frame_height, fps, brigthness, contrast, saturation, exposure, gain, hue, fourcc;
  string img_path, img_path_save;
  //leggo i parametri specificati nel launch file
  nh.param<int>("/CameraStream/device", device, 2); //di defualt camera intel r200
  nh.param<double>("/CameraStream/width", frame_width, 960); //di defualt camera intel r200
  nh.param<double>("/CameraStream/height", frame_height, 540); //di defualt camera intel r200
  nh.param<double>("/CameraStream/fps", fps, 15); //di defualt camera intel r200
  nh.param<double>("/CameraStream/brigthness", brigthness, 0.219608); //di defualt camera intel r200
  nh.param<double>("/CameraStream/contrast", contrast, 0.3333333); //di defualt camera intel r200
  nh.param<double>("/CameraStream/saturation", saturation, 0.501961); //di defualt camera intel r200
  nh.param<double>("/CameraStream/hue", hue, 0.5); //di defualt camera intel r200
  nh.param<double>("/CameraStream/gain", gain, 0.125); //di defualt camera intel r200
  nh.param<std::string>("/CameraStream/img_path", img_path, "");
  nh.param<bool>("/CameraStream/real_time", real_time , false);
  nh.param<bool>("/CameraStream/save_img", save_img , false);
  nh.param<std::string>("/CameraStream/img_path_save", img_path_save, "");
  




  /*Se non sono in modalità debug, apro la webcam************************************************/
  if(real_time)
  {
    /*Open the webcam*/

    cap.open(device);		//Open system default camera INTEl

     // Check if video device can be opened with the given index
    if(!cap.isOpened()) 
    {
      ROS_ERROR("No camera Found");
      return 1;
    }

    /*Set Camera Parameters*/
    char read_param = 1; 
    char set_param = 0;
    if(set_param)
    {
  	
      cap.set(CV_CAP_PROP_FRAME_WIDTH, frame_width); 
      cap.set(CV_CAP_PROP_FRAME_HEIGHT, frame_height);
      //cap.set(CV_CAP_PROP_FPS, fps);
      cap.set(CV_CAP_PROP_BRIGHTNESS,brigthness);
      cap.set(CV_CAP_PROP_CONTRAST,contrast);
      cap.set(CV_CAP_PROP_SATURATION,saturation);
      //cap.set(CV_CAP_PROP_EXPOSURE,0.1); //not supported
      cap.set(CV_CAP_PROP_GAIN, gain); //not supported
      //cap.set(CV_CAP_PROP_FOURCC, ??); //4 character code of the codec
      cap.set(CV_CAP_PROP_HUE, hue);//Hue of the image
    }
    
    
    if(read_param)
    {
      frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
      frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
      //fps = cap.get(CV_CAP_PROP_FPS);
      brigthness = cap.get(CV_CAP_PROP_BRIGHTNESS);  
      contrast = cap.get(CV_CAP_PROP_CONTRAST);
      saturation = cap.get(CV_CAP_PROP_SATURATION);
      //exposure = cap.get(CV_CAP_PROP_EXPOSURE);
      gain = cap.get(CV_CAP_PROP_GAIN); // not supported
      //fourcc = cap.get(CV_CAP_PROP_FOURCC);// 4 character code of the codec
      hue = cap.get(CV_CAP_PROP_HUE); //Hue of the image
      /*STAMPO A VIDEO PARAMETRI CAMERa*/
      std::cout <<"width: "<< frame_width << std::endl;
      std::cout <<"height: "<< frame_height << std::endl;
      std::cout <<"fps: "<< fps << std::endl;
      std::cout <<"brightness: "<< brigthness << std::endl;
      std::cout <<"contrast: "<< contrast << std::endl;
      std::cout <<"saturation: "<< saturation << std::endl;
      std::cout <<"exposure: "<< exposure << std::endl;
      //std::cout <<"fourcc: "<< fourcc << std::endl;
      std::cout <<"gain: "<< gain << std::endl;
      std::cout <<"hue: "<< hue << std::endl;
    }

  }

  
 
  

  /*******************************cilco principale***************************************************/
  //prealloco tutte le immagini
  Mat bgr_image = Mat::zeros( cv::Size(frame_width,frame_height), CV_8UC3 );
  Mat bgr_image_rs = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat hsv_image = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat imgThresholded = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat img_red = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat bin_image = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat img_edges = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat drawing = Mat::zeros(cv::Size(320,180), CV_8UC3 );

  int id_img = 0;

  while (nh.ok()) 
  {
    double tempo;
    start=clock();

    
    if(real_time)
    {
      //catturo l'immagine
      cap >> bgr_image;
      // Check if grabbed frame is actually full with some content
      if(bgr_image.empty()) 
      {
        ROS_ERROR("No camera Found");
      }
      //salvo l'immagine se richiesto
      if(save_img)
      {
        stringstream s_idx;
        s_idx << id_img;
        string str_path  = s_idx.str();

        if(id_img < 10)
          str_path  = img_path_save + "/left000"+ str_path + ".jpg";
        if(id_img>=10 && id_img < 100)
          str_path  = img_path_save +  "/left00"+ str_path + ".jpg";
        if(id_img>=100 && id_img < 1000)
          str_path  = img_path_save + "/left0"+ str_path + ".jpg";
        if(id_img>=1000 && id_img < 10000)
          str_path  = img_path_save +  "/left"+ str_path + ".jpg";

        //salvo su disco l'immagine

        imwrite( str_path, bgr_image );

        id_img++;
      }

    }

   


  
    //pubblico sul topic l'immagine che ho appena acquisito//////////////////////////////////////////////////
    sensor_msgs::CameraInfo info_image;
    info_image.height = bgr_image.size().height;
    info_image.width = bgr_image.size().width;
    sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg();
    pub.publish(msg_image);
    info_pub.publish(info_image);

    
  }
}
