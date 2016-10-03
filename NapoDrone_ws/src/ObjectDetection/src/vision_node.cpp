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

  //Publisher
  image_transport::ImageTransport it(nh);
  image_pub = it.advertise("/camera/image_raw", 100);
  info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info",100);
  //Subscriber
  attitude_sub = nh.subscribe("/napodrone/attitude", 1, AttitudeCallback);
  
  //ros::Rate loop_rate(100);
  /*Camera 	parameters*/
  camera cam;
  string img_path, img_path_save, camera_param_file;
  //leggo i parametri specificati nel launch file
  nh.param<int>("/VisionNode/device", cam.device, 2); //di defualt camera intel r200
  nh.param<double>("/VisionNode/width", cam.frame_width, 960); //di defualt camera intel r200
  nh.param<double>("/VisionNode/height", cam.frame_height, 540); //di defualt camera intel r200
  nh.param<double>("/VisionNode/fps", cam.fps, 15); //di defualt camera intel r200
  nh.param<double>("/VisionNode/brigthness", cam.brigthness, 0.219608); //di defualt camera intel r200
  nh.param<double>("/VisionNode/contrast", cam.contrast, 0.3333333); //di defualt camera intel r200
  nh.param<double>("/VisionNode/saturation", cam.saturation, 0.501961); //di defualt camera intel r200
  nh.param<double>("/VisionNode/hue", cam.hue, 0.5); //di defualt camera intel r200trtr
  nh.param<double>("/VisionNode/gain", cam.gain, 0.125); //di defualt camera intel r200
  nh.param<double>("/VisionNode/u0", cam.u0, 0);
  nh.param<double>("/VisionNode/v0", cam.v0, 0);
  nh.param<double>("/VisionNode/fu", cam.fu, 0);
  nh.param<double>("/VisionNode/fv", cam.fv, 0);
  nh.param<double>("/VisionNode/k1", cam.k1, 0);
  nh.param<double>("/VisionNode/k2", cam.k2, 0);
  nh.param<double>("/VisionNode/p1", cam.p1, 0);
  nh.param<double>("/VisionNode/p2", cam.p2, 0);
  nh.param<std::string>("/VisionNode/img_path", img_path, "");
  nh.param<bool>("/VisionNode/real_time", real_time , false);
  nh.param<bool>("/VisionNode/save_img", save_img , false);
  nh.param<std::string>("/VisionNode/img_path_save", img_path_save, "");





  
  /*inizializzo
 variabili globali ***************************************************************/
  init_global_var();




  /*Se non sono in modalità debug, apro la webcam************************************************/
  if(real_time)
  {
    /*Open the webcam*/
    
    cap.open(cam.device);		//Open system default camera INTEl

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
  //prealloco tutte le immagini
  Mat bgr_image = Mat::zeros( cv::Size(cam.frame_width,cam.frame_height), CV_8UC3 );
  Mat bgr_image_rs = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat hsv_image = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat imgThresholded = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat img_red = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  Mat img_edges = Mat::zeros( cv::Size(320,180), CV_8UC3 );
  

  int id_img = 0;
  if(real_time)
  {
    fd1=fopen("/home/odroid/time.txt", "w");
    fclose(fd1);
  }


/*******************************************************************************************************
                  LOOP
*********************************************************************************************************/



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
        
        currentTime = clock();
        //cout << double(currentTime)/CLOCKS_PER_SEC << endl;

        //salvo i tempi a cui ho acquisito la nuova immagine
        fd1=fopen("/home/odroid/time.txt", "a");
        fprintf(fd1, "%i", id_img);
        fprintf(fd1, "%s", "  ");
        fprintf(fd1, "%f\n",double(currentTime)/CLOCKS_PER_SEC) ;
        fclose(fd1);

        id_img++;
      }

    }
    //se non utilizzo la camera...
    else
    {
      stringstream ss_id;
      stringstream ss;
      ss << idx;
      string str = ss.str();
      ss_id << id_test;
      string num_test = ss_id.str();
      if(idx < 10)
        str = img_path + "test" + num_test+ "/left000"+ str+ ".jpg";
      if(idx>=10 && idx < 100)
        str = img_path + "test" + num_test+ "/left00"+ str+ ".jpg";
      if(idx>=100 && idx < 1000)
        str = img_path + "test" + num_test+ "/left0"+ str+ ".jpg";
      if(idx>=1000 && idx < 10000)
        str = img_path + "test" + num_test+ "/left"+ str+ ".jpg";
      
	  
      //apro l'immagine a colori
      bgr_image = imread( str, CV_LOAD_IMAGE_COLOR );
    
      //la visualizzo a PC
      imshow( "original image", bgr_image );
      waitKey(30);
      
    }
   



    //calcolo il tempo che ci metto per elaborare l'immagine
    start_cv=clock();

  
    //pubblico sul topic l'immagine che ho appena acquisito//////////////////////////////////////////////////
    sensor_msgs::CameraInfo info_image;
    info_image.height = bgr_image.size().height;
    info_image.width = bgr_image.size().width;
    sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg();
    image_pub.publish(msg_image);
    info_pub.publish(info_image);


    //1B-COPRO  LA PART CHE NON MI INTERESSA///////////////////////////////////////////////////////////////////
    //questo serve per eliminare la pinza dall'immagine
    //cout <<bin_image.size().width << endl << bin_image.size().height<< endl;
    if(bgr_image.size().width == 960 && bgr_image.size().height == 540)
    {
      //levo immagine pinza più in alto
      Rect rec1;
      rec1.x = 640;
      rec1.y = 0;
      rec1.height = 100;
      rec1.width = bgr_image.size().width - rec1.x;
      rectangle(bgr_image, rec1, Scalar(0,0,0), CV_FILLED, 8, 0 );
      //pinza piu bassa
      Rect rec2;
      rec2.x = 640;
      rec2.y = 440;
      rec2.height = bgr_image.size().height- rec2.y;
      rec2.width = bgr_image.size().width- rec2.x;
      rectangle(bgr_image, rec2, Scalar(0,0,0), CV_FILLED, 8, 0 );
      //cebtro pinza
      Rect rec3;
      rec3.x = 845;
      rec3.y = rec1.height;
      rec3.height = 340;
      rec3.width = bgr_image.size().width- rec3.x;
      rectangle(bgr_image, rec3, Scalar(0,0,0), CV_FILLED, 8, 0 );
    }
    /*if(bgr_image.size().width == 1920 && bgr_image.size().height == 1080)
    {
      //levo immagine pinza più in alto
      Rect rec1;
      rec1.x = 1320;
      rec1.y = 0;
      rec1.height = 150;
      rec1.width = bgr_image.size().width - rec1.x;
      rectangle(bgr_image, rec1, Scalar(0,0,0), CV_FILLED, 8, 0 );
      //pinza piu bassa
      Rect rec2;
      rec2.x = 1280;
      rec2.y = 820;
      rec2.height = bgr_image.size().height- rec2.y;
      rec2.width = bgr_image.size().width- rec2.x;
      rectangle(bgr_image, rec2, Scalar(0,0,0), CV_FILLED, 8, 0 );
      //cebtro pinza
      Rect rec3;
      rec3.x = 1690;
      rec3.y = 650;
      rec3.height = 670;
      rec3.width = bgr_image.size().width- rec3.x;
      rectangle(bgr_image, rec3, Scalar(0,0,0), CV_FILLED, 8, 0 );
    }*/

    if(!real_time && false)
    {
      namedWindow("Rect", CV_WINDOW_AUTOSIZE); 
      cvCreateTrackbar("x", "Rect", &x, bgr_image.size().width);
      cvCreateTrackbar("y", "Rect", &y, bgr_image.size().height);
      cvCreateTrackbar("h", "Rect", &h, bgr_image.size().width); 
      cvCreateTrackbar("w", "Rect", &w, bgr_image.size().height); 
      Rect rec;
      rec.x = x;
      rec.y = y;
      rec.height = h;
      rec.width = w;
      rectangle(bgr_image, rec, Scalar(120,0,255), CV_FILLED, 8, 0 );
      imshow( "copro pinza", bgr_image );
      waitKey(30);
    }

    //1-C RESIZE IMMAGINE
    Size size(320,180);//the dst image size,e.g.100x100
    resize(bgr_image,bgr_image_rs,size);//resize image
    bgr_image = bgr_image_rs;
    


    //2- ottengo l'immagine in bianco e nero e l'iimagine in HSV e filtro per i vari colori//////////////////
    Mat gray_image, gray_image_b;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
    //Mat gray_image;
    //cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
    
    //gaussian Blur
    //GaussianBlur(hsv_image, hsv_image,cv::Size(7,7),0 );


    //2A- filtro rosso
    img_red = color_filter_image(hsv_image, "red");
    //2B- filtro blu
    //Mat img_blue = color_filter_image(hsv_image, "blue");
    //.. altri filtri per colori


    //...combinazione di piu filtri..
    //addWeighted(img_red, 1.0, img_blue, 1.0, 0.0, imgThresholded);
    imgThresholded = img_red;

    if(!real_time)
    {
      imshow( "imgThresholded", imgThresholded );
      waitKey(30);
    }



    //3-Remove small object and fill small holes ////////////////////////////////////////////////////////////
    morphological_filter(imgThresholded);
    if(!real_time && false)
    {
      imshow( "imgThresholded_foreground", imgThresholded );
      waitKey(30);
    }

    //4-Canny///////////////////////////////////////////////////////////////////////////////////////////////////
    Canny(imgThresholded, img_edges, 50,100);
    if(!real_time )
    {
      imshow( "canny", img_edges );
      waitKey(30);
    }

    //5--Find contours//////////////////////////////////////////////////////////////////////////////////////////
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( img_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );
    
    //6-Find object///////////////////////////////////////////////////////////////////////////////////////////////
    vector<Point2f>  boxObj;
    find_object(contours, &boxObj);

    cout<< "oggetti num : " <<  boxObj.size() / 4 << endl;
    if(!real_time )
    {
      RNG rng(12345);
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      for(int i = 0 ; i < boxObj.size() / 4  ; i ++)
      {
        for( int j = 0; j < 4; j++ )
        {
          
          line( bgr_image_rs, boxObj[j+i*4], boxObj[(j+1)%4 + i*4], color, 2, 8 );
        }

      }    

      imshow( "drawing", bgr_image_rs );
      waitKey(30);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    end=clock();
    tempo=((double)(end-start))/CLOCKS_PER_SEC;
    cout << "FREQ: " << 1/tempo << endl;

    tempo=((double)(end-start_cv))/CLOCKS_PER_SEC;
    cout << "FREQ COMPUTER VISION: " << tempo << "meno di: " << 0.05 << endl;

    
    //valuta callback
    ros::spinOnce();
  }
}
