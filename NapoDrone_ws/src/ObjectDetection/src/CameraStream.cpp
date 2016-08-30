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
*    FINESTRE DI CONTROLLO
*
**************************************************************************************/
void make_control_window(  )
{
  /**********************Creo Una Finestra  DI Controllo***********************/
  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  //Create trackbars in "Control" window
  cvCreateTrackbar("Image num", "Control", &idx, 3000);
  cvCreateTrackbar("Test num", "Control", &id_test, 3);
  /*****************************************************************************/
  namedWindow("Test Color", CV_WINDOW_AUTOSIZE);
  cvCreateTrackbar("LowH", "Test Color", &iLowH, 179); //Hue (0 - 179) valore *2 = angolo
  cvCreateTrackbar("HighH", "Test Color", &iHighH, 179);
  cvCreateTrackbar("LowS", "Test Color", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Test Color", &iHighS, 255);
  cvCreateTrackbar("LowV", "Test Color", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Test Color", &iHighV, 255);
  cvCreateTrackbar("Clos Erod", "Test Color", &size_erode_clos, 255); //Value (0 - 255)
  cvCreateTrackbar("Clos Dil", "Test Color", &size_dil_clos, 255);
  cvCreateTrackbar("Fill Erod", "Test Color", &size_erode_fill, 255); //Value (0 - 255)
  cvCreateTrackbar("Fill Dil", "Test Color", &size_dil_fill , 255);

  /**********************Creo Una Finestra  DI Controllo***********************/
  namedWindow("Blob", CV_WINDOW_AUTOSIZE); //create a window called "Blob"
  cvCreateTrackbar("Min Tresh", "Blob", &min_threshold_blob , 255);
  cvCreateTrackbar("Max Tresh", "Blob", &max_threshold_blob , 255);
  cvCreateTrackbar("Blob Threshold Step", "Blob", &thresholdStep_blob , 50);
  cvCreateTrackbar("Blob min Area", "Blob", &min_area_blob  ,10000);
  cvCreateTrackbar("Blob max Area", "Blob", &max_area_blob  ,1000000);
  //cvCreateTrackbar("Blob Color", "Blob", &blob_color  , 255);
  cvCreateTrackbar("Min Dist", "Blob", &min_dist_blob , 500);
  /**********************Creo Una Finestra  DI Controllo***********************/
  //namedWindow("Bin e Contour", CV_WINDOW_AUTOSIZE); //create a window called "Bin e Contour"
  //cvCreateTrackbar("Canny Tresh", "Bin e Contour", &thresh  , max_thresh);
  //cvCreateTrackbar("Bin Image", "Bin e Contour", &bin_threshold , 255);
  //cvCreateTrackbar("Bin Type", "Bin e Contour", &bin_type;  , 4);


}
/********************************************************************************
*
*    BLOB
*
**************************************************************************************/
void find_and_extract_blob(Mat src_image)
{

  // Set up the detector with parameters.
  // Setup SimpleBlobDetector parameters.
  SimpleBlobDetector::Params params;
  // Change thresholds
  params.minThreshold = min_threshold_blob;
  params.maxThreshold = max_threshold_blob;
  params.thresholdStep = thresholdStep_blob+1;

  //min dist blob
  params.minDistBetweenBlobs = min_dist_blob;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = min_area_blob+1;
  params.maxArea = max_area_blob + 1;
  
  //Colorros::Publisher info_pub
  params.filterByColor = false;
  params.blobColor = blob_color;

  //other parameter
  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.filterByCircularity = false;

  //detect blob 
  std::vector<KeyPoint> keypoints;
  
  SimpleBlobDetector detector(params);
  detector.detect( src_image, keypoints);

  //Ptr<SimpleBlobDetector> sbd = SimpleBlobDetector::create(params);
  //sbd->detect(src_image, keypoints, Mat());

  
  
    Mat im_with_keypoints;
    for(std::vector<cv::KeyPoint>::iterator blobIterator = keypoints.begin(); blobIterator != keypoints.end(); blobIterator++)
    {
      std::cout << "size of blob is: " << blobIterator->size << std::endl;
      std::cout << "point is at: " << blobIterator->pt.x << " " << blobIterator->pt.y << std::endl;
      
      if(!real_time)
      {
        stringstream ssx;
        stringstream ssy;
        ssx << blobIterator->pt.x;
        ssy << blobIterator->pt.y;

        string text = "(" + ssx.str() + "," + ssy.str() + ")";
        //center the text
        Point textOrg(blobIterator->pt.x,blobIterator->pt.y);
        // then put the text itself
        putText(src_image, text, textOrg, FONT_HERSHEY_SCRIPT_SIMPLEX, 0.1, Scalar(120,0,255), 1, 8);
      }
     }
    
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    if(!real_time)
    {
      drawKeypoints( src_image, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      // Show blobs
      imshow("blobs", im_with_keypoints );
      waitKey(30);
    }
    //sensor_msgs::CameraInfo info_image;
    //info_image.height = im_with_keypoints.size().height;
    //info_image.width = im_with_keypoints.size().width;
    //sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", im_with_keypoints).toImageMsg();
    //pub.publish(msg_image);
    //info_pub.publish(info_image);



  

  
}
/********************************************************************************
*
*    MORPHOLOGICAL FILTER
*
**************************************************************************************/
void morphological_filter(Mat& image)
{
  //morphological opening (remove small objects from the foreground)
  erode(image, image, getStructuringElement(MORPH_ELLIPSE, Size(size_erode_clos+1, size_erode_clos+1)) );
  dilate( image, image, getStructuringElement(MORPH_ELLIPSE, Size(size_dil_clos+1, size_dil_clos+1)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate(image, image, getStructuringElement(MORPH_ELLIPSE, Size(size_erode_fill+1, size_erode_fill+1)) ); 
  erode(image, image, getStructuringElement(MORPH_ELLIPSE, Size(size_dil_fill+1, size_dil_fill+1)) );

}
/********************************************************************************
*
*    FILTER COLOR
*
**************************************************************************************/
Mat color_filter_image(Mat hsv_image, string color )
{ // http://www.rapidtables.com/convert/color/rgb-to-hsv.htm
  Mat imgThresholded;
  if (color == "red")
  { 
    Mat upper_red, lower_red;
    inRange(hsv_image, Scalar(0, iLowS, iLowV), Scalar(10, iHighS, iHighV), lower_red);
    inRange(hsv_image, Scalar(165, iLowS, iLowV), Scalar(179, iHighS, iHighV), upper_red);  
    addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0, imgThresholded);
  }
  if (color == "blue")
  { 

    inRange(hsv_image, Scalar(100, iLowS, iLowV), Scalar(120, iHighS, iHighV), imgThresholded);   

  }
  
  //GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);
  return imgThresholded;

}
/********************************************************************************
*
*    Binarizazione
*
**************************************************************************************/
Mat Binarize_Image( Mat src_gray)
{
  /* 0: Binary
     1: Binary Inverted
     2: Threshold Truncated
     3: Threshold to Zero
     4: Threshold to Zero Inverted
   */
  Mat dst;
  threshold( src_gray, dst, bin_threshold, max_BINARY_value,bin_type);
  return dst;
}
/********************************************************************************
*
*    FIND CONTOURS
*
**************************************************************************************/
/** @function thresh_callback */
void thresh_callback(Mat src_gray)
{
   
  int thresh ;
  int max_thresh;
  thresh = 100;
  max_thresh = 255;
  RNG rng(12345);

  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );
  //CV_CHAIN_APPROX_SIMPLE
  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
      {
         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
         drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
      }
  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
  waitKey(30);
  
  int idx = -1;
  double area_max = 0;
  vector<Point> contours_max;

  for(int i = 0; i < contours.size(); i++)
  { cout << contourArea(contours[i]) << endl;
    if(contourArea(contours[i]) > area_max)
    {
      area_max = contourArea(contours[i]);
      idx = i;
    }
  }
  cout << "area max" << area_max << endl;
  //copio i valori dentro il vettore
  for(int i = 0; i < contours[idx].size(); i++)
  {
    contours_max.push_back(contours[idx][i]);
  }


  /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( 1);
    vector<Rect> boundRect( 1);

    //epsilon – Parameter specifying the approximation accuracy. This is the maximum distance between the original curve and its approximation.
    int epsilon = 3;
    bool closed = true;
    for( int i = 0; i < 1; i++ )
    { 
      approxPolyDP( Mat(contours_max), contours_poly[i], epsilon, closed );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        
    }



     vector<Moments> mu(1);
    for( size_t i = 0; i < 1; i++ )
      {
        mu[i] = moments( contours_max, false ); }

    vector<Point2f> mc( 1 );
    for( size_t i = 0; i < 1; i++ )
     { 
      mc[i] = Point2f( static_cast<float>(mu[i].m10/mu[i].m00) , static_cast<float>(mu[i].m01/mu[i].m00) ); 
     }
    


    /// Draw polygonal contour + bonding rects + circles
    Mat drawing_box = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( int i = 0; i< 1; i++ )
    {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing_box, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       //rectangle( drawing_box, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
        circle( drawing_box, mc[i], 4, color, -1, 8, 0 );

     }
   

  /// Show in a window
  namedWindow( "Contours box", CV_WINDOW_AUTOSIZE );
  imshow( "Contours box", drawing_box );
  
  waitKey(30);


}
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
  idx = 0;
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

}
/********************************************************************************
*
*    SORT POINT
*
**************************************************************************************/
struct str{
    bool operator() ( Point2f a, Point2f b ){
        if ( a.x != b.x ) 
            return a.x < b.x;
        return a.y <= b.y ;
    }
} comp;
void sort_point(Point2f* rect_points)
{

  
  //ordino i puti secondo l'ascissa, da minore a maggiore
  sort(rect_points,rect_points+4,comp);

  //order coordinates clockwise
  //now, sort the left-most coordinates according to their
  //y-coordinates so we can grab the top-left and bottom-left
  //points, respectively
  Point2f bottom_left, top_left;
  if(rect_points[0].y > rect_points[1].y)
  {
    Point2f app;
    app.x = rect_points[0].x;
    app.y = rect_points[0].y;
    rect_points[0].x = rect_points[1].x;
    rect_points[0].y = rect_points[1].y;
    rect_points[1].x = app.x;
    rect_points[1].y = app.y;

  }
  else
  {
    //..già ordinati
  }

  //adesso ho il primo punto bottom_left, il secondo top_left
  //adesso calcolo la distanza dal bottom left agli altri due punti a destra.
  //qeullo con distanza maggiore sarà il top_right
  double dist_1 = (rect_points[0].x * rect_points[2].x ) + (rect_points[0].y * rect_points[2].y ) ;
  double dist_2 = (rect_points[0].x * rect_points[3].x ) + (rect_points[0].y * rect_points[3].y ) ;
  if (dist_1 < dist_2)
  {
    //scambio di posizione il 3 con il 4
    Point2f app;
    app.x = rect_points[2].x;
    app.y = rect_points[2].y;
    rect_points[2].x = rect_points[3].x;
    rect_points[2].y = rect_points[3].y;
    rect_points[3].x = app.x;
    rect_points[3].y = app.y;
  }
  else
  {
    //sono già tutti ordinati
  }

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
  
  /*inizializzo variabili globali ***************************************************************/
  init_global_var();




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
    char set_param = 1;
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

  }else
  {

    //creo le finestre di controllo
    make_control_window();

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
   


  
    //pubblico sul topic l'immagine che ho appena acquisito//////////////////////////////////////////////////
    sensor_msgs::CameraInfo info_image;
    info_image.height = bgr_image.size().height;
    info_image.width = bgr_image.size().width;
    sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg();
    pub.publish(msg_image);
    info_pub.publish(info_image);

    //calcolo il tempo che ci metto per elaborare l'immagine
    start_cv=clock();

    //1B-COPRO  LA PART CHE NON MI INTERESSA///////////////////////////////////////////////////////////////////
    //questo serve per eliminare la pinza dall'immagine
    //cout <<bin_image.size().width << endl << bin_image.size().height<< endl;
    if(bgr_image.size().width == 960 && bgr_image.size().height == 540)
    {
      //levo immagine pinza più in alto
      Rect rec1;
      rec1.x = 655;
      rec1.y = 0;
      rec1.height = 70;
      rec1.width = bgr_image.size().width - rec1.x;
      rectangle(bgr_image, rec1, Scalar(0,0,0), CV_FILLED, 8, 0 );
      //pinza piu bassa
      Rect rec2;
      rec2.x = 640;
      rec2.y = 410;
      rec2.height = bgr_image.size().height- rec2.y;
      rec2.width = bgr_image.size().width- rec2.x;
      rectangle(bgr_image, rec2, Scalar(0,0,0), CV_FILLED, 8, 0 );
      //cebtro pinza
      Rect rec3;
      rec3.x = 860;
      rec3.y = 65;
      rec3.height = 355;
      rec3.width = bgr_image.size().width- rec3.x;
      rectangle(bgr_image, rec3, Scalar(0,0,0), CV_FILLED, 8, 0 );
    }
    if(bgr_image.size().width == 1920 && bgr_image.size().height == 1080)
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
    }

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

    //Canny
    Canny(imgThresholded, img_edges, 50,100);
    if(!real_time )
    {
      imshow( "canny", img_edges );
      waitKey(30);
    }

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours( img_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

    /// Find the rotated rectangles  for each contour
    vector<RotatedRect> minRect( contours.size() );

    
    RNG rng(12345);
    for( int i = 0; i< contours.size(); i++ )
    {

      Point2f rect_points[4]; 
      minRect[i] = minAreaRect( Mat(contours[i]) );
      minRect[i].points( rect_points );
      //ordino i punti dal bottom_left in senso orario
      sort_point(rect_points);
      

      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      // ontour
      drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      for( int j = 0; j < 4; j++ )
      {
        line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
      }
    }
    
    if(!real_time )
    {
      imshow( "drawing", drawing );
      waitKey(30);
    }


    //4-Binarizzare l'immagine e eliminare parti non interessanti nell'immagine///////////////////////////////
    /*bin_image = Binarize_Image(imgThresholded);
    if(!real_time)
    {
      imshow( "Binarizzata", bin_image );
      waitKey(30);
    }
    //5-Cerca blob
    find_and_extract_blob(bin_image);
     */

    end=clock();
    tempo=((double)(end-start))/CLOCKS_PER_SEC;
    cout << "FREQ: " << 1/tempo << endl;

    tempo=((double)(end-start_cv))/CLOCKS_PER_SEC;
    cout << "FREQ COMPUTER VISION: " << 1/tempo << endl;

    //loop_rate.sleep();
    //valuta callback
    ros::spinOnce();
  }
}
