#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include "obj_detection/Features.h"
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//Topic Publisher
ros::Publisher info_pub;
image_transport::Publisher image_pub;
ros::Publisher features_rect_pub;
//Topic Subscriber
ros::Subscriber attitude_sub;
ros::Subscriber image_r200_sub;


using namespace cv;
using namespace std;


//videocapture
cv::VideoCapture cap;

//binarizzazione
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;
int bin_threshold ;
int bin_type;

//controllo immagine
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
int id_img;
//blob
int min_area_blob;
int blob_color ;
int min_threshold_blob;
int max_threshold_blob ;
int min_dist_blob ;
int max_area_blob;
int thresholdStep_blob;

//time
clock_t start,end;
clock_t start_cv;
clock_t currentTime;
//file
FILE *fd1 ;

//contour
double area_min_contour;
//debug mode, per calibrazione di "copri pinza"
int x = 0;
int y = 0;
int h = 20;
int w = 20;


/*************************************************************************************
*
*   STRUTTURE
*
*******************************************************************************************/
//camera
struct camera
{
  int device;
  double frame_width;
  double frame_height;
  double fps;
  double brigthness;
  double contrast;
  double saturation;
  double exposure;
  double hue;
  double gain;
  double u0;
  double v0;
  double fu;
  double fv;
  double k1;
  double k2;
  double p1;
  double p2;
};

//parametri nodo
struct param_node{
  string img_path;
  string img_path_save;
  string camera_param_file;
  bool save_img;
  bool real_time;
}params;

//assetto struttura
struct attitude
{
    double roll;
    double pitch;
    double yaw;
}attitude_UAV;
//
struct str{
    bool operator() ( Point2f a, Point2f b ){
        if ( a.x != b.x ) 
            return a.x < b.x;
        return a.y <= b.y ;
    }
} comp;

//pacchetto letto quando leggo una nuova immagine
struct image_packet
{
  Mat bgr_image;
  std_msgs::Header header_im;
};

/**************************************************************************************
*
*   FUNZIONI
*
*******************************************************************************************/
void make_control_window();
//void find_and_extract_blob(Mat src_image);
void morphological_filter(Mat& image);
Mat color_filter_image(Mat hsv_image, string color );
Mat Binarize_Image( Mat src_gray);
void calcola_baricentro(Point2f* rect_points, double* cx, double* cy);
void sort_point(Point2f* rect_points);
void find_object(vector<vector<Point> > contours, vector<Point2f>*  boxObj);
void virtual_image_plane_roll(double phi, double u1, double v1, double fv, double fu, double u0, double v0, double* u, double* v);
void virtual_image_plane_pitch(double theta, double u1, double v1, double fv, double fu, double u0, double v0, double* u, double* v);
void obj_detection_function(image_packet bgr_image);
