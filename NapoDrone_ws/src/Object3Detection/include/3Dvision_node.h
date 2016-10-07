#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <math.h>



//Topic Publisher

//Topic Subscriber
ros::Subscriber features_2D_sub;
ros::Subscriber point_cloud_sub;


using namespace cv;
using namespace std;



/**************************************************************************************
*
*   FUNZIONI
*
*******************************************************************************************/

