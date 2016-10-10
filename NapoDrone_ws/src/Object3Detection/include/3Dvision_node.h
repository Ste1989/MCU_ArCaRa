#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include "obj_detection/Features.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <opencv2/opencv.hpp>
#include <math.h>



//Topic Publisher

//Topic Subscriber
ros::Subscriber features_2D_sub;
ros::Subscriber point_cloud_sub;


using namespace cv;
using namespace std;

double t0_sec;


/**************************************************************************************
*
*   STRUTTURE DATI
*
*******************************************************************************************/
struct features_packet
{
	uint sec;
	uint nsec;
	std::vector<double> features;

};
struct points_packet
{
	uint sec;
	uint nsec;
	//# 2D structure of the point cloud. If the cloud is unordered, height is
	//# 1 and width is the length of the point cloud.
	uint height;
	uint width;
	std::vector<sensor_msgs::PointField> fields; //# Describes the channels and their layout in the binary data blob.
	bool is_bigendian;  //# Is this data bigendian?
	uint point_step;  //# Length of a point in bytes
	uint row_step;   // # Length of a row in bytes
	std::vector<unsigned char> data;  //# Actual point data, size is (row_step*height)
	bool is_dense;  // # True if there are no invalid points

};

//buffer di ricezione dei pacchetti
std::vector<points_packet> buffer_points_packet;
std::vector<features_packet> buffer_features_packet;


/**************************************************************************************
*
*   FUNZIONI
*
*******************************************************************************************/
void  check_syncronization_pkg();
void compute_3D_data(features_packet pkg_features, points_packet pkg_points);
