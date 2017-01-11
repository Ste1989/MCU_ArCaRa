/*********************************************************************************************//**
* @file aruco_mapping.h
*
* Copyright (c)
* Smart Robotic Systems
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/* Author: Jan Bacik */

#ifndef ARUCO_MAPPING_H
#define ARUCO_MAPPING_H

// Standard ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>


// Aruco libraries
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/arucofidmarkers.h>
#include <aruco/boarddetector.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom message
#include <aruco_mapping/ArucoMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
/** \brief Aruco mapping namespace */
namespace aruco_mapping
{

/** \brief Client class for Aruco mapping */  


class ArucoMapping
{
public:
  
  /** \brief Struct to keep marker information */
  struct MarkerInfo
  {

    bool visible;                                   // Marker visibile in actual image?
    int marker_id;                                  // Marker ID
    int previous_marker_id;                         // Used for chaining markers
    geometry_msgs::Pose geometry_msg_to_previous;   // Position with respect to previous marker
    geometry_msgs::Pose geometry_msg_to_world;      // Position with respect to world's origin
    tf::StampedTransform tf_to_previous;            // TF with respect to previous marker
    tf::StampedTransform tf_to_world;               // TF with respect to world's origin
    geometry_msgs::Pose current_camera_pose;        // Position of camera with respect to the marker
    tf::Transform current_camera_tf;                // TF of camera with respect to the marker
  };

public:


  /** \brief Construct a client for EZN64 USB control*/  
  ArucoMapping(ros::NodeHandle *nh);
    
  ~ArucoMapping();

  /** \brief Callback function to handle image processing*/
  void imageCallback(const sensor_msgs::ImageConstPtr &original_image);

  /** Callback per IMU data*/
  void imu_cb(const sensor_msgs::Imu::ConstPtr& imu);



private:
  
  /** \brief Function to parse data from calibration file*/
  bool parseCalibrationFile(std::string filename);

   /** \brief Function to parse data from scenario file*/
   bool parseScenarioFile(std::string scenario_file);

  /** \brief Function to publish all known TFs*/
  void publishTfs(bool world_option);

  /** \brief Function to publish all known markers for visualization purposes*/
  void publishMarker(geometry_msgs::Pose markerPose, int MarkerID, int rank);

  /** \brief Publisher of visualization_msgs::Marker message to "aruco_markers" topic*/
  ros::Publisher marker_visualization_pub_;

  /** \brief Publisher of aruco_mapping::ArucoMarker custom message*/
  ros::Publisher marker_msg_pub_;

  /*sottoscrizione al topic di IMU*/
  ros::Subscriber imu_topic_sub;

  /*publisher immagine di debug*/
  image_transport::Publisher debug_pub;

  /*publisher posa*/
  ros::Publisher pose_cam_pub;
  ros::Publisher pose_body_pub;

  /*publisher */
  ros::Publisher transform_pub; 


  /** \brief Compute TF from marker detector result*/
  tf::Transform arucoMarker2Tf(const aruco::Marker &marker);

  /** \brief Process actual image, detect markers and compute poses */
  bool processImage(cv::Mat input_image,cv::Mat output_image, std_msgs::Header header);

  /**funzione di trasformazione di sistema di riferimento*/
  tf::Transform getTf_camera_world(const cv::Mat &Rvec, const cv::Mat &Tvec);

  /**funzione di trasformazione di sistema di riferimento*/
  tf::Transform getTf_body_world(const cv::Mat &Rvec, const cv::Mat &Tvec);

  /*funzione per passare da quaternione a angoli di eulero*/
  void quaternion_2_euler(double xquat, double yquat, double zquat, double wquat, double& roll, double& pitch, double& yaw);



  //Launch file params
  std::string calib_filename_; 
  std::string scenario_filename_;                    
  std::string space_type_;                        
  float marker_size_;
  int num_of_markers_;
  bool roi_allowed_;
  int  roi_x_;                                      
  int  roi_y_;                                      
  int  roi_w_;                                    
  int  roi_h_;
  bool save_data_on_file;
  std::string file_path_save;
  
  //cose aggiunte
  int id_img;
  double secs_0;

  /*angoli di eulero stimati dal marker piu vicino*/
  double roll_m, pitch_m, yaw_m;

  /*angoli di eulero stimati dalla board*/
  double roll_b, pitch_b, yaw_b;

  /*angoli di eulero stimati dall 'IMU*/
  double roll_imu, pitch_imu, yaw_imu;

  /*vettore P_cam_body__cam*/
  tf::Vector3 P_cam_body__cam;

  /*strutture dati per la board detection*/
  aruco::BoardConfiguration the_board_config;
  std::string board_config;
  aruco::BoardDetector the_board_detector;
  aruco::Board the_board_detected;
  char initialize_pose_old;
  bool pose_valid;
  /** \brief Container holding MarkerInfo data about all detected markers */
  std::vector<MarkerInfo> markers_;
   
  /** \brief Actual TF of camera with respect to world's origin */
  tf::StampedTransform world_position_transform_;
  
  /** \brief Actual Pose of camera with respect to world's origin */
  geometry_msgs::Pose Pose_world_cam__w_Marker;
  geometry_msgs::PoseStamped Pose_world_cam__w;
  geometry_msgs::PoseStamped Pose_world_body__w;
  geometry_msgs::PoseStamped Pose_world_body__w_old;


  /*calibrazione camera*/
  aruco::CameraParameters aruco_calib_params_;

  

  int marker_counter_;
  int marker_counter_previous_;
  int closest_camera_index_;
  int lowest_marker_id_;
  bool first_marker_detected_;
  
  tf::TransformListener *listener_;
  tf::TransformBroadcaster broadcaster_;

  //Consts
   static const int CV_WAIT_KEY = 10;
   static const int CV_WINDOW_MARKER_LINE_WIDTH = 2;

   static const double WAIT_FOR_TRANSFORM_INTERVAL = 2.0;
   static const double BROADCAST_WAIT_INTERVAL = 0.0001;
   static const double INIT_MIN_SIZE_VALUE = 1000000;

   static const double RVIZ_MARKER_HEIGHT = 0.01;
   static const double RVIZ_MARKER_LIFETIME = 3;
   static const double RVIZ_MARKER_COLOR_R = 1.0;
   static const double RVIZ_MARKER_COLOR_G = 1.0;
   static const double RVIZ_MARKER_COLOR_B = 1.0;
   static const double RVIZ_MARKER_COLOR_A = 1.0;

   static const double THIS_IS_FIRST_MARKER = -2;

}; //ArucoMapping class
}  //aruco_mapping namespace



#endif //ARUCO_MAPPING_H
