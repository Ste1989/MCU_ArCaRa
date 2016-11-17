/*********************************************************************************************//**
* @file aruco_mapping.cpp
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

#ifndef ARUCO_MAPPING_CPP
#define ARUCO_MAPPING_CPP

#include <aruco_mapping.h>
#include <fstream>

namespace aruco_mapping
{

ArucoMapping::ArucoMapping(ros::NodeHandle *nh) :
  listener_ (new tf::TransformListener),  // Initialize TF Listener  
  num_of_markers_ (10),                   // Number of used markers
  marker_size_(0.1),                      // Marker size in m
  calib_filename_("empty"),               // Calibration filepath
  space_type_ ("plane"),                  // Space type - 2D plane 
  roi_allowed_ (false),                   // ROI not allowed by default
  first_marker_detected_(false),          // First marker not detected by defualt
  lowest_marker_id_(-1),                  // Lowest marker ID
  marker_counter_(0),                     // Reset marker counter
  closest_camera_index_(0)                // Reset closest camera index 
  
{
  double temp_marker_size;  
  
  //Parse params from launch file 
  nh->getParam("/aruco_mapping/calibration_file", calib_filename_);
  nh->getParam("/aruco_mapping/scenario_file", scenario_filename_);
  nh->getParam("/aruco_mapping/marker_size", temp_marker_size); 
  nh->getParam("/aruco_mapping/num_of_markers", num_of_markers_);
  nh->getParam("/aruco_maping/space_type",space_type_);
  nh->getParam("/aruco_mapping/roi_allowed",roi_allowed_);
  nh->getParam("/aruco_mapping/roi_x",roi_x_);
  nh->getParam("/aruco_mapping/roi_y",roi_y_);
  nh->getParam("/aruco_mapping/roi_w",roi_w_);
  nh->getParam("/aruco_mapping/roi_h",roi_h_);
     
  // Double to float conversion
  marker_size_ = float(temp_marker_size);
  
  if(calib_filename_ == "empty")
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  else
  {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_ );
    ROS_INFO_STREAM("Number of markers: " << num_of_markers_);
    ROS_INFO_STREAM("Marker Size: " << marker_size_);
    ROS_INFO_STREAM("Type of space: " << space_type_);
    ROS_INFO_STREAM("ROI allowed: " << roi_allowed_);
    ROS_INFO_STREAM("ROI x-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI y-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI width: "  << roi_w_);
    ROS_INFO_STREAM("ROI height: " << roi_h_);      
  }
    
  //ROS publishers
  marker_msg_pub_           = nh->advertise<aruco_mapping::ArucoMarker>("aruco_poses",1);
  marker_visualization_pub_ = nh->advertise<visualization_msgs::Marker>("aruco_markers",1);
          
  //Parse data from calibration file
  parseCalibrationFile(calib_filename_);

  //Initialize OpenCV window
  //cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);       
      
  //Resize marker container
  markers_.resize(num_of_markers_);
  
  // Default markers_ initialization
  for(size_t i = 0; i < num_of_markers_;i++)
  {
    markers_[i].previous_marker_id =-1;
    markers_[i].visible = false;
    
  }

  //parsing del file contenente lo scenrio
  parseScenarioFile(scenario_filename_);
  

}

ArucoMapping::~ArucoMapping()
{
 delete listener_;
}


bool
ArucoMapping::parseScenarioFile(std::string scenario_filename_)
{

  const char* filename = scenario_filename_.c_str();
  FILE* fd;
  fd = fopen(filename,"rb");
  if( fd == NULL ) {
    ROS_ERROR("Scenario data NOT FOUND ");
    return false;
  }
  int id;
  float px; 
  float py;
  while(!feof(fd)) 
  {
    fscanf(fd,"%d %f %f",&id,&px,&py );
    markers_[id].marker_id = id;
    markers_[id].geometry_msg_to_world.position.x = px;
    markers_[id].geometry_msg_to_world.position.y = py;
    markers_[id].geometry_msg_to_world.position.z = 0; // we assume a plane

    markers_[id].geometry_msg_to_world.orientation.x = 0; // we assume a plane
    markers_[id].geometry_msg_to_world.orientation.y = 0; // we assume a plane
    markers_[id].geometry_msg_to_world.orientation.z = 0; // we assume a plane
    markers_[id].geometry_msg_to_world.orientation.w = 1; // we assume a plane    
  }

  fclose(fd);
  ROS_INFO_STREAM("Scenario data loaded successfully");
  return true;
}



bool
ArucoMapping::parseCalibrationFile(std::string calib_filename)
{
  sensor_msgs::CameraInfo camera_calibration_data;
  std::string camera_name = "camera";

  camera_calibration_parsers::readCalibrationIni(calib_filename, camera_name, camera_calibration_data);

  // Alocation of memory for calibration data
  cv::Mat  *intrinsics       = new(cv::Mat)(3, 3, CV_64F);
  cv::Mat  *distortion_coeff = new(cv::Mat)(5, 1, CV_64F);
  cv::Size *image_size       = new(cv::Size);

  image_size->width = camera_calibration_data.width;
  image_size->height = camera_calibration_data.height;

  for(size_t i = 0; i < 3; i++)
    for(size_t j = 0; j < 3; j++)
    intrinsics->at<double>(i,j) = camera_calibration_data.K.at(3*i+j);

  for(size_t i = 0; i < 5; i++)
    distortion_coeff->at<double>(i,0) = camera_calibration_data.D.at(i);

  ROS_DEBUG_STREAM("Image width: " << image_size->width);
  ROS_DEBUG_STREAM("Image height: " << image_size->height);
  ROS_DEBUG_STREAM("Intrinsics:" << std::endl << *intrinsics);
  ROS_DEBUG_STREAM("Distortion: " << *distortion_coeff);


  //Load parameters to aruco_calib_param_ for aruco detection
  aruco_calib_params_.setParams(*intrinsics, *distortion_coeff, *image_size);

  //Simple check if calibration data meets expected values
  if ((intrinsics->at<double>(2,2) == 1) && (distortion_coeff->at<double>(0,4) == 0))
  {
    ROS_INFO_STREAM("Calibration data loaded successfully");
    return true;
  }
  else
  {
    ROS_WARN("Wrong calibration data, check calibration file and filepath");
    return false;
  }
}

void
ArucoMapping::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }
  
  // sensor_msgs::Image to OpenCV Mat structure
  cv::Mat I = cv_ptr->image;
  
  // region of interest
  if(roi_allowed_==true)
    I = cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

  //Marker detection
  processImage(I,I);
  
  // Show image
  //cv::imshow("Mono8", I);
  //cv::waitKey(10);  
}


bool

ArucoMapping::processImage(cv::Mat input_image,cv::Mat output_image)
{
  aruco::MarkerDetector Detector;
  std::vector<aruco::Marker> temp_markers;

  //Set visibility flag to false for all markers
  for(size_t i = 0; i < num_of_markers_; i++)
      markers_[i].visible = false;

  // Save previous marker count


  // Detect markers
  Detector.detect(input_image,temp_markers,aruco_calib_params_,marker_size_);
    
  // If no marker found, print statement
  if(temp_markers.size() == 0)
    ROS_DEBUG("No marker found!");

  //------------------------------------------------------
  // FIRST MARKER DETECTED
  //------------------------------------------------------


  //------------------------------------------------------
  // FOR EVERY MARKER DO SEEN
  //------------------------------------------------------
  for(size_t i = 0; i < temp_markers.size();i++)
  {
    int index;
    int current_marker_id = temp_markers[i].id;

    //Draw marker convex, ID, cube and axis
    temp_markers[i].draw(output_image, cv::Scalar(0,0,255),2);
    aruco::CvDrawingUtils::draw3dCube(output_image,temp_markers[i], aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(output_image,temp_markers[i], aruco_calib_params_);

 

    // Change visibility flag of new marker
    for(size_t j = 0;j < num_of_markers_; j++)
    {
      for(size_t k = 0;k < temp_markers.size(); k++)
      {
        if(markers_[j].marker_id == temp_markers[k].id)
          markers_[j].visible = true;
      }   
    }

    //------------------------------------------------------
    // For existing marker do
    //------------------------------------------------------
    if(current_marker_id < num_of_markers_)
    {
      
      markers_[current_marker_id].current_camera_tf = arucoMarker2Tf(temp_markers[i]);
      markers_[current_marker_id].current_camera_tf = markers_[current_marker_id].current_camera_tf.inverse();

      const tf::Vector3 marker_origin = markers_[current_marker_id].current_camera_tf.getOrigin();
      markers_[current_marker_id].current_camera_pose.position.x = marker_origin.getX();
      markers_[current_marker_id].current_camera_pose.position.y = marker_origin.getY();
      markers_[current_marker_id].current_camera_pose.position.z = marker_origin.getZ();
      
      //cout << markers_[current_marker_id].current_camera_pose.position.x << endl;  
      //cout << markers_[current_marker_id].current_camera_pose.position.y << endl;  
      //cout << markers_[current_marker_id].current_camera_pose.position.z << endl;


      const tf::Quaternion marker_quaternion = markers_[current_marker_id].current_camera_tf.getRotation();
      markers_[current_marker_id].current_camera_pose.orientation.x = marker_quaternion.getX();
      markers_[current_marker_id].current_camera_pose.orientation.y = marker_quaternion.getY();
      markers_[current_marker_id].current_camera_pose.orientation.z = marker_quaternion.getZ();
      markers_[current_marker_id].current_camera_pose.orientation.w = marker_quaternion.getW();
    }

    //------------------------------------------------------
    // For new marker do
    //------------------------------------------------------

    //------------------------------------------------------
    // Compute global position of new marker
    //------------------------------------------------------

  }

  //------------------------------------------------------
  // Compute which of visible markers is the closest to the camera
  //------------------------------------------------------
  bool any_markers_visible=false;
  int num_of_visible_markers=0;


  double minimal_distance = INIT_MIN_SIZE_VALUE;
  for(int k = 0; k < num_of_markers_; k++)
  {
    double a,b,c,size;

    // If marker is visible, distance is calculated
    if(markers_[k].visible==true)
    {
      a = markers_[k].current_camera_pose.position.x;
      b = markers_[k].current_camera_pose.position.y;
      c = markers_[k].current_camera_pose.position.z;
      size = std::sqrt((a * a) + (b * b) + (c * c));
      if(size < minimal_distance)
      {
        minimal_distance = size;
        closest_camera_index_ = k;
      }

      any_markers_visible = true;
      num_of_visible_markers++;
    }
  }
  

  //------------------------------------------------------
  // Publish all known markers
  //------------------------------------------------------
 
  //publishTfs(true);

  //------------------------------------------------------
  // Compute global camera pose
  //------------------------------------------------------
  if(any_markers_visible == true)
  {
    

    //devo calcolare la posizione della camera nel sistema di riferimento world

    //prendo la posizione della camera rispetto al marker piu vicino
    tf::Vector3 camera_origin = markers_[closest_camera_index_].current_camera_tf.getOrigin();
    geometry_msgs::Pose marker_origin = markers_[closest_camera_index_].geometry_msg_to_world;
    //supponendo di aver scelto come in questo caso i sistemi current_camera_tf e world allineati 
    //e supponendo di aver montato i marker allineati non si deve ruotare il sistema di riferimento
    tf::Quaternion camera_quaternion = markers_[closest_camera_index_].current_camera_tf.getRotation();



    // Saving TF to Pose
    world_position_geometry_msg_.position.x = camera_origin.getX() + marker_origin.position.x;
    world_position_geometry_msg_.position.y = camera_origin.getY() + marker_origin.position.y;
    world_position_geometry_msg_.position.z = camera_origin.getZ() + marker_origin.position.z;

    //da controllare che non si sia bisogno di nessuna rotazione
    world_position_geometry_msg_.orientation.x = camera_quaternion.getX();
    world_position_geometry_msg_.orientation.y = camera_quaternion.getY();
    world_position_geometry_msg_.orientation.z = camera_quaternion.getZ();
    world_position_geometry_msg_.orientation.w = camera_quaternion.getW();
  }

  //------------------------------------------------------
  // Publish all known markers
  //------------------------------------------------------
  
  publishTfs(true);

  //------------------------------------------------------
  // Publish custom marker message
  //------------------------------------------------------
  aruco_mapping::ArucoMarker marker_msg;

  if((any_markers_visible == true))
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "world";
    marker_msg.marker_visibile = true;
    marker_msg.num_of_visible_markers = num_of_visible_markers;
    marker_msg.global_camera_pose = world_position_geometry_msg_;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();

    //stampo su file la posizione della camera stimata nel fram world
    /*FILE* fd;
    fd = fopen("/home/robot/camera_pose.txt", "a");
    fprintf(fd, "%f", world_position_geometry_msg_.position.x);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", world_position_geometry_msg_.position.y);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", world_position_geometry_msg_.position.z);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", world_position_geometry_msg_.orientation.x);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", world_position_geometry_msg_.orientation.y);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f", world_position_geometry_msg_.orientation.z);
    fprintf(fd, "%s", " ");
    fprintf(fd, "%f\n", world_position_geometry_msg_.orientation.w);
    fclose(fd);*/	

    for(size_t j = 0; j < num_of_markers_; j++)
    {
      if(markers_[j].visible == true)
      {
        marker_msg.marker_ids.push_back(markers_[j].marker_id);
        marker_msg.global_marker_poses.push_back(markers_[j].geometry_msg_to_world);       
      }
    }
  }
  else
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "world";
    marker_msg.num_of_visible_markers = num_of_visible_markers;
    marker_msg.marker_visibile = false;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();
  }

  // Publish custom marker msg
  marker_msg_pub_.publish(marker_msg);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishTfs(bool world_option)
{
  for(int i = 0; i < num_of_markers_; i++)
  {

   //codice che funziona con i marker variabili
   /*// Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << i;
    // Older marker - or World
    std::stringstream marker_tf_id_old;
    if(i == 0)
      marker_tf_id_old << "world";
    else
      marker_tf_id_old << "marker_" << markers_[i].previous_marker_id;
    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].tf_to_previous,ros::Time::now(),marker_tf_id_old.str(),marker_tf_id.str()));

    // Position of camera to its marker
    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << i;
    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].current_camera_tf,ros::Time::now(),marker_tf_id.str(),camera_tf_id.str()));

    if(world_option == true)
    {
      // Global position of marker TF
      std::stringstream marker_globe;
      marker_globe << "marker_globe_" << i;
      broadcaster_.sendTransform(tf::StampedTransform(markers_[i].tf_to_world,ros::Time::now(),"world",marker_globe.str()));
    }
   */ 
    if(markers_[i].visible == true)
    {  
      // Cubes for RVIZ - markers
      publishMarker(markers_[i].geometry_msg_to_previous,markers_[i].marker_id,i);
      publishMarker(markers_[i].geometry_msg_to_world,markers_[i].marker_id,i);
    }
  }

  // Global Position of object
  //if(world_option == true)
  //  broadcaster_.sendTransform(tf::StampedTransform(world_position_transform_,ros::Time::now(),"world","camera_position"));
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index)
{
  visualization_msgs::Marker vis_marker;

  
  vis_marker.header.frame_id = "world";
 

  vis_marker.header.stamp = ros::Time::now();
  vis_marker.ns = "basic_shapes";
  vis_marker.id = marker_id;
  vis_marker.type = visualization_msgs::Marker::CUBE;
  vis_marker.action = visualization_msgs::Marker::ADD;

  vis_marker.pose = marker_pose;
  vis_marker.scale.x = marker_size_;
  vis_marker.scale.y = marker_size_;
  vis_marker.scale.z = RVIZ_MARKER_HEIGHT;

  vis_marker.color.r = RVIZ_MARKER_COLOR_R;
  vis_marker.color.g = RVIZ_MARKER_COLOR_G;
  vis_marker.color.b = RVIZ_MARKER_COLOR_B;
  vis_marker.color.a = RVIZ_MARKER_COLOR_A;

  vis_marker.lifetime = ros::Duration(RVIZ_MARKER_LIFETIME);

  marker_visualization_pub_.publish(vis_marker);
}

////////////////////////////////////////////////////////////////////////////////////////////////

tf::Transform
ArucoMapping::arucoMarker2Tf(const aruco::Marker &marker)
{
  cv::Mat marker_rotation(3,3, CV_32FC1);
  cv::Rodrigues(marker.Rvec, marker_rotation);
  cv::Mat marker_translation = marker.Tvec;

  cv::Mat rotate_to_ros(3,3,CV_32FC1);
  rotate_to_ros.at<float>(0,0) = -1.0;
  rotate_to_ros.at<float>(0,1) = 0;
  rotate_to_ros.at<float>(0,2) = 0;
  rotate_to_ros.at<float>(1,0) = 0;
  rotate_to_ros.at<float>(1,1) = 0;
  rotate_to_ros.at<float>(1,2) = 1.0;
  rotate_to_ros.at<float>(2,0) = 0.0;
  rotate_to_ros.at<float>(2,1) = 1.0;
  rotate_to_ros.at<float>(2,2) = 0.0;

  marker_rotation = marker_rotation * rotate_to_ros.t();

  // Origin solution
  tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2),
                              marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2),
                              marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2));

  tf::Vector3 marker_tf_tran(marker_translation.at<float>(0,0),
                             marker_translation.at<float>(1,0),
                             marker_translation.at<float>(2,0));

  return tf::Transform(marker_tf_rot, marker_tf_tran);
}



}  //aruco_mapping

#endif  //ARUCO_MAPPING_CPP
