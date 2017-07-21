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


namespace aruco_mapping
{



/********************************************************************************************/
/*                                                                                         */
/*    INIT                                                                                 */
/*                                                                                         */
/*******************************************************************************************/
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
  nh->param<std::string>("/aruco_mapping/board_config", board_config, "boardConfiguration.yml");
  nh->param<bool>("/aruco_mapping/save_data", save_data_on_file , false);
  nh->param<std::string>("/aruco_mapping/file_path_save",file_path_save , "");
  nh->param<double>("/aruco_mapping/Px_cam_body_cam",P_cam_body__cam[0], 0);
  nh->param<double>("/aruco_mapping/Py_cam_body_cam",P_cam_body__cam[1] , 0);
  nh->param<double>("/aruco_mapping/Pz_cam_body_cam",P_cam_body__cam[2], 0);
  
  

  if(save_data_on_file)
  {
    //initialize file

    //imu data from callback
    std::string str_path  = file_path_save + "imu_data.txt";
    FILE* fd;
    fd = fopen(str_path.c_str(), "w");
    fclose(fd);
    //camera pose from vision
    str_path  = file_path_save + "camera_pose.txt";
    fd = fopen(str_path.c_str(), "w");
    fclose(fd);

  }

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
    ROS_INFO_STREAM("Save data: " << save_data_on_file);      
    ROS_INFO_STREAM("File path save: " << file_path_save); 
    ROS_INFO_STREAM("Px_cam_body_cam: " << P_cam_body__cam[0]);
    ROS_INFO_STREAM("Py_cam_body_cam: " << P_cam_body__cam[1]);
    ROS_INFO_STREAM("Pz_cam_body_cam: " << P_cam_body__cam[2]);
  }
    
  //ROS publishers
  image_transport::ImageTransport it(*nh);
  marker_msg_pub_           = nh->advertise<aruco_mapping::ArucoMarker>("/aruco/pose_cam_from_marker",1);
  marker_visualization_pub_ = nh->advertise<visualization_msgs::Marker>("/aruco/marker_detected",1);
  debug_pub = it.advertise("/aruco/img_detect_marker", 1);
  pose_cam_pub = nh->advertise<geometry_msgs::PoseStamped>("/aruco/pose_cam_from_board", 100);
  pose_body_pub = nh->advertise<geometry_msgs::PoseStamped>("/aruco/pose_body_from_board", 100);
  transform_pub = nh->advertise<geometry_msgs::TransformStamped>("/aruco/transform", 100);
    
  //service client
  get_time_sec0 = nh->serviceClient<autopilot_manager::init_time>("/get_time_t0");

  //Parse data from calibration file
  parseCalibrationFile(calib_filename_);

  //Initialize OpenCV window
  cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);       
      
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
  

  //board config
  the_board_config.readFromFile(board_config.c_str());
  ROS_INFO("Letta configurazione board %s", board_config.c_str());

   //tempo 0
  //per inizializzare secs_0 richiamo il client
  autopilot_manager::init_time srv_msg;
  bool res = get_time_sec0.call(srv_msg);

  if(res)
  {
      secs_0 = srv_msg.response.sec0;
  }
  else
  {   
      ROS_WARN("ATTENZIONE TEMPO NON INIZIALIZZATO CORRETTAMENTE");
      secs_0 = ros::Time::now().toSec();    
  }

  //initiliazer pose old
  initialize_pose_old = 1;
  

  /*inizializzazione file*/
  if(save_data_on_file)
  {

      FILE* fd;
      std::string str_path  = file_path_save + "camera_pose_valid.txt";
      fd = fopen(str_path.c_str(), "w");
      fclose(fd);

      str_path  = file_path_save + "camera_pose.txt";
      fd = fopen(str_path.c_str(), "w");
      fclose(fd);
  }


}

ArucoMapping::~ArucoMapping()
{
 delete listener_;
}

/********************************************************************************************/
/*                                                                                         */
/*    PARSE SCENARIO                                                                       */
/*                                                                                         */
/*******************************************************************************************/
bool ArucoMapping::parseScenarioFile(std::string scenario_filename_)
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


/********************************************************************************************/
/*                                                                                         */
/*    PARSE CALIBRATION FILE                                                               */
/*                                                                                         */
/*******************************************************************************************/
bool ArucoMapping::parseCalibrationFile(std::string calib_filename)
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
/********************************************************************************************/
/*                                                                                         */
/*    QUATERNION_2_EULER                                                                    */
/*                                                                                         */
/*******************************************************************************************/
void ArucoMapping::quaternion_2_euler(double xquat, double yquat, double zquat, double wquat, double& roll, double& pitch, double& yaw)
{
  //Trasformo le corrdinate SVO in coordinate frame camera.
  double r11 = wquat*wquat + xquat*xquat - yquat*yquat - zquat*zquat;
  double r12 = 2*(xquat*yquat - wquat*zquat);
  double r13 = 2*(zquat*xquat + wquat*yquat);
  double r21 =  2*(xquat*yquat + wquat*zquat);
  double r22 = wquat*wquat - xquat*xquat + yquat*yquat - zquat*zquat;
  double r23 = 2*(yquat*zquat - wquat*xquat);
  double r31 = 2*(zquat*xquat - wquat*yquat);
  double r32 = 2*(yquat*zquat + wquat*xquat);
  double r33 = wquat*wquat - xquat*xquat - yquat*yquat + zquat*zquat;
  //Scrivo la trasposta:
  double rt11,rt12,rt13,rt21,rt22,rt23,rt31,rt32,rt33;
  rt11 = r11;
  rt12 = r21;
  rt13 = r31;
  rt21 = r12;
  rt22 = r22;
  rt23 = r32;
  rt31 = r13;
  rt32 = r23;
  rt33 = r33;
  //calcolo angoli di eulero
  roll = atan2(rt23,rt33);
  pitch = -asin(rt13);
  yaw = atan2(rt12,rt11);
}
/********************************************************************************************/
/*                                                                                         */
/*    getTF CAMERA in WORLD                                                                 */
/*                                                                                         */
/*******************************************************************************************/
tf::Transform ArucoMapping::getTf_camera_world(const cv::Mat &Rvec, const cv::Mat &Tvec)
{
  cv::Mat rot(3, 3, CV_32FC1);
  cv::Rodrigues(Rvec, rot);

  //cv::Mat rotate_to_sys(3, 3, CV_32FC1);
  /**
  /* Fixed the rotation to meet the ROS system
  /* Doing a basic rotation around X with theta=PI
  /* By Sahloul
  /* See http://en.wikipedia.org/wiki/Rotation_matrix for details
  */

  //  1 0 0
  //  0 -1  0
  //  0 0 -1
  /*rotate_to_sys.at<float>(0,0) = 1.0;
  rotate_to_sys.at<float>(0,1) = 0.0;
  rotate_to_sys.at<float>(0,2) = 0.0;
  rotate_to_sys.at<float>(1,0) = 0.0;
  rotate_to_sys.at<float>(1,1) = -1.0;
  rotate_to_sys.at<float>(1,2) = 0.0;
  rotate_to_sys.at<float>(2,0) = 0.0;
  rotate_to_sys.at<float>(2,1) = 0.0;
  rotate_to_sys.at<float>(2,2) = -1.0;*/

  //MODIFICATO, LEVATO LA ROTAZONE DI 180 SU X
  //rot = rot*rotate_to_sys.t();

  tf::Matrix3x3 R_world__cam(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
    rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
    rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

  tf::Vector3 P_cam_world__cam(Tvec.at<float>(0,0), Tvec.at<float>(1,0), Tvec.at<float>(2,0));
  tf::Vector3 P_world_cam__cam = -P_cam_world__cam;

  //rotazione del vettore nel sistema di riferiemento world
  tf::Vector3 P_world_cam__world  = R_world__cam.transpose() * P_world_cam__cam;

  return tf::Transform(R_world__cam, P_world_cam__world);
}
/********************************************************************************************/
/*                                                                                         */
/*    getTF CAMERA in WORLD                                                                 */
/*                                                                                         */
/*******************************************************************************************/
tf::Transform ArucoMapping::getTf_body_world(const cv::Mat &Rvec, const cv::Mat &Tvec)
{
  cv::Mat rot(3, 3, CV_32FC1);
  cv::Rodrigues(Rvec, rot);


  tf::Matrix3x3 R_world__cam(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
    rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
    rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

  tf::Vector3 P_cam_world__cam(Tvec.at<float>(0,0), Tvec.at<float>(1,0), Tvec.at<float>(2,0));
  tf::Vector3 P_world_cam__cam = -P_cam_world__cam;

  //devo sommarci il vettore tra cam e body
  tf::Vector3 P_world_body__cam = P_world_cam__cam + P_cam_body__cam;
 
  //rotazione del vettore nel sistema di riferiemento world
  tf::Vector3 P_world_body__world  = R_world__cam.transpose() * P_world_body__cam;

  return tf::Transform(R_world__cam, P_world_body__world);
}
/********************************************************************************************/
/*                                                                                         */
/*    CALBACK IMU                                                                           */
/*                                                                                         */
/*******************************************************************************************/
void ArucoMapping::imu_cb(const sensor_msgs::Imu::ConstPtr& imu)
{
    double roll, pitch, yaw;
    quaternion_2_euler(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w, roll, pitch, yaw);
    roll_imu = roll;
    pitch_imu = pitch;
    yaw_imu = yaw;
      
    if(save_data_on_file)
    {

      double secs = imu->header.stamp.sec;

      if (secs_0 > secs) 
        secs_0 = secs;
    
      secs = secs + (double(imu->header.stamp.nsec)/pow(10,9));
      std::string str_path  = file_path_save + "imu_data.txt";
      FILE* fd;
      fd = fopen(str_path.c_str(), "a");
      fprintf(fd, "%f", secs -  secs_0);
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", roll_imu); //2
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", pitch_imu); //3
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f\n", yaw_imu); //4 
      fclose(fd);

    }

  
}
/********************************************************************************************/
/*                                                                                         */
/*    IMAGE CALLBACK                                                                   */
/*                                                                                         */
/*******************************************************************************************/
void ArucoMapping::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
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
  processImage(I,I,original_image->header);
  
  // Show image
  cv::imshow("Mono8", I);
  cv::waitKey(10);  
}

/********************************************************************************************/
/*                                                                                         */
/*    PROCESS IMAGE                                                                        */
/*                                                                                         */
/*******************************************************************************************/
bool ArucoMapping::processImage(cv::Mat input_image,cv::Mat output_image, std_msgs::Header header)
{
  ROS_INFO("**********************NEW IMAGE*******************************");
  static tf::TransformBroadcaster br;
  //detector
  aruco::MarkerDetector Detector;
  std::vector<aruco::Marker> temp_markers;

  //Set visibility flag to false for all markers
  for(size_t i = 0; i < num_of_markers_; i++)
      markers_[i].visible = false;

  //A) Detect markers
  Detector.detect(input_image,temp_markers,aruco_calib_params_,marker_size_);
  // If no marker found, print statement
  if(temp_markers.size() == 0)
    ROS_DEBUG("No marker found!");
  //B)Detect board
  float probDetect = the_board_detector.detect(temp_markers, the_board_config, the_board_detected, aruco_calib_params_,marker_size_);

  
  //se è stata trovata la board
  if (probDetect > 0.0)
  {
    
    //ROS_INFO("P_cam_world__cam: %f  %f %f",the_board_detected.Tvec.at<float>(0,0),the_board_detected.Tvec.at<float>(1,0),the_board_detected.Tvec.at<float>(2,0) );
    //ROS_INFO("R_world__cam (RPY): %f  %f %f", the_board_detected.Rvec.at<float>(0,0)*180/M_PI , the_board_detected.Rvec.at<float>(1,0)*180/M_PI, the_board_detected.Rvec.at<float>(2,0)*180/M_PI  );
  
    //trasformazione da P_cam_world__CAM a P_cam_world__WORLD
    tf::Transform P_R_world_cam__world = getTf_camera_world(the_board_detected.Rvec, the_board_detected.Tvec);
    //trasformazione da P_cam_world_CAM a P_body_world_WORLD
    tf::Transform P_R_world_body__world = getTf_body_world(the_board_detected.Rvec, the_board_detected.Tvec);

    //creazione del messaggio "stamped"
    tf::StampedTransform P_R_world_cam__w_stamped(P_R_world_cam__world, header.stamp, header.frame_id, "world");
    br.sendTransform(P_R_world_cam__w_stamped);

    //cam
    tf::poseTFToMsg(P_R_world_cam__world, Pose_world_cam__w.pose);
    Pose_world_cam__w.header.frame_id = "world";
    Pose_world_cam__w.header.stamp = header.stamp;
    //body
    tf::poseTFToMsg(P_R_world_body__world, Pose_world_body__w.pose);
    Pose_world_body__w.header.frame_id = "world";
    Pose_world_body__w.header.stamp = header.stamp;

    //B1)validazione del dato prima di pubblicarlo
    //se initialize_pose_old è a 1 devo inizializzare la posa old a quella vecchia e tornare
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(initialize_pose_old)
    {
      Pose_world_body__w_old = Pose_world_body__w;
      initialize_pose_old = 0;
      pose_valid = true;
    }
    else
    {
      //qui devo controlloare se il dato è valido
      if(abs(Pose_world_body__w.pose.position.x - Pose_world_body__w_old.pose.position.x) > 0.10 || abs(Pose_world_body__w.pose.position.y - Pose_world_body__w_old.pose.position.y) > 0.10)
      {
        ROS_WARN("POSA SCARTATA");
        //al passo successivo devo riniziallizzare pose
        initialize_pose_old = 1;
        //ritorno false
        pose_valid =  false;
      }
      else
      {
        Pose_world_body__w_old = Pose_world_body__w;
        initialize_pose_old = 0;
        pose_valid =  true;
      }

    } 
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //B2)Pubblicazione su topic della posa ricavata dalla board
    //camera
    if(pose_valid)
    {
      //cam
      pose_cam_pub.publish(Pose_world_cam__w);
      //body
      pose_body_pub.publish(Pose_world_body__w);
    }
    quaternion_2_euler(Pose_world_cam__w.pose.orientation.x, Pose_world_cam__w.pose.orientation.y, Pose_world_cam__w.pose.orientation.z, Pose_world_cam__w.pose.orientation.w, roll_b, pitch_b, yaw_b);
    
    ROS_INFO("BOARD CAMERA POSE ESTIMATION:");
    ROS_INFO("P_world_cam_w: %f  %f %f",  Pose_world_cam__w.pose.position.x,Pose_world_cam__w.pose.position.y,Pose_world_cam__w.pose.position.z);
    ROS_INFO("R_world__cam (RPY): %f  %f %f",roll_b*180/M_PI ,pitch_b*180/M_PI ,yaw_b*180/M_PI);

    ROS_INFO("BOARD BODY POSE ESTIMATION:");
    ROS_INFO("P_world_body_w: %f  %f %f",  Pose_world_body__w.pose.position.x,Pose_world_body__w.pose.position.y,Pose_world_body__w.pose.position.z);
    
    geometry_msgs::TransformStamped transformMsg;
    tf::transformStampedTFToMsg(P_R_world_cam__w_stamped, transformMsg);
    transform_pub.publish(transformMsg);


    if (probDetect > 0.0) aruco::CvDrawingUtils::draw3dAxis(output_image, the_board_detected, aruco_calib_params_);
    
    if(debug_pub.getNumSubscribers() > 0)
    {
      //show also the internal image resulting from the threshold operation
      cv_bridge::CvImage debug_msg;
      debug_msg.header.frame_id = header.frame_id;
      debug_msg.header.stamp = header.stamp;
      debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
      debug_msg.image = Detector.getThresholdedImage();
      debug_pub.publish(debug_msg.toImageMsg());
    }

  }//fine if board detect
  


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
      
      const tf::Quaternion marker_quaternion = markers_[current_marker_id].current_camera_tf.getRotation();
      markers_[current_marker_id].current_camera_pose.orientation.x = marker_quaternion.getX();
      markers_[current_marker_id].current_camera_pose.orientation.y = marker_quaternion.getY();
      markers_[current_marker_id].current_camera_pose.orientation.z = marker_quaternion.getZ();
      markers_[current_marker_id].current_camera_pose.orientation.w = marker_quaternion.getW();
    }
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
    Pose_world_cam__w_Marker.position.x = camera_origin.getX() + marker_origin.position.x;
    Pose_world_cam__w_Marker.position.y = camera_origin.getY() + marker_origin.position.y;
    Pose_world_cam__w_Marker.position.z = camera_origin.getZ() + marker_origin.position.z;

    //da controllare che non si sia bisogno di nessuna rotazione
    Pose_world_cam__w_Marker.orientation.x = camera_quaternion.getX();
    Pose_world_cam__w_Marker.orientation.y = camera_quaternion.getY();
    Pose_world_cam__w_Marker.orientation.z = camera_quaternion.getZ();
    Pose_world_cam__w_Marker.orientation.w = camera_quaternion.getW();

    
    quaternion_2_euler(Pose_world_cam__w_Marker.orientation.x, Pose_world_cam__w_Marker.orientation.y, Pose_world_cam__w_Marker.orientation.z, Pose_world_cam__w_Marker.orientation.w, roll_m, pitch_m, yaw_m);
    ROS_INFO("MARKER POSE ESTIMATION:");
    ROS_INFO("P_world_cam_w MARKER: %f  %f %f",  Pose_world_cam__w_Marker.position.x, Pose_world_cam__w_Marker.position.y, Pose_world_cam__w_Marker.position.z);
    ROS_INFO("R_cam__world MARKER (RPY): %f  %f %f",roll_m*180/M_PI ,pitch_m*180/M_PI ,yaw_m*180/M_PI);



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
    marker_msg.global_camera_pose = Pose_world_cam__w_Marker;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();

   if(save_data_on_file)
    {

      //stampo su file la posizione della camera stimata nel fram world
      double secs = header.stamp.sec;
      secs = secs + (double(header.stamp.nsec)/pow(10,9));
      
      std::string str_path  = file_path_save + "camera_pose.txt";
      FILE* fd;
      fd = fopen(str_path.c_str(), "a");
      fprintf(fd, "%f", secs -  secs_0);
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w.pose.position.x); //2
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w.pose.position.y); //3
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w.pose.position.z); //4 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w_Marker.position.x);//5
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w_Marker.position.y); //6
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w_Marker.position.z); //7
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_body__w.pose.position.x); //8
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_body__w.pose.position.y); //9
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_body__w.pose.position.z); //10 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", roll_b); //11
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", pitch_b); //12
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", yaw_b); //13
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", roll_m); //14
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", pitch_m); //15
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", yaw_m); //16
      fprintf(fd, "%s", " ");
      fprintf(fd, "%i\n", pose_valid); //17
      fclose(fd); 
    }
     if(save_data_on_file && pose_valid)
    {

      //stampo su file la posizione della camera stimata nel fram world
      double secs = header.stamp.sec;
      secs = secs + (double(header.stamp.nsec)/pow(10,9));
      
      std::string str_path  = file_path_save + "camera_pose_valid.txt";
      FILE* fd;
      fd = fopen(str_path.c_str(), "a");
      fprintf(fd, "%f", secs -  secs_0);
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w.pose.position.x); //2
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w.pose.position.y); //3
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w.pose.position.z); //4 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w_Marker.position.x);//5
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w_Marker.position.y); //6
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_cam__w_Marker.position.z); //7
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_body__w.pose.position.x); //8
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_body__w.pose.position.y); //9
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", Pose_world_body__w.pose.position.z); //10 
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", roll_b); //11
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", pitch_b); //12
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", yaw_b); //13
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", roll_m); //14
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", pitch_m); //15
      fprintf(fd, "%s", " ");
      fprintf(fd, "%f", yaw_m); //16
      fprintf(fd, "%s", " ");
      fprintf(fd, "%i\n", pose_valid); //17
      fclose(fd); 
    }
    

    //pubblico informazioni sui marker
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

/********************************************************************************************/
/*                                                                                         */
/*    publishTfs                                                                           */
/*                                                                                         */
/*******************************************************************************************/

void ArucoMapping::publishTfs(bool world_option)
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

/********************************************************************************************/
/*                                                                                         */
/*    publishMarker                                                                        */
/*                                                                                         */
/*******************************************************************************************/

void ArucoMapping::publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index)
{
  visualization_msgs::Marker vis_marker;

  
  vis_marker.header.frame_id = "world";
 

  vis_marker.header.stamp = ros::Time::now();
  //visMarker.header = transformMsg.header;
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


/********************************************************************************************/
/*                                                                                         */
/*    arucoMarker2Tf                                                                        */
/*                                                                                         */
/*******************************************************************************************/
tf::Transform ArucoMapping::arucoMarker2Tf(const aruco::Marker &marker)
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
  rotate_to_ros.at<float>(1,2) = -1.0;  //1.0
  rotate_to_ros.at<float>(2,0) = 0.0;
  rotate_to_ros.at<float>(2,1) = -1.0; //1.0 prima era cosi , aggiunta una rotazione di 180 su X
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
