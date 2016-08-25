#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CameraInfo.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);
  ros::Publisher info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info",1);
  /*Camera 	parameters*/
  double frame_width, frame_height, fps, brigthness, contrast, saturation, exposure, gain, hue, fourcc;
  char read_param = 1; 
  char set_param = 1;
  /*Open the webcam*/
  cv::VideoCapture cap;
  cap.open(2);		//Open system default camera INTEl
  
  if(set_param)
  {
	/*SET CAMERA PARAMETER*/
    /*
    brightness: 0.219608
    contrast: 0.333333
    saturation: 0.501961
    exposure: -1
    gain: 0.125
    hue: 0.5*/

	frame_width= 960; //1920; //1200
	frame_height = 540;//1080; //800
	fps = 30;
    brigthness = 0.219608; //0.5
    contrast = 0.3333; //0
    saturation = 0.501961; //0.45
	hue = 0.5; //0.5
    gain = 0.125;

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
	  
	  
	  
  }

  

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
  
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) {
	ROS_ERROR("No camera Found");
	return 1;
  }
  
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  while (nh.ok()) 
  {
    cap >> frame;
   
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) 
    {
		
      //cv::Mat src_gray;
      //cvtColor( frame, src_gray, CV_BGR2GRAY );
      sensor_msgs::CameraInfo info;
      info.height = frame_height;
      info.width = frame_width;
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      info_pub.publish(info);
    }

    else
    {
      ROS_ERROR("No camera Found");
    }

    ros::spinOnce();
  }
}






/*

}
*/

