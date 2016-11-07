
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>


//stato del drone
mavros_msgs::State current_state;

//ros topic subscriber
ros::Subscriber state_sub;
//ros topic publisher
ros::Publisher rc_pub;
//ros topic service
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;