
#include "pix_manager_node.h"

mavros_msgs::State current_state;

/********************************************************************************************/
/*                                                                                         */
/*    CALBACK PER LEGGERE LO STATO DEL DRONE                                               */
/*                                                                                         */
/*******************************************************************************************/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/********************************************************************************************/
/*                                                                                         */
/*    MAIN                                                                                 */
/*                                                                                         */
/*******************************************************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pix_manager_node");
    ros::NodeHandle nh;

    //stato del drone
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    //topic per override la radio
    rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10);
    //service per armare il drone
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //service per cambiare il modo del drone
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    
    //impostiamo lo sttao del drone a STABILIZE DI DEFAULTz
    mavros_msgs::SetMode default_mode;
    default_mode.request.custom_mode = "STABILIZE";
    ros::Time last_request = ros::Time::now();
    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    

    //Attendo che sia in default mode
    while(ros::ok() && current_state.mode != "STABILIZE")
    {
        if( (ros::Time::now() - last_request > ros::Duration(5.0)))
            if( set_mode_client.call(default_mode) && default_mode.response.success){
                ROS_INFO("DEFUALT MODE ENABLED");
            }
            last_request = ros::Time::now();
     }


     while(ros::ok())
{}

        if(current_state.armed)
        {
        ROS_INFO("Vehicle override");
        //local_pos_pub.publish(pose);
        mavros_msgs::OverrideRCIn rc;
        
        rc.channels[0] = 0;
        rc.channels[1] = 0;
        rc.channels[2] = 1500;
        rc.channels[3] = 0; // yaw
        //rc.channels[4] = 500;
        rc_pub.publish(rc);



        }
        else{
        
        //local_pos_pub.publish(pose);
        mavros_msgs::OverrideRCIn rc;
        
        rc.channels[3] = 0;
        rc.channels[1] = 0;
        rc.channels[2] = 0;
        rc.channels[0] = 0;
        //rc.channels[4] = 500;
        rc_pub.publish(rc);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}