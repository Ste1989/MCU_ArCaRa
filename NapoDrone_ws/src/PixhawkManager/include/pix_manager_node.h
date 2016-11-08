
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include "std_msgs/Int32.h"

//stato del drone
mavros_msgs::State current_state;


std::string init_flight_mode;
int loop_rate;

//ros topic subscriber
ros::Subscriber state_sub;
ros::Subscriber cmd_sub;
ros::Subscriber mode_sub;
ros::Subscriber param_sub;
//ros topic publisher
ros::Publisher rc_pub;
ros::Publisher state_pub;
//ros topic service
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;


//ENUMERATORI
/*possibili stati autopilota *************************************/
typedef enum{
    CONNECTING,
    CONNECTED,
    ARMABLE,
    NOT_ARMABLE,
    ARMED,
    TAKE_OFF,
    LANDED,
    DISCONNECTED,
    HOVER,
    LANDING,
    RTL_STATUS,
    EMERGENCY_STOP_STATUS,
}status_px4;


/*possibili richieste di flight mode *************************************/
typedef enum{
    NO_MODE,
    STABILIZE,
    ALT_HOLD,
    LOITER,
    AUTO,
    ACRO,
    SPORT,
    DRIFT,
    GUIDED,
    CIRCLE,
    POS_HOLD,
    BRAKE,
    FOLLOW_ME,
    SIMPLE_SUPER,
} mode_request;
/*possibili richiesta di comandi**********************************/
typedef enum{
    NO_REQ,
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    RTL,
    EMERGENCY_STOP,
} cmd_request;

cmd_request current_cmd_req;
/*possibili richieste di cambio parametri*****************************/
typedef enum{
    NO_PARAM,
    ALT_TAKEOFF,
} param_request;



//FUNZIONI////////////////////////////////////////////////////////////////////////////////////
//callback per lo stato del drone
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void cmd_cb(const std_msgs::Int32::ConstPtr& msg);
void mode_cb(const std_msgs::Int32::ConstPtr& msg);
void param_cb(const std_msgs::Int32::ConstPtr& msg);
bool arm_vehicle();
bool disarm_vehicle();