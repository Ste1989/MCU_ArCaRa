# Import DroneKit-Python
from dronekit import * #connect, VehicleMode
# Import Utility.py (by Matteo)
from Utility import *
import dronekit_sitl
import socket
import exceptions

#################################################
			# CONNECTION
#If ew have a REAL vehicle we must be set it here! (connection string)
#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None
#Start SITL if no connection string specified
if not connection_string:
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
try:
	vehicle = connect('/dev/ttyACM0', wait_ready=True)
# Bad TCP connection
except socket.error:
    print 'No server exists!'

# Bad TTY connection
except exceptions.OSError as e:
    print 'No serial exists!'

# API Error
except dronekit.APIException:
    print 'Timeout!'

# Other error
except:
    print 'Some other error!'


###########################################################################
				# STATE
# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
print " Autopilot Firmware version: %s" % vehicle.version
print "   Major version number: %s" % vehicle.version.major
print "   Minor version number: %s" % vehicle.version.minor
print "   Patch version number: %s" % vehicle.version.patch
print "   Release type: %s" % vehicle.version.release_type()
print "   Release version: %s" % vehicle.version.release_version()
print "   Stable release?: %s" % vehicle.version.is_stable()
print " Autopilot capabilities"
print "   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float
print "   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float
print "   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int
print "   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int
print "   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union
print "   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp
print "   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target
print "   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned
print "   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int
print "   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain
print "   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target
print "   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination
print "   Supports mission_float message type: %s" % vehicle.capabilities.mission_float
print "   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration
print " Global Location: %s" % vehicle.location.global_frame
print " Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
print "Global Location (relative altitude): %s" % vehicle.home_location
# The returned value is None before you download the commands or if the home_location has not yet been set by the autopilot. 
print " Local Location: %s" % vehicle.location.local_frame #NED
print " Attitude: %s" % vehicle.attitude
print " Velocity: %s" % vehicle.velocity
print " GPS: %s" % vehicle.gps_0
print " Gimbal status: %s" % vehicle.gimbal
print " Battery: %s" % vehicle.battery
print " EKF OK?: %s" % vehicle.ekf_ok
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Rangefinder: %s" % vehicle.rangefinder
print " Rangefinder distance: %s" % vehicle.rangefinder.distance
print " Rangefinder voltage: %s" % vehicle.rangefinder.voltage
print " Heading: %s" % vehicle.heading
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Groundspeed: %s" % vehicle.groundspeed    # settable
print " Airspeed: %s" % vehicle.airspeed    # settable
print " Mode: %s" % vehicle.mode.name    # settable
print " Armed: %s" % vehicle.armed    # settable


############################ FIRST TESTS from GUIDE SECTION

# try to arm and take off to target altitude
arm_and_takeoff(20, vehicle)

print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
############################ FIRST TESTS

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
print("Completed")


####################################################################################


