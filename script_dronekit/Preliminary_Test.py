# Import DroneKit-Python
from dronekit import * #connect, VehicleMode
# Import Utility.py (by Matteo)
#from Utility import *
#import dronekit_sitl
#import socket
import exceptions

#################################################
			# CONNECTION
#If ew have a REAL vehicle we must be set it here! (connection string)
#Set up option parsing to get connection string


# Connect to the Vehicle.
vehicle = connect('/dev/ttyACM0', wait_ready=True)
#vehicle = connect('...', wait_ready='armable')
#vehicle.armed = True
# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"

print " Attitude: %s" % vehicle.attitude

print "Basic pre-arm checks"
# Don't try to arm until autopilot is ready
while not vehicle.is_armable:
	print " Waiting for vehicle to initialise..."
	print " GPS: %s" % vehicle.gps_0
	print " Mode: %s" % vehicle.mode.name
	print " EKF OK?: %s" % vehicle.ekf_ok
        time.sleep(1)
print "Arming motors"
# Copter should arm in GUIDED mode
vehicle.mode    = VehicleMode("GUIDED")
vehicle.armed   = True
# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
	print " Waiting for arming..."
        time.sleep(1)

print "Ready to Taking off!"

time.sleep(10)
############################ FIRST TESTS

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
print("Completed")


####################################################################################


