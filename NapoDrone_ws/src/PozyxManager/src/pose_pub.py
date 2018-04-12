#!/usr/bin/env python
"""
ROS node that publishes the pose (position + quaternion) of the Pozyx

This is an example of how to combine sensor data and positioning into a single
channel output.

Quite overkill using _Pose, as this consists of 7 float64s, while the source
data comes from integers. Suggestions to replace this are quite welcomed.
"""

import pypozyx
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D,POZYX_POS_ALG_TRACKING,POZYX_FAILURE, POZYX_2D,Coordinates, POZYX_SUCCESS, POZYX_ANCHOR_SEL_AUTO,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList)
from pythonosc.udp_client import SimpleUDPClient
remote_id = None

anchors_ids = [0xA000,0xA001,0xA002,0xA003];
height_anchor = [2000,2000,2000,2000];

enable_auto_calibration = False
class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""
#########################################################################################################
#
#                               Init
#
########################################################################################################
    def __init__(self, pozyx, osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id
#########################################################################################################
#
#                               SETUP
#
########################################################################################################
    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V1.1 -------------")
        print("NOTES: ")
        print("- No parameters required.")
        print()
        print("- System will auto start configuration")
        print()
        print("- System will auto start positioning")
        print()
        self.pozyx.printDeviceInfo(self.remote_id)
        print()
        print("------------POZYX POSITIONING V1.1 --------------")
        print()
        #cancella i devices in memoria
        self.pozyx.clearDevices(self.remote_id)
        #auto range delle ancore
        status = self.setAnchorAuto()
        if status == POZYX_FAILURE:
            print("----------------------")
            print("fai calibrazione manuale")
            print("----------------------")            
        else:
            print("----------------------")
            print("calibrazione OK")
            print("----------------------")
        self.printPublishConfigurationResult()

#########################################################################################################
#
#                               calibrazione Automatica
#
########################################################################################################
    def setAnchorAuto(self):
        if enable_auto_calibration == True :
            status = self.pozyx.doAnchorCalibration(POZYX_2D,100,anchors_ids,height)
            #adesso rileggo le posizione delle ancore
            list_size = SingleRegister()
            self.pozyx.getDeviceListSize(list_size, self.remote_id)
            #print("List size: {0}".format(list_size[0]))
            if list_size[0] != len(self.anchors):
                self.printPublishErrorCode("configuration")
                return

            device_list = DeviceList(list_size=list_size[0])
            self.pozyx.getDeviceIds(device_list, self.remote_id)
            #print("Calibration result:")
            #print("Anchors found: {0}".format(list_size[0]))
            #print("Anchor IDs: ", device_list)

            for i in range(list_size[0]):
                anchor_coordinates = Coordinates()
                self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
                if i == 0:
                    self.anchors[i] = DeviceCoordinates(0xA000, 1, Coordinates(int(anchor_coordinates.x), int(anchor_coordinates.y), height_anchor[i]))
                if i == 1:
                    self.anchors[i] = DeviceCoordinates(0xA001, 1, Coordinates(int(anchor_coordinates.x), int(anchor_coordinates.y), height_anchor[i]))
                if i == 2:
                    self.anchors[i] = DeviceCoordinates(0xA002, 1, Coordinates(int(anchor_coordinates.x), int(anchor_coordinates.y), height_anchor[i]))
                if i == 3:
                    self.anchors[i] = DeviceCoordinates(0xA003, 1, Coordinates(int(anchor_coordinates.x), int(anchor_coordinates.y), height_anchor[i]))            
             
             #adesso in self.anchors dovrei avere le coordinate delle antenne calcolate in automatico tranne l altezza imposta
             #richiamo quindi la funzione seTAnchorManual
        self.setAnchorsManual()
#########################################################################################################
#
#                               Main LOOP
#
########################################################################################################
    def loop(self):
        #Performs positioning and displays/exports the results.
        print("sono qui")
        position = Coordinates()
        quat = pypozyx.Quaternion()
        status = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        self.pozyx.getQuaternion(quat, remote_id=remote_id)
        if status == POZYX_SUCCESS:
            self.printPublishPosition(position)
            pub.publish(Point(position.x, position.y, position.z),
                    Quaternion(quat.x, quat.y, quat.z, quat.w))
        else:
            self.printPublishErrorCode("positioning")
        


#########################################################################################################
#
#                               Pubblica posizione
#
########################################################################################################

    def printPublishPosition(self, position):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        network_id = self.remote_id
        if network_id is None:
            network_id = 0
        print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
            "0x%0.4x" % network_id, pos=position))
        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, int(position.x), int(position.y), int(position.z)])
#########################################################################################################
#
#                               stampa messaggi di errore
#
########################################################################################################
    def printPublishErrorCode(self, operation):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, -1])
            # should only happen when not being able to communicate with a remote Pozyx.
#########################################################################################################
#
#                               setta le ancore manulamente
#
########################################################################################################
    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors))
        return status
#########################################################################################################
#
#                               Stampa configurazione ancore
#
########################################################################################################
    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [device_list[i], int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)
#########################################################################################################
#
#                               Stampa configurazione ancore
#
########################################################################################################
    def printPublishAnchorConfiguration(self):
        """Prints and potentially publishes the anchor configuration"""

        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, int(anchor.coordinates.x), int(anchor.coordinates.y), int(anchor.coordinates.z)])
                sleep(0.025)

#########################################################################################################
#
#                               MAIN
#
########################################################################################################

if __name__ == '__main__':
    # shortcut to not have to find out the port yourself
    
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()
    
    remote_id = 0x6069                 # remote device network ID
    remote = False                   # whether to use a remote device
    if not remote:
        remote_id = None

    use_processing = False             # enable to send position data through OSC
    ip = "127.0.0.1"                   # IP for the OSC UDP
    network_port = 8888                # network port for the OSC UDP
    osc_udp_client = None
    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)
    # necessary data for calibration, change the IDs and coordinates yourself
    anchors = [DeviceCoordinates(0xA000, 1, Coordinates(0, 0, 2000)),
               DeviceCoordinates(0xA001, 1, Coordinates(4560, 0, 2000)),
               DeviceCoordinates(0xA002, 1, Coordinates(4040, 3323, 2000)),
               DeviceCoordinates(0xA003, 1, Coordinates(716, 1559, 2000))]

    algorithm = POZYX_POS_ALG_TRACKING  # positioning algorithm to use
    dimension = POZYX_3D               # positioning dimension
    height = 1000                      # height of device, required in 2.5D positioning

    pozyx = PozyxSerial(serial_port)
    r = ReadyToLocalize(pozyx, osc_udp_client, anchors, algorithm, dimension, height, remote_id)
    r.setup()
    rospy.init_node('pozyx_pose_node')
    pub = rospy.Publisher('pozyx_pose', Pose, queue_size=40)
    while not rospy.is_shutdown():
        try:
            r.loop()
        except rospy.ROSInterruptException:
            pass