#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from camera_aravis.srv import *


def call_getconnecteddevices_service():
    rospy.wait_for_service('camera_aravis_node/get_connected_devices')
    try:
        getSerials = rospy.ServiceProxy(
            'camera_aravis_node/get_connected_devices', GetConnectedDevices)
        resp = getSerials()

        print('Number of connected cameras: ' + str(len(resp.serials)))

        for serial in resp.serials:
            print('Camera serial number: ' + str(serial))

    except rospy.ServiceException as e:
        print ("Service call failed: " + e)


def main():
    call_getconnecteddevices_service()


if __name__ == "__main__":
    main()
