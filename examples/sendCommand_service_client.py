#!/usr/bin/env python

import thread
import signal
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from camera_aravis.srv import *


def call_getconnecteddevices_service():
    rospy.wait_for_service('camera_aravis_node/getconnecteddevices')
    try:
        getSerials = rospy.ServiceProxy(
            'camera_aravis_node/getconnecteddevices', GetConnectedDevices)
        resp = getSerials()
        return resp.serials

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def call_sendcommand_service(serials, command, value):
    rospy.wait_for_service('camera_aravis_node/sendcommand')
    try:
        sendCommand = rospy.ServiceProxy(
            'camera_aravis_node/sendcommand', SendCommand)
        resp = sendCommand(serials, command, value)

        print(resp.response)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


terminate = False
nextCommand = True
value = ''


def signal_handling(signum, frame):
    global terminate
    terminate = True


def input_thread():
    global value
    global nextCommand
    while True:
        if terminate == True:
            break
        if nextCommand:
            value = raw_input('Please enter requested exposure time [us]: ')
            print(value)
            if value.isdigit():
                nextCommand = False
            else:
                print('please enter a number not something else')


def main():
    global nextCommand

    serials = call_getconnecteddevices_service()

    # if you want specific cameras just override serials
    serials = ['4103217455']

    print ('press strg-C to exit')

    signal.signal(signal.SIGINT, signal_handling)

    thread.start_new_thread(input_thread, ())

    while True:
        if terminate == True:
            break
        if not nextCommand:
            print('set ExposureTime ...')
            call_sendcommand_service(serials, "ExposureTime", float(value))
            print('finished')
            nextCommand = True


if __name__ == "__main__":
    main()
