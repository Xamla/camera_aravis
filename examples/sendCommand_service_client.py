#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from camera_aravis.srv import *

def call_getconnecteddevices_service():
  rospy.wait_for_service('camera_aravis_node/getconnecteddevices')
  try:
    getSerials = rospy.ServiceProxy('camera_aravis_node/getconnecteddevices', GetConnectedDevices)
    resp = getSerials()
    
    return resp.serials

  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def call_sendcommand_service(serials, command, value):
  rospy.wait_for_service('camera_aravis_node/sendcommand')
  try:
    sendCommand = rospy.ServiceProxy('camera_aravis_node/sendcommand', SendCommand)
    resp = sendCommand(serials, command, value)
    
    print(resp.response)

  except rospy.ServiceException, e:
    print "Service call failed: %s"%e

def main():

  serials = call_getconnecteddevices_service()

  call_sendcommand_service("ExposureTime", 3000.0, serials)

if __name__ == "__main__":
  main()
  
