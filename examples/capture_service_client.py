#!/usr/bin/env python

import sys
import rospy
import cv2
import pdb
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from camera_aravis.srv import *

class capture_client:
  def __init__(self):
    self.bridge = CvBridge()

  def call_capture_service(self, serials):
    rospy.wait_for_service('camera_aravis_node/capture')
    try:
      capture = rospy.ServiceProxy('camera_aravis_node/capture', Capture)
      resp = capture(serials)
       
      for i, serial in enumerate(serials):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(resp.images[i], "bgr8")
        except CvBridgeError as e:
          print(e)

        cv2.imshow(serial,cv_image)
        cv2.waitKey(1000)

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

def main():
  client = capture_client()
  client.call_capture_service(['4103235743','4103217455'])

if __name__ == "__main__":
  main()
  
