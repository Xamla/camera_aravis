#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import pdb
import json
import datetime
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from camera_aravis.srv import *
#import Tkinter as tk

class FileManager:
  def __init__(self, path):
    self.path =  path + '/'

  def writeJsonFile(self, filename, data):
    with open(self.path+filename+'.json', 'w') as fp:
      json.dump(data, fp)

  def loadJsonFile(self, filename):
    with open(self.path+filename+'.json', 'r') as fp:
      return json.load(fp)

  def writeImage(self, filename, data):
    cv2.imwrite(self.path+filename+'.png', data)

  def loadImage(self, filename):
    return cv2.imread(self.path+filename+'.png')

class CaptureClient:
  def __init__(self):
    self.bridge = CvBridge()

  def call_capture_service(self, serials):
    rospy.wait_for_service('camera_aravis_node/capture')
    try:
      capture = rospy.ServiceProxy('camera_aravis_node/capture', Capture)
      resp = capture(serials)
      
      data = {}
      for i, serial in enumerate(serials):
        try:
          data[serial] = self.bridge.imgmsg_to_cv2(resp.images[i], "bgr8")
        except CvBridgeError as e:
          print(e)

      return data        

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

class SaveImageAndJointState:

  def __init__(self, path, serials):
    rospy.init_node('collect_image_and_joint_values',
                anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.fileManager = FileManager(path)
    self.captureClient = CaptureClient()
    self.serials = serials

  def capture(self): 
    
    captureResult = self.captureClient.call_capture_service(self.serials)
    robot_state = self.robot.get_current_state()

    data = {}
    data['name'] = robot_state.joint_state.name
    data['position'] = robot_state.joint_state.position

    now = datetime.datetime.now()
    filenameCommonPart = now.strftime("%Y-%m-%d_%H:%M:%S")

    for i, serial in enumerate(captureResult):
      self.fileManager.writeImage(filenameCommonPart+'_camera_'+serial, captureResult[serial])
    self.fileManager.writeJsonFile(filenameCommonPart+'_joint_state',data)
    


if __name__ == "__main__":
  path = '/home/volk/Desktop/recording'
  #serials = ['4103217455']
  serials = ['4103235743','4103217455']

  saveIAJ = SaveImageAndJointState(path, serials)

  print('start capture process') 
  saveIAJ.capture()
  print('ready for next')

  #def callback(event):
  #  print('start capture process') 
  #  saveIAJ.capture()
  #  print('ready for next')

  #root = tk.Tk()
  #canvas= tk.Canvas(root, width=100, height=100)
  #canvas.create_text(50,50,fill="black",font="Times 10 italic bold",
  #                      text="Click to capture")
  #canvas.bind("<Button-1>", callback)
  #canvas.pack()
  #root.mainloop()


