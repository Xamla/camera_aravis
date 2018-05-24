#!/usr/bin/env python
import thread
import signal
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


def call_getconnecteddevices_service():
    rospy.wait_for_service('camera_aravis_node/getconnecteddevices')
    try:
        getSerials = rospy.ServiceProxy(
            'camera_aravis_node/getconnecteddevices', GetConnectedDevices)
        resp = getSerials()
        return resp.serials

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


class FileManager:
    def __init__(self, path):
        self.path = path + '/'

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
                    data[serial] = self.bridge.imgmsg_to_cv2(
                        resp.images[i], "bgr8")
                except CvBridgeError as e:
                    print(e)

            return data

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            raise Exception('capture failed!')


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

        try:
            captureResult = self.captureClient.call_capture_service(
                self.serials)
            robot_state = self.robot.get_current_state()

            data = {}
            data['name'] = robot_state.joint_state.name
            data['position'] = robot_state.joint_state.position

            now = datetime.datetime.now()
            filenameCommonPart = now.strftime("%Y-%m-%d_%H:%M:%S")

            for i, serial in enumerate(captureResult):
                self.fileManager.writeImage(
                    filenameCommonPart+'_camera_'+serial, captureResult[serial])
            self.fileManager.writeJsonFile(
                filenameCommonPart+'_joint_state', data)
        except Exception, e:
            print e


terminate = False
nextCommand = True


def signal_handling(signum, frame):
    global terminate
    terminate = True


def input_thread():
    global nextCommand
    while True:
        if terminate == True:
            break
        if nextCommand:
            raw_input('Please enter to start the next capture')


def main():
    global nextCommand

    serials = call_getconnecteddevices_service()

    path = '/home/volk/Desktop/recording'
    serials = ['4103217455']
    #serials = ['4103235743','4103217455']

    saveIAJ = SaveImageAndJointState(path, serials)

    print ('press strg-C to exit')

    signal.signal(signal.SIGINT, signal_handling)

    thread.start_new_thread(input_thread, ())

    while True:
        if terminate == True:
            break
        if not nextCommand:
            print('start capture process')
            saveIAJ.capture()
            print('ready for next')
            nextCommand = True


if __name__ == "__main__":
    main()
