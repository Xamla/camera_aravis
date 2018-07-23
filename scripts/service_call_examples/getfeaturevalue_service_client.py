#!/usr/bin/env python

import sys
import rospy
import time
from camera_aravis.srv import GetFeatureValue, GetFeatureValueRequest, GetConnectedDevices


def call_getconnecteddevices_service():
    rospy.wait_for_service('camera_aravis_node/get_connected_devices')
    try:
        getSerials = rospy.ServiceProxy(
            'camera_aravis_node/get_connected_devices', GetConnectedDevices)
        resp = getSerials()
        #temp = ['12']
        return resp.serials

    except rospy.ServiceException as e:
        print ("Service call failed: " + e)


class GetFeatureValueClient:

    def call_get_feature_value_service(self, serials):
        rospy.wait_for_service('camera_aravis_node/get_feature_value')
        try:
            start = time.time()
            service = rospy.ServiceProxy(
                'camera_aravis_node/get_feature_value', GetFeatureValue)
            req = GetFeatureValueRequest()
            req.serials = serials
            req.features = ['DeviceTemperature'] * len(serials)
            resp = service(req)
            end = time.time()
            print('Service call takes: ' + str(end-start))
            return resp
        except rospy.ServiceException as e:
            print ("Service call failed: " + str(e))


def main():
    client = GetFeatureValueClient()

    serials = call_getconnecteddevices_service()

    if len(serials) >= 1:
        print('request tempature from following cameras:')
        for serial in serials:
            print('Camera with serial number: ' + str(serial))
        resp = client.call_get_feature_value_service(serials)
        print('tempatures are:\n {}'.format('\n'.join(resp.values)))
    else:
        print('no cameras are available')


if __name__ == "__main__":
    main()
