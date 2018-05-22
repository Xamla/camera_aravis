#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include <camera_aravis/Capture.h>
#include <camera_aravis/GetConnectedDevices.h>
#include <camera_aravis/SendCommand.h>
#include <camera_aravis/SetIO.h>

#include "xamla_sysmon_msgs/HeartBeat.h"
#include "xamla_sysmon_msgs/statuscodes.h"

#include "camera_aravis/genicam.h"

class CameraManager
{
public:
  CameraManager() = delete;
  CameraManager(std::shared_ptr<ros::NodeHandle> &nodeHandle);

  static gboolean update_callback(void* cameraManager);

  bool capture_callback(camera_aravis::CaptureRequest& request,
                        camera_aravis::CaptureResponse& response);

  bool getConnectedDevices_callback(camera_aravis::GetConnectedDevicesRequest,
                                   camera_aravis::GetConnectedDevicesResponse& response);

  bool sendCommand_callback(camera_aravis::SendCommandRequest& request,
                       camera_aravis::SendCommandResponse& response);

  bool setIO_callback(camera_aravis::SetIORequest& request,
                      camera_aravis::SetIOResponse& response);

  void sendHeartBeat();

protected:
  //search for new devices and initialize
  //them when device is requested
  void initializeDevices(bool is_first_time=false);
  xamla_sysmon_msgs::HeartBeat createHeartBeatMessage();

  //attributes
  std::shared_ptr<ros::NodeHandle> phNodeHandle;

  std::vector<std::string> requested_cameras;

  std::unordered_map<std::string, std::shared_ptr<GeniCam>> cameras;

  ros::ServiceServer captureServiceServer;
  ros::ServiceServer getConnectedDevicesServiceServer;
  ros::ServiceServer sendCommandServiceServer;
  ros::ServiceServer setIOServiceServer;

  ros::Publisher heartBeatPublisher;
};

#endif // CAMERAMANAGER_H
