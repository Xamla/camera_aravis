#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include <thread>

#include <camera_aravis/Capture.h>
#include <camera_aravis/GetConnectedDevices.h>
#include <camera_aravis/GetFeatureValue.h>
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

  ~CameraManager();

  bool runUpdate();

  bool capture_callback(camera_aravis::CaptureRequest& request,
                        camera_aravis::CaptureResponse& response);

  bool getConnectedDevices_callback(camera_aravis::GetConnectedDevicesRequest,
                                   camera_aravis::GetConnectedDevicesResponse& response);

  bool getFeatureValue_callback(camera_aravis::GetFeatureValueRequest& request,
                                camera_aravis::GetFeatureValueResponse& response);

  bool sendCommand_callback(camera_aravis::SendCommandRequest& request,
                       camera_aravis::SendCommandResponse& response);

  bool setIO_callback(camera_aravis::SetIORequest& request,
                      camera_aravis::SetIOResponse& response);

  bool runHeartbeat();

protected:
  //search for new devices and initialize
  //them when device is requested
  void initializeDevices(bool is_first_time=false);
  xamla_sysmon_msgs::HeartBeat createHeartbeatMessage();

  //attributes
  std::shared_ptr<ros::NodeHandle> phNodeHandle;
  std::unique_ptr<std::thread> pUpdateThread;
  std::atomic<bool> runUpdateThread;
  std::mutex updateMutex;
  std::condition_variable  killUpdateThread;

  std::unique_ptr<std::thread> pHeartbeatThread;
  std::atomic<bool> runHeartbeatThread;
  std::mutex heartbeatMutex;
  std::condition_variable killHeartbeatThread;

  std::vector<std::string> requested_cameras;

  std::unordered_map<std::string, std::shared_ptr<GeniCam>> cameras;

  ros::ServiceServer captureServiceServer;
  ros::ServiceServer getConnectedDevicesServiceServer;
  ros::ServiceServer getFeatureValueServiceServer;
  ros::ServiceServer sendCommandServiceServer;
  ros::ServiceServer setIOServiceServer;

  ros::Publisher heartBeatPublisher;
};

#endif // CAMERAMANAGER_H
