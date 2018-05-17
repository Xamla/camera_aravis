#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include "camera_aravis/genicam.h"

class CameraManager
{
public:
  CameraManager() = delete;
  CameraManager(std::shared_ptr<ros::NodeHandle> &nodeHandle);

  static gboolean update_callback(void* cameraManager);

protected:
  //search for new devices and initialize
  //them when device is requested
  void initializeDevices(bool is_first_time=false);

  //attributes
  std::shared_ptr<ros::NodeHandle> phNodeHandle;

  std::vector<std::string> requested_cameras;

  std::unordered_map<std::string, GeniCam> cameras;
};

#endif // CAMERAMANAGER_H
