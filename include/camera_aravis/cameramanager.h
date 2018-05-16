#ifndef CAMERAMANAGER_H
#define CAMERAMANAGER_H

#include "camera_aravis/genicam.h"

class CameraManager
{
public:
  CameraManager(std::shared_ptr<ros::NodeHandle> &nodeHandle);

  static gboolean update_callback(void* cameraManager);

protected:
  //search for new devices and initialize
  //them when device is requested
  void initializeDevices();

  std::shared_ptr<ros::NodeHandle> phNodeHandle;

};

#endif // CAMERAMANAGER_H
