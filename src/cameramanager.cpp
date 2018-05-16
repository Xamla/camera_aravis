#include "camera_aravis/cameramanager.h"

CameraManager::CameraManager(std::shared_ptr<ros::NodeHandle> &nodeHandle)
{
  this->phNodeHandle = nodeHandle;

  initializeDevices();
}

gboolean CameraManager::update_callback(void *cameraManager)
{
  ((CameraManager*) cameraManager)->initializeDevices();

  return true;
}


void CameraManager::initializeDevices()
{
  std::vector<std::string> requested_cameras;
  std::unordered_map<std::string, std::string> available_cameras;

  // Print out some useful info.
  ROS_INFO("Attached cameras:");
  arv_update_device_list();

  nDevices = arv_get_n_devices();
  ROS_INFO("# Number of found Devices: %d", nDevices);
  for (i = 0; i < nDevices; i++)
  {
    std::string device_ID = arv_get_device_id(i);
    ROS_INFO("Device%d: %s", i, device_ID.c_str());
    size_t position = device_ID.rfind('-');
    if (position!=std::string::npos && device_ID.size()>(position+1))
    {
      available_cameras[device_ID.substr(position+1)] = device_ID;
    } else
    {
      available_cameras[device_ID] = device_ID;
    }

  }

  if (global.phNode->hasParam(ros::this_node::getName() + "/camera_serials"))
  {
    global.phNode->getParam(ros::this_node::getName() + "/camera_serials",
                            requested_cameras);
  }

  if (requested_cameras_serial.size()==0)
  {
    for(auto &camera : available_cameras)
    {
      requested_cameras.push_back(camera.first);
    }
  }

}
