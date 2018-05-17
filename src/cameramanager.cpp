#include "camera_aravis/cameramanager.h"

CameraManager::CameraManager(std::shared_ptr<ros::NodeHandle> &nodeHandle):
  phNodeHandle(nodeHandle)
{
  cameras.reserve(16);
  initializeDevices(true);
}

gboolean CameraManager::update_callback(void *cameraManager)
{
  ((CameraManager*) cameraManager)->initializeDevices();

  return true;
}


void CameraManager::initializeDevices(bool is_first_time)
{
  uint64_t nDevices=0;
  std::unordered_map<std::string, std::string> available_cameras;

  // Print out some useful info.
  ROS_INFO("Update Device List:");
  arv_update_device_list();

  nDevices = arv_get_n_devices();
  ROS_INFO("# Number of found Devices: %d", nDevices);
  for (int i = 0; i < nDevices; i++)
  {
    std::string device_ID = arv_get_device_id(i);
    size_t position = device_ID.rfind('-');
    if (position!=std::string::npos && device_ID.size()>(position+1))
    {
      available_cameras[device_ID.substr(position+1)] = device_ID;
    } else
    {
      available_cameras[device_ID] = device_ID;
    }

  }

  if(is_first_time == true)
  {
    if (phNodeHandle->hasParam(ros::this_node::getName() + "/camera_serials"))
    {
      phNodeHandle->getParam(ros::this_node::getName() + "/camera_serials",
                              requested_cameras);
    }

    if (requested_cameras.size()==0)
    {
      for(auto &serial_deviceID : available_cameras)
      {
        requested_cameras.push_back(serial_deviceID.first);
      }
    }
  }

  for (const auto &serial_deviceID : available_cameras)
  {
    if(std::find(requested_cameras.begin(),
                 requested_cameras.end(),
                 serial_deviceID.first) != requested_cameras.end())
    {
      auto iter = cameras.find(serial_deviceID.first);
      if( iter != cameras.end() && iter->second.getCameraState() == CameraState::NOTINITIALIZED)
      {
        ROS_INFO("Reactivate camera: %s  with device id: %s",
                 serial_deviceID.first.c_str(), serial_deviceID.second.c_str());

        iter->second.reestablishConnection(phNodeHandle,
                                           serial_deviceID);
      }
      else
      {
        // Open the camera, and set it up.
        ROS_INFO("Add requested camera: %s  with device id: %s",
                 serial_deviceID.first.c_str(), serial_deviceID.second.c_str());
/*
        cameras.emplace(std::piecewise_construct,
                        std::forward_as_tuple(serial_deviceID.first),
                        std::forward_as_tuple(GeniCam(phNodeHandle,
                                                      serial_deviceID)));*/
      }
    }

  }

}
