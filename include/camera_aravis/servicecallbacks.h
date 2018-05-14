#ifndef SERVICECALLBACKS_H
#define SERVICECALLBACKS_H

#include <thread>
#include <chrono>
#include <vector>

#include <camera_aravis/Capture.h>
#include <camera_aravis/GetConnectedDevices.h>
#include <camera_aravis/SendCommand.h>


#include "camera_aravis/genicam.h"

void restoreAquistionsMode(std::unordered_map<std::string, GeniCam>::iterator& iter, bool& wasStreaming)
{
  arv_device_execute_command(iter->second.pDevice, "AcquisitionStop");
  arv_device_set_string_feature_value (iter->second.pDevice, "AcquisitionMode", "Continuous");
  arv_device_set_string_feature_value (iter->second.pDevice, "TriggerMode", "Off");

  if(wasStreaming)
  {
    arv_device_execute_command(iter->second.pDevice,
                             "AcquisitionStart");
  }
}

// service callbacks
bool capture_callback(camera_aravis::CaptureRequest& request,
                      camera_aravis::CaptureResponse& response,
                      std::unordered_map<std::string, GeniCam>& cameras)
{
  for(auto& serial : request.serials)
  {
    bool wasStreaming = false;
    bool changedTriggerProperties = false;
    auto iter = cameras.find(serial);
    try
    {
      if(!iter->second.genicamFeatures.is_implemented("TriggerMode") ||
         !iter->second.genicamFeatures.is_implemented("TriggerSource") ||
         !iter->second.genicamFeatures.is_implemented("AcquisitionMode") ||
         !iter->second.genicamFeatures.is_implemented("ExposureTime"))
      {
        std::runtime_error("Cature Service: can not be used because "
                           "software and hardware triggering is not "
                           "supported by camera with serial number: " + serial);
      }

      std::chrono::nanoseconds exposure_time;
      if(iter != cameras.end())
      {
        changedTriggerProperties = true;
        if(iter->second.state == State::STREAMING)
        {
          wasStreaming = true;
          arv_device_execute_command(iter->second.pDevice,
                                   "AcquisitionStop");
        }

        arv_device_set_string_feature_value (iter->second.pDevice, "AcquisitionMode", "SingleFrame");
        arv_device_set_string_feature_value (iter->second.pDevice, "TriggerMode", "On");
        arv_device_set_string_feature_value (iter->second.pDevice, "TriggerSource", "Software");

        arv_device_execute_command(iter->second.pDevice,
                                 "AcquisitionStart");

        iter->second.isNewImage = false;
        arv_device_execute_command(iter->second.pDevice, "TriggerSoftware");

        for(int i = 0; i < 3; i++)
        {
          if(iter->second.isNewImage == true)
          {
            break;
          }
          else if(i<2)
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          }
          else
          {
            throw std::runtime_error("Cature Service: after 3 times the "
                                     "exposure time a new image was still "
                                     "not available abort; serial: " + serial);
          }
        }

        response.serials.push_back(serial);
        response.images.push_back(iter->second.imageMsg);

        restoreAquistionsMode(iter, wasStreaming);
      }
      else
      {
        std::runtime_error("Cature Service: request image from "
                           "not available camera with serial: " + serial);
      }

    } catch(const std::exception& e)
    {
      if(changedTriggerProperties)
      {
        restoreAquistionsMode(iter, wasStreaming);
      }

      ROS_WARN(e.what());
      response.images.clear();
      response.serials.clear();
      return false;
    }

  }

  return true;
}


bool getConnectedDevices_callback(camera_aravis::GetConnectedDevicesRequest& request,
                                 camera_aravis::GetConnectedDevicesResponse& response,
                                 std::unordered_map<std::string, GeniCam>& cameras)
{
  for(auto& camera : cameras)
  {
    if(camera.second.state == State::READY ||
       camera.second.state == State::STREAMING)
    {
      response.serials.push_back(camera.first);
    }
  }
  return true;
}

#endif // SERVICECALLBACKS_H
