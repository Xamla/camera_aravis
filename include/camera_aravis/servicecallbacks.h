#ifndef SERVICECALLBACKS_H
#define SERVICECALLBACKS_H

#include <thread>
#include <chrono>
#include <vector>
#include <string>

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
        std::chrono::microseconds exposure_time(int(arv_device_get_float_feature_value(iter->second.pDevice, "ExposureTime")));

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

bool sendCommand_callback(camera_aravis::SendCommandRequest& request,
                     camera_aravis::SendCommandResponse& response,
                     std::unordered_map<std::string, GeniCam>& cameras)
{
  for(auto &serial : request.serials )
  {
    bool wasStreaming = false;
    bool changedTriggerProperties = false;
    auto iter = cameras.find(serial);
    try
    {
      if(!iter->second.genicamFeatures.is_implemented(request.command_name))
      {
        std::runtime_error("sendCommand Service: can set value for command" +
                           request.command_name + " ,because it is not " +
                           "supported by camera with serial number: " + serial);
      }

      if(iter != cameras.end())
      {
        changedTriggerProperties = true;
        if(iter->second.state == State::STREAMING)
        {
          wasStreaming = true;
          arv_device_execute_command(iter->second.pDevice,
                                   "AcquisitionStop");
        }

        std::string value = std::to_string(request.value);
        iter->second.genicamFeatures.get_feature(request.command_name).set_current_value(
              iter->second.pDevice, value);

        if(wasStreaming == true)
        {
          arv_device_execute_command(iter->second.pDevice,
                                   "AcquisitionStart");
        }
      }
      else
      {
        std::runtime_error("Cature Service: request image from "
                           "not available camera with serial: " + serial);
      }

    }
    catch(const std::exception &e)
    {
      if(wasStreaming == true)
      {
        arv_device_execute_command(iter->second.pDevice,
                                 "AcquisitionStart");
      }
      response.response = "only the command for cameras before the camera with serial "
                          "number " + serial + " could be executed";
      ROS_WARN(e.what());
      return false;
    }
  }
  response.response="everthing works fine";
  return true;
}

#endif // SERVICECALLBACKS_H
