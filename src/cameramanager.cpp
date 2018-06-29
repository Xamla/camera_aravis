#include "camera_aravis/cameramanager.h"
#include <algorithm>
#include <ctype.h>

// -- Public methods

CameraManager::CameraManager(std::shared_ptr<ros::NodeHandle> &nodeHandle):
  phNodeHandle(nodeHandle), runUpdateThread(true), runHeartbeatThread(true), pUpdateThread(new std::thread(&CameraManager::runUpdate, this))
{
  heartBeatPublisher = nodeHandle->advertise<xamla_sysmon_msgs::HeartBeat>("heartbeat", 1);

  cameras.reserve(16);
  initializeDevices(true);

  captureServiceServer =
      phNodeHandle->advertiseService<camera_aravis::CaptureRequest,
                                      camera_aravis::CaptureResponse>
      ("capture", std::bind(&CameraManager::capture_callback, this, std::placeholders::_1, std::placeholders::_2));

  getConnectedDevicesServiceServer =
      phNodeHandle->advertiseService<camera_aravis::GetConnectedDevicesRequest,
                                      camera_aravis::GetConnectedDevicesResponse>
      ("get_connected_devices", std::bind(&CameraManager::getConnectedDevices_callback, this, std::placeholders::_1, std::placeholders::_2));

  sendCommandServiceServer =
      phNodeHandle->advertiseService<camera_aravis::SendCommandRequest,
                                      camera_aravis::SendCommandResponse>
      ("send_command", std::bind(&CameraManager::sendCommand_callback, this, std::placeholders::_1, std::placeholders::_2));

  setIOServiceServer =
      phNodeHandle->advertiseService<camera_aravis::SetIORequest,
                                      camera_aravis::SetIOResponse>
      ("set_io", std::bind(&CameraManager::setIO_callback, this, std::placeholders::_1, std::placeholders::_2));

  pHeartbeatThread.reset(new std::thread(&CameraManager::runHeartbeat, this));
  ROS_INFO("-------Initialization Complete----------");
}

CameraManager::~CameraManager()
{
  runUpdateThread.store(false);
  killUpdateThread.notify_all();
  pUpdateThread->join();

  runHeartbeatThread.store(false);
  killHeartbeatThread.notify_all();
  pHeartbeatThread->join();
}

bool CameraManager::runUpdate()
{
  auto waitTime = std::chrono::seconds(10);

  while(runUpdateThread.load() == true)
  {
    std::unique_lock<std::mutex> lck(updateMutex);
    if(killUpdateThread.wait_for(lck, waitTime) == std::cv_status::timeout)
    {
      //ROS_INFO("Update device list");
      initializeDevices();
    }
  }
  return true;
}

bool CameraManager::capture_callback(camera_aravis::CaptureRequest &request,
                                     camera_aravis::CaptureResponse &response)
{
  for(auto& serial : request.serials)
  {
    auto iter = cameras.find(serial);
    try
    {

      if(iter != cameras.end() &&
         (iter->second->getCameraState() != CameraState::NOTINITIALIZED))
      {
        if(iter->second->capture(response.images))
          response.serials.push_back(serial);
        else
        {
          throw std::runtime_error("Capture Service: image capture timeout for camera with serial " + serial +
                                   ", abort complete process");
        }
      }
      else
      {
        throw std::runtime_error("Capture Service: camera with serial "
                           + serial + " is not available, "
                                      "abort complete process");
      }

    } catch(const std::exception& e)
    {
      ROS_WARN("%s", e.what());
      response.images.clear();
      response.serials.clear();
      return false;
    }
  }
  return true;
}

bool CameraManager::getConnectedDevices_callback(camera_aravis::GetConnectedDevicesRequest,
                                                 camera_aravis::GetConnectedDevicesResponse &response)
{
  for(auto& camera : cameras)
  {
    if(camera.second->getCameraState() != CameraState::NOTINITIALIZED)
    {
      response.serials.push_back(camera.first);
    }
  }
  return true;
}

bool CameraManager::sendCommand_callback(camera_aravis::SendCommandRequest &request,
                                         camera_aravis::SendCommandResponse &response)
{
  for(auto &serial : request.serials )
  {
    auto iter = cameras.find(serial);
    try
    {
      if(iter != cameras.end() &&
         (iter->second->getCameraState() != CameraState::NOTINITIALIZED))
      {
        if(!iter->second->tryToSetFeatureValue(request.command_name, std::to_string(request.value)))
        {
          throw std::runtime_error("sendCommand Service: requested feature: " + request.command_name +
                                   "is not available for or could not be set"
                                   "camera with serial " + serial + " , abort");
        }
      }
      else
      {
        throw std::runtime_error("sendCommand Service: camera with serial "
                           + serial + " is not available, abort process");
      }

    }
    catch(const std::exception &e)
    {
      response.response = "only the command for cameras before the camera with serial "
                          "number " + serial + " could be executed";
      ROS_WARN("%s", e.what());
      return false;
    }
  }
  response.response="everthing works fine";
  return true;
}

bool CameraManager::setIO_callback(camera_aravis::SetIORequest &request,
                                   camera_aravis::SetIOResponse &response)
{
  bool isOK = false;
  std::string line = "Line"+std::to_string(request.io_port);
  std::string userOutput = "UserOutput"+std::to_string(request.io_port);

  for(auto &serial : request.serials )
  {
    auto iter = cameras.find(serial);
    try
    {
      if(iter != cameras.end() &&
         (iter->second->getCameraState() != CameraState::NOTINITIALIZED))
      {
        isOK = iter->second->tryToSetFeatureValue("LineSelector", line.c_str());
        isOK = iter->second->tryToSetFeatureValue("LineMode", "Output");
        isOK = iter->second->tryToSetFeatureValue("LineSource", userOutput.c_str());
        isOK = iter->second->tryToSetFeatureValue("UserOutputSelector", userOutput.c_str());
        isOK = iter->second->tryToSetFeatureValue("UserOutputValue", std::to_string(request.value).c_str());

        if(!isOK)
        {
          throw std::runtime_error("setIO Service: requested io: " + std::to_string(request.io_port) +
                                   " could not be set"
                                   "for camera with serial " + serial + " , abort");
        }
      }
      else
      {
        throw std::runtime_error("setIO Service: camera with serial "
                           + serial + " is not available, abort process");
      }

    }
    catch(const std::exception &e)
    {
      response.response = "only the command for cameras before the camera with serial "
                          "number " + serial + " could be executed";
      ROS_WARN("%s", e.what());
      return false;
    }
  }
  response.response="everthing works fine";
  return true;
}

bool CameraManager::runHeartbeat()
{
  auto waitTime = std::chrono::milliseconds(200);

  while(runHeartbeatThread.load() == true)
  {
    std::unique_lock<std::mutex> lck(heartbeatMutex);
    if(killHeartbeatThread.wait_for(lck, waitTime) == std::cv_status::timeout)
    {
      heartBeatPublisher.publish(createHeartbeatMessage());
    }
  }
  return true;
}

// -- protected methods

void CameraManager::initializeDevices(bool is_first_time)
{
  uint64_t nDevices=0;
  std::unordered_map<std::string, std::string> available_cameras;

  // Print out some useful info.
  //ROS_INFO("Update Device List:");
  arv_update_device_list();
  nDevices = arv_get_n_devices();
  //ROS_INFO("# Number of found Devices: %d", nDevices);

  for (int i = 0; i < nDevices; i++)
  {
    std::string device_ID = arv_get_device_id(i);
    //ROS_INFO("#Device: %s", device_ID.c_str());

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

      if(requested_cameras.size() == 1 &&
         std::all_of(requested_cameras[0].begin(), requested_cameras[0].end(), [](char c){ return isspace(c);}))
      {
        requested_cameras.clear();
      }

      for(auto& item : requested_cameras)
      {
        size_t position = item.rfind('-');
        if (position!=std::string::npos && item.size()>(position+1))
        {
          item = item.substr(position+1);
        }
      }
    }
  }

  if (requested_cameras.size()==0)
  {
    for(auto &serial_deviceID : available_cameras)
    {
      requested_cameras.push_back(serial_deviceID.first);
    }
  }

  for (const auto &serial_deviceID : available_cameras)
  {
    if(std::find(requested_cameras.begin(),
                 requested_cameras.end(),
                 serial_deviceID.first) != requested_cameras.end())
    {
      auto iter = cameras.find(serial_deviceID.first);
      if( iter != cameras.end() && iter->second->getCameraState() == CameraState::NOTINITIALIZED)
      {
        ROS_INFO("Reconnect to camera: %s  with device id: %s",
                 serial_deviceID.first.c_str(), serial_deviceID.second.c_str());

        iter->second->reestablishConnection(phNodeHandle);
      }
      else if(iter == cameras.end())
      {
        // Open the camera, and set it up.
        ROS_INFO("Add requested camera: %s  with device id: %s",
                 serial_deviceID.first.c_str(), serial_deviceID.second.c_str());

        cameras.emplace(std::piecewise_construct,
                        std::forward_as_tuple(serial_deviceID.first),
                        std::forward_as_tuple(std::make_shared<GeniCam>(phNodeHandle, serial_deviceID)));
      }
    }

  }
}


xamla_sysmon_msgs::HeartBeat CameraManager::createHeartbeatMessage()
{
  bool isGO = true;
  xamla_sysmon_msgs::HeartBeat msg;

  if(requested_cameras.size() != cameras.size())
  {
    isGO = false;
  }

  for(auto &camera : cameras)
  {
    if(camera.second->getCameraState() == CameraState::NOTINITIALIZED)
    {
      std::cout<<"wrong"<<std::endl;
      isGO = false;
      break;
    }
  }

  if(isGO == true)
  {
    msg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::GO);
  }
  else
  {
    msg.status = static_cast<int>(TopicHeartbeatStatus::TopicCode::INTERNAL_ERROR);
  }

  msg.details = TopicHeartbeatStatus::generateMessageText(TopicHeartbeatStatus::intToStatusCode(msg.status));
  msg.header.stamp = ros::Time::now();

  return msg;
}
