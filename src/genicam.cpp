#include "camera_aravis/genicam.h"

// -- defines --

#define ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pixel_format)                           \
  (((pixel_format) >> 16) & 0xff)
#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format)                          \
  ((((pixel_format) >> 16) & 0xff) >> 3)

const char* szBufferStatusFromInt[] = {
    "ARV_BUFFER_STATUS_SUCCESS",         "ARV_BUFFER_STATUS_CLEARED",
    "ARV_BUFFER_STATUS_TIMEOUT",         "ARV_BUFFER_STATUS_MISSING_PACKETS",
    "ARV_BUFFER_STATUS_WRONG_PACKET_ID", "ARV_BUFFER_STATUS_SIZE_MISMATCH",
    "ARV_BUFFER_STATUS_FILLING",         "ARV_BUFFER_STATUS_ABORTED"};

// function

static void stream_cb (void *user_data, ArvStreamCallbackType type, ArvBuffer *buffer)
{
  if (type == ARV_STREAM_CALLBACK_TYPE_INIT) {
    if (!arv_make_thread_realtime (10))
      g_warning ("Failed to make stream thread real time");
    if(!arv_make_thread_high_priority (-10))
      g_warning ("Failed to make stream thread high priority");
  }
}



// --- constructor ---

GeniCam::GeniCam(std::shared_ptr<ros::NodeHandle> &phNode, const std::pair<const std::string, std::string> &serial_deviceid) :
  pImageTransport(std::make_shared<image_transport::ImageTransport>(*phNode)),
  pCameraInfoManager(std::make_shared<camera_info_manager::CameraInfoManager>(
                       ros::NodeHandle(ros::this_node::getName()+"/"+serial_deviceid.first),
                       serial_deviceid.first))
{
  serialNumber = serial_deviceid.first;
  deviceID = serial_deviceid.second;
  reestablishConnection(phNode);
}

GeniCam::~GeniCam()
{
  handleConnectionLoss();
}

//-- public methods --

void GeniCam::newbuffer_callback(ArvStream *pStream, GeniCam *pCameradata)
{
  pCameradata->processNewBuffer(pStream);
}

void GeniCam::connectionlost_callback(ArvGvDevice *pGvDevice, GeniCam *pCameradata)
{
  ROS_WARN("Connection loss detected");
  pCameradata->handleConnectionLoss();
}

bool GeniCam::reestablishConnection(std::shared_ptr<ros::NodeHandle> &phNode)
{
  cameraState.store(CameraState::NOTINITIALIZED);

  try
  {
    pCamera = NULL;
    pCamera = arv_camera_new(deviceID.c_str());
    if (!pCamera)
    {
      throw std::runtime_error(("camera with ID: "+ serialNumber +" could not be opened. skip."));
    }

    arv_camera_gv_auto_packet_size(pCamera);

    pDevice = arv_camera_get_device(pCamera);

    ROS_INFO("Opened: %s-%s", arv_device_get_string_feature_value(
                              pDevice, "DeviceVendorName"),
           arv_device_get_string_feature_value(pDevice, "DeviceID"));

    pStream = NULL;

    pStream = createStream(serialNumber);
    if (!pStream)
    {
      throw std::runtime_error(("stream for camera with ID: "+ serialNumber +" could not be opened. skip."));
    }

    // Set up image_raw.
    publisher = pImageTransport->advertiseCamera(
        ros::this_node::getName() + "/" + serialNumber + "/image_raw", 1,
        std::bind(&GeniCam::connectCallback, this),
        std::bind(&GeniCam::disconnectCallback, this));

    // Connect signals with callbacks.
    newbufferHandlerID = g_signal_connect(pStream,
                     "new-buffer", G_CALLBACK(GeniCam::newbuffer_callback),
                     this);
    connectionLostHandlerID = g_signal_connect(pDevice, "control-lost",
                     G_CALLBACK(GeniCam::connectionlost_callback),
                     this);

    if (!genicamFeatures.init(
          phNode, pDevice, serialNumber))
    {
      throw std::runtime_error("parameter for camera with ID: " + serialNumber +" could not be initialized. skip");
    }

    frame_id = "camera_"+serialNumber;
    phNode->getParam(ros::this_node::getName() + "/" +
                     serialNumber + "/frame_id",
                     frame_id);

    sequenceCounter = 0;
    nBuffers = 0;

    cm = 0L;
    tm = 0L;
    em = 0L;

    xRoi = 0;
    yRoi = 0;
    widthRoi = 0;
    heightRoi = 0;
    arv_camera_get_region(pCamera,
                          &xRoi,
                          &yRoi,
                          &widthRoi,
                          &heightRoi);

    pszPixelformat =
        g_string_ascii_down(g_string_new(arv_device_get_string_feature_value(
                                pDevice, "PixelFormat")))->str;
    nBytesPixel = ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(
        arv_device_get_integer_feature_value(pDevice, "PixelFormat"));

    arv_device_execute_command(pDevice,
                               "AcquisitionStop");

    arv_device_set_string_feature_value (pDevice, "AcquisitionMode", "Continuous");
    arv_device_set_string_feature_value (pDevice, "TriggerSelector", "ExposureStart");
    arv_device_set_string_feature_value (pDevice, "TriggerMode", "On");
    arv_device_set_string_feature_value (pDevice, "TriggerSource", "Software");

    arv_device_execute_command(pDevice,
                               "AcquisitionStart");

    arv_stream_set_emit_signals(
          (ArvStream*)pStream, TRUE);


    cameraState.store(CameraState::READY);

    return true;

  } catch (const std::exception& e)
  {
    ROS_WARN("%s", e.what());
    return false;
  }
}

CameraState GeniCam::getCameraState()
{
  return cameraState.load();
}

void GeniCam::showStatistic()
{
  if(cameraState.load() != CameraState::NOTINITIALIZED)
  {
    guint64 n_completed_buffers;
    guint64 n_failures;
    guint64 n_underruns;
    guint64 n_resent;
    guint64 n_missing;
    arv_stream_get_statistics((ArvStream*)pStream, &n_completed_buffers,
                              &n_failures, &n_underruns);
    ROS_INFO("Completed buffers = %Lu",
             (unsigned long long)n_completed_buffers);
    ROS_INFO("Failures          = %Lu", (unsigned long long)n_failures);
    ROS_INFO("Underruns         = %Lu", (unsigned long long)n_underruns);
    arv_gv_stream_get_statistics(pStream, &n_resent, &n_missing);
    ROS_INFO("Resent buffers    = %Lu", (unsigned long long)n_resent);
    ROS_INFO("Missing           = %Lu", (unsigned long long)n_missing);
  }
}

bool GeniCam::capture(std::vector<sensor_msgs::Image> &imageContainer)
{
  std::lock_guard<std::mutex> lck(aquisitionChangeMutex);
  bool wasStreaming = false;
  bool changeTriggerMode = false;
  try
  {
    if(!genicamFeatures.is_implemented("TriggerMode") ||
       !genicamFeatures.is_implemented("TriggerSource") ||
       !genicamFeatures.is_implemented("AcquisitionMode") ||
       !genicamFeatures.is_implemented("ExposureTime"))
    {
      throw std::runtime_error("Capture: can not be used because "
                         "software and hardware triggering is not "
                         "supported by camera with serial number: " + serialNumber);
    }
    if(cameraState.load() != CameraState::NOTINITIALIZED)
    {
      //auto start = std::chrono::high_resolution_clock::now();
      if (cameraState.load() == CameraState::STREAMING)
      {
        wasStreaming = true;
        arv_device_execute_command(pDevice,
                                   "AcquisitionStop");
        arv_device_set_string_feature_value (pDevice, "TriggerSelector", "ExposureStart");
        arv_device_set_string_feature_value (pDevice, "TriggerMode", "On");
        arv_device_set_string_feature_value (pDevice, "TriggerSource", "Software");
        arv_device_execute_command(pDevice,
                                 "AcquisitionStart");
      }
      cameraState.store(CameraState::CAPTURING);
      changeTriggerMode = true;

      std::chrono::microseconds wait_time = std::chrono::microseconds(long(arv_device_get_float_feature_value(pDevice, "ExposureTime")));
      // set wait_time to a min because the image transport via ethernet is not infinity fast
      if(wait_time < std::chrono::microseconds(20000))
      {
        wait_time = std::chrono::microseconds(20000);
      }
      //std::chrono::milliseconds wait_time = std::chrono::milliseconds(1000);
      size_t tries = 3;
      for(size_t i = 0; i<tries; i++)
      {
        std::unique_lock<std::mutex> lck(imageWaitMutex);
        arv_device_execute_command(pDevice, "TriggerSoftware");
        if(newImageAvailable.wait_for(lck, wait_time*5)==std::cv_status::no_timeout)
        {
          imageContainer.push_back(imageMsg);
          break;
        }
        else if(i==tries-1)
        {
          throw std::runtime_error("Capture: after 5 times the "
                                   "exposure time or min 100ms and 3 tries an new image was still "
                                   "not available abort; serial: " + serialNumber);
        }
      }
      restoreAquistionsMode(wasStreaming);
      //auto t3 = std::chrono::high_resolution_clock::now();
      //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t3-start);
      //std::cout<<"query image takes: "<<duration.count()<<"ms"<<std::endl;
    }
    else
    {
      throw std::runtime_error("Capture: camera with serial "
                         + serialNumber + "is not available");
    }
  } catch(const std::exception& e)
  {
    if(changeTriggerMode == true)
    {
      restoreAquistionsMode(wasStreaming);
    }

    ROS_WARN("%s", e.what());
    return false;
  }
  return true;
}

bool GeniCam::tryToSetFeatureValue(const std::string &feature, const std::string &value)
{
  bool wasStreaming = false;
  bool setSucessful = false;
  if(genicamFeatures.is_implemented(feature) &&
     (cameraState.load() != CameraState::NOTINITIALIZED))
  {
    std::lock_guard<std::mutex> lck(aquisitionChangeMutex);
    if(cameraState.load() == CameraState::STREAMING)
    {
      wasStreaming = true;
      arv_device_execute_command(pDevice,
                               "AcquisitionStop");
    }

    setSucessful = genicamFeatures.get_feature(feature)->set_current_value(
          pDevice, value);

    if(wasStreaming == true)
    {
      arv_device_execute_command(pDevice,
                               "AcquisitionStart");
    }
  }
  if(setSucessful == true) return true;
  else return false;
}

bool GeniCam::tryToGetFeatureValue(const std::string &feature, std::string &value)
{
  if(genicamFeatures.is_implemented(feature))
  {
    if(cameraState.load() != CameraState::NOTINITIALIZED)
    {
      value = genicamFeatures.get_feature(feature)->get_current_value(pDevice);
    }
    return true;
  }
  else

  return false;
}

// -- private methods --

ArvGvStream* GeniCam::createStream(const std::string &camera_serial)
{
  gboolean bAutoBuffer = FALSE;
  gboolean bPacketResend = TRUE;
  unsigned int timeoutPacket = 2000; // milliseconds
  unsigned int timeoutFrameRetention = 3000;

  ArvGvStream* pStream =
      (ArvGvStream*)arv_device_create_stream(pDevice, stream_cb, NULL);
  if (pStream)
  {
    ArvBuffer* pBuffer;
    gint nbytesPayload;

    if (!ARV_IS_GV_STREAM(pStream))
      ROS_WARN("Stream is not a GV_STREAM");

    if (bAutoBuffer)
      g_object_set(pStream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                   "socket-buffer-size", 1000000, NULL);
    if (!bPacketResend)
      g_object_set(pStream, "packet-resend",
                   bPacketResend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS
                                 : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                   NULL);
    g_object_set(pStream, "packet-timeout", (unsigned)timeoutPacket * 1000,
                 "frame-retention", (unsigned)timeoutFrameRetention * 1000,
                 NULL);

    // Load up some buffers.
    nbytesPayload = arv_camera_get_payload(pCamera);
    for (int i = 0; i < 50; i++)
    {
      pBuffer = arv_buffer_new(nbytesPayload, NULL);
      arv_stream_push_buffer((ArvStream*)pStream, pBuffer);
    }
  }
  return pStream;
} // CreateStream()

void GeniCam::processNewBuffer(ArvStream *pStream)
{
  uint64_t cn;        // Camera time now

  uint64_t rn; // ROS time now

  uint64_t tn;        // Calculated image time now

  int64_t en; // Error now between calculated image time and ROS time.
  int64_t de; // derivative.
  int64_t ie; // integral.
  int64_t u;  // Output of controller.

  int64_t kp1 = 0L; // Fractional gains in integer form.
  int64_t kp2 = 1024L;
  int64_t kd1 = 0L;
  int64_t kd2 = 1024L;
  int64_t ki1 = -1L; // A gentle pull toward zero.
  int64_t ki2 = 1024L;

  ArvBuffer* pBuffer = NULL;

  pBuffer = arv_stream_try_pop_buffer(pStream);
  if (pBuffer != NULL)
  {
    if (arv_buffer_get_status(pBuffer) == ARV_BUFFER_STATUS_SUCCESS)
    {

      nBuffers++;
      size_t pSize = 0;
      const void* pData = arv_buffer_get_data(pBuffer, &pSize);
      std::vector<uint8_t> this_data(pSize);
      memcpy(&this_data[0], pData, pSize);

      // Camera/ROS Timestamp coordination.
      cn = (uint64_t)arv_buffer_get_timestamp(pBuffer); // Camera now
      rn = ros::Time::now().toNSec();                   // ROS now
      sequenceCounter++;

      if (sequenceCounter < 10)
      {
        cm = cn;
        tm = rn;
      }

      // Control the error between the computed image timestamp and the ROS
      // timestamp.
      en = (int64_t)tm + (int64_t)cn - (int64_t)cm -
           (int64_t)rn; // i.e. tn-rn, but calced from prior values.
      de = en - em;
      ie += en;
      u = kp1 * (en / kp2) + ki1 * (ie / ki2) +
          kd1 * (de / kd2); // kp<0, ki<0, kd>0

      // Compute the new timestamp.
      tn = (uint64_t)((int64_t)tm + (int64_t)cn - (int64_t)cm + u);

      // Save prior values.
      cm = cn;
      tm = tn;
      em = en;

      std::unique_lock<std::mutex> lck(imageWaitMutex);
      // Construct the image message.
      imageMsg.header.stamp.fromNSec(tn);
      imageMsg.header.seq = sequenceCounter;
      imageMsg.header.frame_id = frame_id;
      imageMsg.width = widthRoi;
      imageMsg.height = heightRoi;
      imageMsg.encoding = pszPixelformat;
      imageMsg.step = imageMsg.width * nBytesPixel;
      imageMsg.data = this_data;

      // get current CameraInfo data
      camerainfo = pCameraInfoManager->getCameraInfo();
      camerainfo.header.stamp = imageMsg.header.stamp;
      camerainfo.header.seq = imageMsg.header.seq;
      camerainfo.header.frame_id = imageMsg.header.frame_id;
      camerainfo.width = widthRoi;
      camerainfo.height = heightRoi;
      lck.unlock();
      newImageAvailable.notify_all();

      publisher.publish(imageMsg, camerainfo);
    }
    else
      ROS_WARN("Frame error: %s",
               szBufferStatusFromInt[arv_buffer_get_status(pBuffer)]);

    arv_stream_push_buffer(pStream, pBuffer);
  }
}

void GeniCam::handleConnectionLoss()
{
  if(cameraState.load() != CameraState::NOTINITIALIZED)
  {
    arv_device_execute_command(pDevice,
                               "AcquisitionStop");

    publisher.shutdown();

    arv_stream_set_emit_signals(
          (ArvStream*)pStream, FALSE);

    g_signal_handler_disconnect(pStream, newbufferHandlerID);
    g_signal_handler_disconnect(pDevice, connectionLostHandlerID);

    g_clear_object(&pStream);
    g_clear_object(&pStream);

    cameraState.store(CameraState::NOTINITIALIZED);
  }
}

void GeniCam::connectCallback()
{
  if (publisher.getNumSubscribers() == 1){
    std::lock_guard<std::mutex> lck(aquisitionChangeMutex);
    cameraState.store(CameraState::STREAMING);
    arv_device_execute_command(pDevice,
                               "AcquisitionStop");
    arv_device_set_string_feature_value (pDevice, "TriggerMode", "Off");
    arv_device_set_string_feature_value (pDevice, "TriggerSelector", "AcquisitionStart");
    arv_device_execute_command(pDevice,
                               "AcquisitionStart");

    std::string info_text = "someone subscribe to topic " + publisher.getTopic() + " start continuous image aquisition";
    ROS_INFO("%s",info_text.c_str());
  }
}

void GeniCam::disconnectCallback()
{
  if (publisher.getNumSubscribers() == 0){
    std::lock_guard<std::mutex> lck(aquisitionChangeMutex);
    arv_device_execute_command(pDevice,
                               "AcquisitionStop");
    arv_device_set_string_feature_value (pDevice, "TriggerSelector", "ExposureStart");
    arv_device_set_string_feature_value (pDevice, "TriggerMode", "On");
    arv_device_set_string_feature_value (pDevice, "TriggerSource", "Software");
    arv_device_execute_command(pDevice,
                               "AcquisitionStart");
    cameraState.store(CameraState::READY);

    std::string info_text = "number of subscribers to topic " + publisher.getTopic() + " falls to 0. Continuous image aquisition stopped";
    ROS_INFO("%s", info_text.c_str());
  }
}

void GeniCam::restoreAquistionsMode(const bool& wasStreaming)
{
  if(wasStreaming == true)
  {
    arv_device_execute_command(pDevice,
                               "AcquisitionStop");
    arv_device_set_string_feature_value (pDevice, "TriggerMode", "Off");
    arv_device_set_string_feature_value (pDevice, "TriggerSelector", "AcquisitionStart");
    arv_device_execute_command(pDevice,
                               "AcquisitionStart");
    cameraState.store(CameraState::STREAMING);
  }
  else
  {
    cameraState.store(CameraState::READY);
  }
}
