/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */

// camera_aravis
//
// This is a ROS node that operates GenICam-based cameras via the Aravis
// library.
// Commonly available camera features are supported through the
// dynamic_reconfigure user-interface and GUI,
// and for those features not in the GUI but that are specific to a camera, they
// can be set in the
// camera by setting the appropriate parameter at startup.  This code reads
// those parameters, and
// if any of them match a camera feature, then the camera is written to.
//
// For example, if a camera has a feature called "IRFormat" that is an integer
// 0, 1, or 2, you can do
// rosparam set camnode/IRFormat 2
// and this driver will write it to the camera at startup.  Note that the
// datatype of the parameter
// must be correct for the camera feature (e.g. bool, int, double, string, etc),
// so for example you should use
// rosparam set camnode/GainAuto true
// and NOT
// rosparam set camnode/GainAuto 1
//

#include <arvbuffer.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <mutex>
#include <stdlib.h>
#include <math.h>

#include <glib.h>

#include <ros/time.h>
#include <ros/duration.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <camera_aravis/CameraAravisConfig.h>

//#include "camera_aravis/SoftwareTrigger.h"
#include <std_srvs/Empty.h>
#include "camera_aravis/DOMTreeHelper.h"
#include "XmlRpc.h"

//#define TUNING	// Allows tuning the gains for the timestamp controller.
//Publishes output on topic /dt, and receives gains on params /kp, /ki, /kd

#define CLIP(x, lo, hi) MIN(MAX((lo), (x)), (hi))
#define THROW_ERROR(m) throw std::string((m))

#define TRIGGERSOURCE_SOFTWARE 0
#define TRIGGERSOURCE_LINE1 1
#define TRIGGERSOURCE_LINE2 2

#define ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pixel_format)                           \
  (((pixel_format) >> 16) & 0xff)
#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format)                          \
  ((((pixel_format) >> 16) & 0xff) >> 3)
typedef camera_aravis::CameraAravisConfig Config;

// static gboolean SoftwareTrigger_callback (void *);
bool SoftwareTrigger_callback(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response);

// Global variables -------------------
struct GeniCam
{
  bool isRunning;

  std::shared_ptr<image_transport::ImageTransport> pTransport;
  image_transport::CameraPublisher publisher;
  std::shared_ptr<camera_info_manager::CameraInfoManager> pCameraInfoManager;
  sensor_msgs::CameraInfo camerainfo;

  // aravis g_objects
  ArvCamera* pCamera;
  ArvDevice* pDevice;
  ArvGvStream* pStream;

  gint width, height; // buffer->width and buffer->height not working, so I used
                      // a global.
  Config config;
  Config configMin;
  Config configMax;

  int isImplementedAcquisitionFrameRate;
  int isImplementedAcquisitionFrameRateEnable;
  int isImplementedGain;
  int isImplementedExposureTimeAbs;
  int isImplementedExposureAuto;
  int isImplementedGainAuto;
  int isImplementedFocusPos;
  int isImplementedTriggerSelector;
  int isImplementedTriggerSource;
  int isImplementedTriggerMode;
  int isImplementedAcquisitionMode;
  int isImplementedMtu;

  int xRoi;
  int yRoi;
  int widthRoi;
  int widthRoiMin;
  int widthRoiMax;
  int heightRoi;
  int heightRoiMin;
  int heightRoiMax;

  int widthSensor;
  int heightSensor;

  const char* pszPixelformat;
  unsigned nBytesPixel;
  int mtu;
  int Acquire;
  const char* keyAcquisitionFrameRate;

  size_t frame_id;
  size_t nBuffers; // Counter for Hz calculation.
#ifdef TUNING
  std::unique_ptr<ros::Publisher> ppubInt64;
#endif

};

struct Global
{
  std::shared_ptr<ros::NodeHandle> phNode;
  int idSoftwareTriggerTimer;
  gboolean bCancel;

  std::unordered_map<std::string, GeniCam> cameras;
} global;

typedef struct
{
  GMainLoop* main_loop;
} ApplicationData;

std::mutex NewBufferMutex;
// ------------------------------------

// Conversions from integers to Arv types.
const char* szBufferStatusFromInt[] = {
    "ARV_BUFFER_STATUS_SUCCESS",         "ARV_BUFFER_STATUS_CLEARED",
    "ARV_BUFFER_STATUS_TIMEOUT",         "ARV_BUFFER_STATUS_MISSING_PACKETS",
    "ARV_BUFFER_STATUS_WRONG_PACKET_ID", "ARV_BUFFER_STATUS_SIZE_MISMATCH",
    "ARV_BUFFER_STATUS_FILLING",         "ARV_BUFFER_STATUS_ABORTED"};

static void set_cancel(int signal){ global.bCancel = TRUE; }


static void stream_cb (void *user_data, ArvStreamCallbackType type, ArvBuffer *buffer)
{
  if (type == ARV_STREAM_CALLBACK_TYPE_INIT) {
    if (!arv_make_thread_realtime (10) &&
        !arv_make_thread_high_priority (-10))
      g_warning ("Failed to make stream thread high priority");
  }
}

ArvGvStream* CreateStream(const std::string &camera_serial)
{
  gboolean bAutoBuffer = FALSE;
  gboolean bPacketResend = TRUE;
  unsigned int timeoutPacket = 40; // milliseconds
  unsigned int timeoutFrameRetention = 200;

  ArvGvStream* pStream =
      (ArvGvStream*)arv_device_create_stream(global.cameras[camera_serial].pDevice, stream_cb, NULL);
  if (pStream)
  {
    ArvBuffer* pBuffer;
    gint nbytesPayload;

    if (!ARV_IS_GV_STREAM(pStream))
      ROS_WARN("Stream is not a GV_STREAM");

    if (bAutoBuffer)
      g_object_set(pStream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                   "socket-buffer-size", 0, NULL);
    if (!bPacketResend)
      g_object_set(pStream, "packet-resend",
                   bPacketResend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS
                                 : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                   NULL);
    g_object_set(pStream, "packet-timeout", (unsigned)timeoutPacket * 1000,
                 "frame-retention", (unsigned)timeoutFrameRetention * 1000,
                 NULL);

    // Load up some buffers.
    nbytesPayload = arv_camera_get_payload(global.cameras[camera_serial].pCamera);
    for (int i = 0; i < 50; i++)
    {
      pBuffer = arv_buffer_new(nbytesPayload, NULL);
      arv_stream_push_buffer((ArvStream*)pStream, pBuffer);
    }
  }
  return pStream;
} // CreateStream()

/*
void RosReconfigure_callback(Config& config, uint32_t level)
{
  int changedAcquire;
  int changedAcquisitionFrameRate;
  int changedExposureAuto;
  int changedGainAuto;
  int changedExposureTimeAbs;
  int changedGain;
  int changedAcquisitionMode;
  int changedTriggerMode;
  int changedTriggerSource;
  int changedSoftwarerate;
  int changedFrameid;
  int changedFocusPos;
  int changedMtu;

  std::string tf_prefix = tf::getPrefixParam(*global.phNode);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);

  if (config.frame_id == "")
    config.frame_id = "camera";

  // Find what the user changed.
  changedAcquire = (global.config.Acquire != config.Acquire);
  changedAcquisitionFrameRate =
      (global.config.AcquisitionFrameRate != config.AcquisitionFrameRate);
  changedExposureAuto = (global.config.ExposureAuto != config.ExposureAuto);
  changedExposureTimeAbs =
      (global.config.ExposureTimeAbs != config.ExposureTimeAbs);
  changedGainAuto = (global.config.GainAuto != config.GainAuto);
  changedGain = (global.config.Gain != config.Gain);
  changedAcquisitionMode =
      (global.config.AcquisitionMode != config.AcquisitionMode);
  changedTriggerMode = (global.config.TriggerMode != config.TriggerMode);
  changedTriggerSource = (global.config.TriggerSource != config.TriggerSource);
  changedSoftwarerate =
      (global.config.softwaretriggerrate != config.softwaretriggerrate);
  changedFrameid = (global.config.frame_id != config.frame_id);
  changedFocusPos = (global.config.FocusPos != config.FocusPos);
  changedMtu = (global.config.mtu != config.mtu);

  // Limit params to legal values.
  config.AcquisitionFrameRate =
      CLIP(config.AcquisitionFrameRate, global.configMin.AcquisitionFrameRate,
           global.configMax.AcquisitionFrameRate);
  config.ExposureTimeAbs =
      CLIP(config.ExposureTimeAbs, global.configMin.ExposureTimeAbs,
           global.configMax.ExposureTimeAbs);
  config.Gain = CLIP(config.Gain, global.configMin.Gain, global.configMax.Gain);
  config.FocusPos = CLIP(config.FocusPos, global.configMin.FocusPos,
                         global.configMax.FocusPos);
  config.frame_id = tf::resolve(tf_prefix, config.frame_id);

  // Adjust other controls dependent on what the user changed.
  if (changedExposureTimeAbs || changedGainAuto ||
      ((changedAcquisitionFrameRate || changedGain || changedFrameid ||
        changedAcquisitionMode || changedTriggerSource ||
        changedSoftwarerate) &&
       config.ExposureAuto == "Once"))
    config.ExposureAuto = "Off";

  if (changedGain || changedExposureAuto ||
      ((changedAcquisitionFrameRate || changedExposureTimeAbs ||
        changedFrameid || changedAcquisitionMode || changedTriggerSource ||
        changedSoftwarerate) &&
       config.GainAuto == "Once"))
    config.GainAuto = "Off";

  if (changedAcquisitionFrameRate)
    config.TriggerMode = "Off";

  // Find what changed for any reason.
  changedAcquire = (global.config.Acquire != config.Acquire);
  changedAcquisitionFrameRate =
      (global.config.AcquisitionFrameRate != config.AcquisitionFrameRate);
  changedExposureAuto = (global.config.ExposureAuto != config.ExposureAuto);
  changedExposureTimeAbs =
      (global.config.ExposureTimeAbs != config.ExposureTimeAbs);
  changedGainAuto = (global.config.GainAuto != config.GainAuto);
  changedGain = (global.config.Gain != config.Gain);
  changedAcquisitionMode =
      (global.config.AcquisitionMode != config.AcquisitionMode);
  changedTriggerMode = (global.config.TriggerMode != config.TriggerMode);
  changedTriggerSource = (global.config.TriggerSource != config.TriggerSource);
  changedSoftwarerate =
      (global.config.softwaretriggerrate != config.softwaretriggerrate);
  changedFrameid = (global.config.frame_id != config.frame_id);
  changedFocusPos = (global.config.FocusPos != config.FocusPos);
  changedMtu = (global.config.mtu != config.mtu);

  // Set params into the camera.
  if (changedExposureTimeAbs)
  {
    if (global.isImplementedExposureTimeAbs)
    {
      ROS_INFO("Set ExposureTimeAbs = %f", config.ExposureTimeAbs);
      arv_device_set_float_feature_value(global.pDevice, "ExposureTimeAbs",
                                         config.ExposureTimeAbs);
    }
    else
      ROS_INFO("Camera does not support ExposureTimeAbs.");
  }

  if (changedGain)
  {
    if (global.isImplementedGain)
    {
      ROS_INFO("Set gain = %f", config.Gain);
      // arv_device_set_integer_feature_value (global.pDevice, "GainRaw",
      // config.GainRaw);
      arv_camera_set_gain(global.pCamera, config.Gain);
    }
    else
      ROS_INFO("Camera does not support Gain or GainRaw.");
  }

  if (changedExposureAuto)
  {
    if (global.isImplementedExposureAuto && global.isImplementedExposureTimeAbs)
    {
      ROS_INFO("Set ExposureAuto = %s", config.ExposureAuto.c_str());
      arv_device_set_string_feature_value(global.pDevice, "ExposureAuto",
                                          config.ExposureAuto.c_str());
      if (config.ExposureAuto == "Once")
      {
        ros::Duration(2.0).sleep();
        config.ExposureTimeAbs = arv_device_get_float_feature_value(
            global.pDevice, "ExposureTimeAbs");
        ROS_INFO("Get ExposureTimeAbs = %f", config.ExposureTimeAbs);
        config.ExposureAuto = "Off";
      }
    }
    else
      ROS_INFO("Camera does not support ExposureAuto.");
  }
  if (changedGainAuto)
  {
    if (global.isImplementedGainAuto && global.isImplementedGain)
    {
      ROS_INFO("Set GainAuto = %s", config.GainAuto.c_str());
      arv_device_set_string_feature_value(global.pDevice, "GainAuto",
                                          config.GainAuto.c_str());
      if (config.GainAuto == "Once")
      {
        ros::Duration(2.0).sleep();
        // config.GainRaw = arv_device_get_integer_feature_value
        // (global.pDevice, "GainRaw");
        config.Gain = arv_camera_get_gain(global.pCamera);
        ROS_INFO("Get Gain = %f", config.Gain);
        config.GainAuto = "Off";
      }
    }
    else
      ROS_INFO("Camera does not support GainAuto.");
  }

  if (changedAcquisitionFrameRate)
  {
    if (global.isImplementedAcquisitionFrameRate)
    {
      ROS_INFO("Set %s = %f", global.keyAcquisitionFrameRate,
               config.AcquisitionFrameRate);
      arv_device_set_float_feature_value(global.pDevice,
                                         global.keyAcquisitionFrameRate,
                                         config.AcquisitionFrameRate);
    }
    else
      ROS_INFO("Camera does not support AcquisitionFrameRate.");
  }

  if (changedTriggerMode)
  {
    if (global.isImplementedTriggerMode)
    {
      ROS_INFO("Set TriggerMode = %s", config.TriggerMode.c_str());
      arv_device_set_string_feature_value(global.pDevice, "TriggerMode",
                                          config.TriggerMode.c_str());
    }
    else
      ROS_INFO("Camera does not support TriggerMode.");
  }

  if (changedTriggerSource)
  {
    if (global.isImplementedTriggerSource)
    {
      ROS_INFO("Set TriggerSource = %s", config.TriggerSource.c_str());
      arv_device_set_string_feature_value(global.pDevice, "TriggerSource",
                                          config.TriggerSource.c_str());
    }
    else
      ROS_INFO("Camera does not support TriggerSource.");
  }

  if ((changedTriggerMode || changedTriggerSource || changedSoftwarerate) &&
      config.TriggerMode == "On" && config.TriggerSource == "Software")
  {
    if (global.isImplementedAcquisitionFrameRate)
    {
      // The software rate is limited by the camera's internal framerate.  Bump
      // up the camera's internal framerate if necessary.
      config.AcquisitionFrameRate = global.configMax.AcquisitionFrameRate;
      ROS_INFO("Set %s = %f", global.keyAcquisitionFrameRate,
               config.AcquisitionFrameRate);
      arv_device_set_float_feature_value(global.pDevice,
                                         global.keyAcquisitionFrameRate,
                                         config.AcquisitionFrameRate);
    }
  }

  if (changedTriggerSource || changedSoftwarerate)
  {
    // Recreate the software trigger callback.
    if (global.idSoftwareTriggerTimer)
    {
      g_source_remove(global.idSoftwareTriggerTimer);
      global.idSoftwareTriggerTimer = 0;
    }
    if (!strcmp(config.TriggerSource.c_str(), "Software"))
    {
      ROS_INFO("Implementation changed, Software Trigger is now a service, "
               "changing trigger rate won't do anything");
      // ROS_INFO ("Set softwaretriggerrate = %f", 1000.0/ceil(1000.0 /
      // config.softwaretriggerrate));

      // Turn on software timer callback.
      // global.idSoftwareTriggerTimer = g_timeout_add ((guint)ceil(1000.0 /
      // config.softwaretriggerrate), SoftwareTrigger_callback, global.pCamera);
    }
  }
  if (changedFocusPos)
  {
    if (global.isImplementedFocusPos)
    {
      ROS_INFO("Set FocusPos = %d", config.FocusPos);
      arv_device_set_integer_feature_value(global.pDevice, "FocusPos",
                                           config.FocusPos);
      ros::Duration(1.0).sleep();
      config.FocusPos =
          arv_device_get_integer_feature_value(global.pDevice, "FocusPos");
      ROS_INFO("Get FocusPos = %d", config.FocusPos);
    }
    else
      ROS_INFO("Camera does not support FocusPos.");
  }
  if (changedMtu)
  {
    if (global.isImplementedMtu)
    {
      ROS_INFO("Set mtu = %d", config.mtu);
      arv_device_set_integer_feature_value(global.pDevice, "GevSCPSPacketSize",
                                           config.mtu);
      ros::Duration(1.0).sleep();
      config.mtu = arv_device_get_integer_feature_value(global.pDevice,
                                                        "GevSCPSPacketSize");
      ROS_INFO("Get mtu = %d", config.mtu);
    }
    else
      ROS_INFO("Camera does not support mtu (i.e. GevSCPSPacketSize).");
  }

  if (changedAcquisitionMode)
  {
    if (global.isImplementedAcquisitionMode)
    {
      ROS_INFO("Set AcquisitionMode = %s", config.AcquisitionMode.c_str());
      arv_device_set_string_feature_value(global.pDevice, "AcquisitionMode",
                                          config.AcquisitionMode.c_str());

      ROS_INFO("AcquisitionStop");
      arv_device_execute_command(global.pDevice, "AcquisitionStop");
      ROS_INFO("AcquisitionStart");
      arv_device_execute_command(global.pDevice, "AcquisitionStart");
    }
    else
      ROS_INFO("Camera does not support AcquisitionMode.");
  }

  if (changedAcquire)
  {
    if (config.Acquire)
    {
      ROS_INFO("AcquisitionStart");
      arv_device_execute_command(global.pDevice, "AcquisitionStart");
    }
    else
    {
      ROS_INFO("AcquisitionStop");
      arv_device_execute_command(global.pDevice, "AcquisitionStop");
    }
  }

  global.config = config;

} // RosReconfigure_callback()
*/

static void NewBuffer_callback(ArvStream* pStream,
                               GeniCam* pCameradata)
{
  static uint64_t cm = 0L; // Camera time prev
  uint64_t cn = 0L;        // Camera time now

#ifdef TUNING
  static uint64_t rm = 0L; // ROS time prev
#endif
  uint64_t rn = 0L; // ROS time now

  static uint64_t tm = 0L; // Calculated image time prev
  uint64_t tn = 0L;        // Calculated image time now

  static int64_t em = 0L; // Error prev.
  int64_t en = 0L; // Error now between calculated image time and ROS time.
  int64_t de = 0L; // derivative.
  int64_t ie = 0L; // integral.
  int64_t u = 0L;  // Output of controller.

  int64_t kp1 = 0L; // Fractional gains in integer form.
  int64_t kp2 = 1024L;
  int64_t kd1 = 0L;
  int64_t kd2 = 1024L;
  int64_t ki1 = -1L; // A gentle pull toward zero.
  int64_t ki2 = 1024L;

  static uint32_t iFrame = 0; // Frame counter.

  ArvBuffer* pBuffer;

#ifdef TUNING
  std_msgs::Int64 msgInt64;
  int kp = 0;
  int kd = 0;
  int ki = 0;

  if (global.phNode->hasParam(ros::this_node::getName() + "/kp"))
  {
    global.phNode->getParam(ros::this_node::getName() + "/kp", kp);
    kp1 = kp;
  }

  if (global.phNode->hasParam(ros::this_node::getName() + "/kd"))
  {
    global.phNode->getParam(ros::this_node::getName() + "/kd", kd);
    kd1 = kd;
  }

  if (global.phNode->hasParam(ros::this_node::getName() + "/ki"))
  {
    global.phNode->getParam(ros::this_node::getName() + "/ki", ki);
    ki1 = ki;
  }
#endif

  pBuffer = arv_stream_try_pop_buffer(pStream);
  if (pBuffer != NULL)
  {
    if (arv_buffer_get_status(pBuffer) == ARV_BUFFER_STATUS_SUCCESS)
    {
      sensor_msgs::Image msg;

      pCameradata->nBuffers++;
      size_t pSize = 0;
      const void* pData = arv_buffer_get_data(pBuffer, &pSize);
      std::vector<uint8_t> this_data(pSize);
      memcpy(&this_data[0], pData, pSize);

      // Camera/ROS Timestamp coordination.
      cn = (uint64_t)arv_buffer_get_timestamp(pBuffer); // Camera now
      rn = ros::Time::now().toNSec();                   // ROS now
      pCameradata->frame_id++;

      if (iFrame < 10)
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

#ifdef TUNING
      ROS_WARN("en=%16ld, ie=%16ld, de=%16ld, u=%16ld + %16ld + %16ld = %16ld",
               en, ie, de, kp1 * (en / kp2), ki1 * (ie / ki2), kd1 * (de / kd2),
               u);
      ROS_WARN(
          "cn=%16lu, rn=%16lu, cn-cm=%8ld, rn-rm=%8ld, tn-tm=%8ld, tn-rn=%ld",
          cn, rn, cn - cm, rn - rm, (int64_t)tn - (int64_t)tm, tn - rn);
      msgInt64.data = tn - rn; // cn-cm+tn-tm; //
      global.ppubInt64->publish(msgInt64);
      rm = rn;
#endif

      // Save prior values.
      cm = cn;
      tm = tn;
      em = en;

      // Construct the image message.
      msg.header.stamp.fromNSec(tn);
      msg.header.seq = pCameradata->frame_id;
      msg.header.frame_id = pCameradata->config.frame_id;
      msg.width = pCameradata->widthRoi;
      msg.height = pCameradata->heightRoi;
      msg.encoding = pCameradata->pszPixelformat;
      msg.step = msg.width * pCameradata->nBytesPixel;
      msg.data = this_data;

      // get current CameraInfo data
      pCameradata->camerainfo = pCameradata->pCameraInfoManager->getCameraInfo();
      pCameradata->camerainfo.header.stamp = msg.header.stamp;
      pCameradata->camerainfo.header.seq = msg.header.seq;
      pCameradata->camerainfo.header.frame_id = msg.header.frame_id;
      pCameradata->camerainfo.width = pCameradata->widthRoi;
      pCameradata->camerainfo.height = pCameradata->heightRoi;

      pCameradata->publisher.publish(msg, pCameradata->camerainfo);
    }
    else
      ROS_WARN("Frame error: %s",
               szBufferStatusFromInt[arv_buffer_get_status(pBuffer)]);

    arv_stream_push_buffer(pStream, pBuffer);
    iFrame++;
  }
} // NewBuffer_callback()

static void ControlLost_callback(ArvGvDevice* pGvDevice)
{
  ROS_ERROR("Control lost.");

  global.bCancel = TRUE;
}

// static gboolean SoftwareTrigger_callback (void *pCamera)
// {
// 	arv_device_execute_command (global.pDevice, "TriggerSoftware");

//     return TRUE;
// }

bool SoftwareTrigger_callback(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response)
{
  //arv_camera_software_trigger(global.cameras[camera_serial].pCamera);
  ROS_INFO("Execueting Trigger");
  return true;
}

// PeriodicTask_callback()
// Check for termination, and spin for ROS.
static gboolean PeriodicTask_callback(void* applicationdata)
{
  ApplicationData *pData = (ApplicationData*)applicationdata;

  if (global.bCancel)
  {
    g_main_loop_quit(pData->main_loop);
    return FALSE;
  }

  if(ros::ok)
  {
    ros::spinOnce();
  }

  return TRUE;
} // PeriodicTask_callback()

// WriteCameraFeaturesFromRosparam()
// Read ROS parameters from this node's namespace, and see if each parameter has
// a similarly named & typed feature in the camera.  Then set the
// camera feature to that value.  For example, if the parameter camnode/Gain is
// set to 123.0, then we'll write 123.0 to the Gain feature
// in the camera.
//
// Note that the datatype of the parameter *must* match the datatype of the
// camera feature, and this can be determined by
// looking at the camera's XML file.  Camera enum's are string parameters,
// camera bools are false/true parameters (not 0/1),
// integers are integers, doubles are doubles, etc.
//
/*void WriteCameraFeaturesFromRosparam(void)
{
  XmlRpc::XmlRpcValue xmlrpcParams;
  XmlRpc::XmlRpcValue::iterator iter;
  ArvGcNode* pGcNode;
  GError* error = NULL;

  global.phNode->getParam(ros::this_node::getName(), xmlrpcParams);

  if (xmlrpcParams.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (iter = xmlrpcParams.begin(); iter != xmlrpcParams.end(); iter++)
    {
      std::string key = iter->first;
      pGcNode = arv_device_get_feature(global.pDevice, key.c_str());
      if (pGcNode && arv_gc_feature_node_is_implemented(
                         ARV_GC_FEATURE_NODE(pGcNode), &error))
      {
        //				unsigned long	typeValue =
        //arv_gc_feature_node_get_value_type((ArvGcFeatureNode *)pGcNode);
        //				ROS_INFO("%s cameratype=%lu, rosparamtype=%d",
        //key.c_str(), typeValue, static_cast<int>(iter->second.getType()));

        // We'd like to check the value types too, but typeValue is often given
        // as G_TYPE_INVALID, so ignore it.
        switch (iter->second.getType())
        {
        case XmlRpc::XmlRpcValue::
            TypeBoolean: // if
                         // ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeBoolean))//
                         // && (typeValue==G_TYPE_INT64))
        {
          int value = (bool)iter->second;
          arv_device_set_integer_feature_value(global.pDevice, key.c_str(),
                                               value);
          ROS_INFO("Read parameter (bool) %s: %d", key.c_str(), value);
        }
        break;

        case XmlRpc::XmlRpcValue::
            TypeInt: // if
                     // ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeInt))//
                     // && (typeValue==G_TYPE_INT64))
        {
          int value = (int)iter->second;
          arv_device_set_integer_feature_value(global.pDevice, key.c_str(),
                                               value);
          ROS_INFO("Read parameter (int) %s: %d", key.c_str(), value);
        }
        break;

        case XmlRpc::XmlRpcValue::
            TypeDouble: // if
                        // ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeDouble))//
                        // && (typeValue==G_TYPE_DOUBLE))
        {
          double value = (double)iter->second;
          arv_device_set_float_feature_value(global.pDevice, key.c_str(),
                                             value);
          ROS_INFO("Read parameter (float) %s: %f", key.c_str(), value);
        }
        break;

        case XmlRpc::XmlRpcValue::
            TypeString: // if
                        // ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeString))//
                        // && (typeValue==G_TYPE_STRING))
        {
          std::string value = (std::string)iter->second;
          arv_device_set_string_feature_value(global.pDevice, key.c_str(),
                                              value.c_str());
          ROS_INFO("Read parameter (string) %s: %s", key.c_str(),
                   value.c_str());
        }
        break;

        case XmlRpc::XmlRpcValue::TypeInvalid:
        case XmlRpc::XmlRpcValue::TypeDateTime:
        case XmlRpc::XmlRpcValue::TypeBase64:
        case XmlRpc::XmlRpcValue::TypeArray:
        case XmlRpc::XmlRpcValue::TypeStruct:
        default:
          ROS_WARN(
              "Unhandled rosparam type in WriteCameraFeaturesFromRosparam()");
        }
      }
    }
  }
} // WriteCameraFeaturesFromRosparam()*/

void init_genicam_parameter(const std::string &camera_serial)
{
  GError *error=NULL;

  const char* pkeyAcquisitionFrameRate[2] = {"AcquisitionFrameRate",
                                             "AcquisitionFrameRateAbs"};

  global.cameras[camera_serial].config = global.cameras[camera_serial].config.__getDefault__();

  // See if some basic camera features exist.
  ArvGcNode* pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "AcquisitionMode");
  global.cameras[camera_serial].isImplementedAcquisitionMode =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "GainRaw");
  global.cameras[camera_serial].isImplementedGain = ARV_GC_FEATURE_NODE(pGcNode)
                                 ? arv_gc_feature_node_is_implemented(
                                       ARV_GC_FEATURE_NODE(pGcNode), &error)
                                 : FALSE;
  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "Gain");
  global.cameras[camera_serial].isImplementedGain |= ARV_GC_FEATURE_NODE(pGcNode)
                                  ? arv_gc_feature_node_is_implemented(
                                        ARV_GC_FEATURE_NODE(pGcNode), &error)
                                  : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "ExposureTimeAbs");
  global.cameras[camera_serial].isImplementedExposureTimeAbs =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "ExposureAuto");
  global.cameras[camera_serial].isImplementedExposureAuto =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "GainAuto");
  global.cameras[camera_serial].isImplementedGainAuto =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "TriggerSelector");
  global.cameras[camera_serial].isImplementedTriggerSelector =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "TriggerSource");
  global.cameras[camera_serial].isImplementedTriggerSource =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "TriggerMode");
  global.cameras[camera_serial].isImplementedTriggerMode =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "FocusPos");
  global.cameras[camera_serial].isImplementedFocusPos =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  pGcNode = arv_device_get_feature(global.cameras[camera_serial].pDevice, "GevSCPSPacketSize");
  global.cameras[camera_serial].isImplementedMtu = ARV_GC_FEATURE_NODE(pGcNode)
                                ? arv_gc_feature_node_is_implemented(
                                      ARV_GC_FEATURE_NODE(pGcNode), &error)
                                : FALSE;

  pGcNode =
      arv_device_get_feature(global.cameras[camera_serial].pDevice, "AcquisitionFrameRateEnable");
  global.cameras[camera_serial].isImplementedAcquisitionFrameRateEnable =
      ARV_GC_FEATURE_NODE(pGcNode) ? arv_gc_feature_node_is_implemented(
                                         ARV_GC_FEATURE_NODE(pGcNode), &error)
                                   : FALSE;

  // Find the key name for framerate.
  global.cameras[camera_serial].keyAcquisitionFrameRate = NULL;
  for (int i = 0; i < 2; i++)
  {
    pGcNode =
        arv_device_get_feature(global.cameras[camera_serial].pDevice, pkeyAcquisitionFrameRate[i]);
    global.cameras[camera_serial].isImplementedAcquisitionFrameRate =
        pGcNode ? arv_gc_feature_node_is_implemented(
                      ARV_GC_FEATURE_NODE(pGcNode), &error)
                : FALSE;
    if (global.cameras[camera_serial].isImplementedAcquisitionFrameRate)
    {
      global.cameras[camera_serial].keyAcquisitionFrameRate = pkeyAcquisitionFrameRate[i];
      break;
    }
  }

  // Get parameter bounds.
  arv_camera_get_exposure_time_bounds(global.cameras[camera_serial].pCamera,
                                      &global.cameras[camera_serial].configMin.ExposureTimeAbs,
                                      &global.cameras[camera_serial].configMax.ExposureTimeAbs);
  arv_camera_get_gain_bounds(global.cameras[camera_serial].pCamera, &global.cameras[camera_serial].configMin.Gain,
                             &global.cameras[camera_serial].configMax.Gain);
  arv_camera_get_sensor_size(global.cameras[camera_serial].pCamera, &global.cameras[camera_serial].widthSensor,
                             &global.cameras[camera_serial].heightSensor);
  arv_camera_get_width_bounds(global.cameras[camera_serial].pCamera, &global.cameras[camera_serial].widthRoiMin,
                              &global.cameras[camera_serial].widthRoiMax);
  arv_camera_get_height_bounds(global.cameras[camera_serial].pCamera, &global.cameras[camera_serial].heightRoiMin,
                               &global.cameras[camera_serial].heightRoiMax);

  if (global.cameras[camera_serial].isImplementedFocusPos)
  {
    gint64 focusMin64, focusMax64;
    arv_device_get_integer_feature_bounds(global.cameras[camera_serial].pDevice, "FocusPos",
                                          &focusMin64, &focusMax64);
    global.cameras[camera_serial].configMin.FocusPos = focusMin64;
    global.cameras[camera_serial].configMax.FocusPos = focusMax64;
  }
  else
  {
    global.cameras[camera_serial].configMin.FocusPos = 0;
    global.cameras[camera_serial].configMax.FocusPos = 0;
  }

  global.cameras[camera_serial].configMin.AcquisitionFrameRate = 0.0;
  global.cameras[camera_serial].configMax.AcquisitionFrameRate = 1000.0;


  global.cameras[camera_serial].xRoi = 0;
  global.cameras[camera_serial].yRoi = 0;
  global.cameras[camera_serial].widthRoi = 0;
  global.cameras[camera_serial].heightRoi = 0;
  arv_camera_get_region(global.cameras[camera_serial].pCamera,
                        &global.cameras[camera_serial].xRoi,
                        &global.cameras[camera_serial].yRoi,
                        &global.cameras[camera_serial].widthRoi,
                        &global.cameras[camera_serial].heightRoi);

  global.cameras[camera_serial].config.ExposureTimeAbs =
      global.cameras[camera_serial].isImplementedExposureTimeAbs
                                      ? arv_device_get_float_feature_value(
                                            global.cameras[camera_serial].pDevice,
                                          "ExposureTimeAbs")
                                      : 0;

  global.cameras[camera_serial].config.Gain =
      global.cameras[camera_serial].isImplementedGain ? arv_camera_get_gain(global.cameras[camera_serial].pCamera) : 0.0;
  global.cameras[camera_serial].pszPixelformat =
      g_string_ascii_down(g_string_new(arv_device_get_string_feature_value(
                              global.cameras[camera_serial].pDevice, "PixelFormat")))->str;
  global.cameras[camera_serial].nBytesPixel = ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(
      arv_device_get_integer_feature_value(global.cameras[camera_serial].pDevice, "PixelFormat"));
  global.cameras[camera_serial].config.FocusPos =
      global.cameras[camera_serial].isImplementedFocusPos
          ? arv_device_get_integer_feature_value(global.cameras[camera_serial].pDevice, "FocusPos")
          : 0;


  // Initial camera settings.
  if (global.cameras[camera_serial].isImplementedExposureTimeAbs)
    arv_device_set_float_feature_value(global.cameras[camera_serial].pDevice, "ExposureTimeAbs",
                                       global.cameras[camera_serial].config.ExposureTimeAbs);
  if (global.cameras[camera_serial].isImplementedGain)
    arv_camera_set_gain(global.cameras[camera_serial].pCamera, global.cameras[camera_serial].config.Gain);
  // arv_device_set_integer_feature_value(global.cameras[camera_serial].pDevice, "GainRaw",
  // global.cameras[camera_serial].config.GainRaw);
  if (global.cameras[camera_serial].isImplementedAcquisitionFrameRateEnable)
    arv_device_set_integer_feature_value(global.cameras[camera_serial].pDevice,
                                         "AcquisitionFrameRateEnable", 1);
  if (global.cameras[camera_serial].isImplementedAcquisitionFrameRate)
    arv_device_set_float_feature_value(global.cameras[camera_serial].pDevice,
                                       global.cameras[camera_serial].keyAcquisitionFrameRate,
                                       global.cameras[camera_serial].config.AcquisitionFrameRate);

  // Set up the triggering.
  if (global.cameras[camera_serial].isImplementedTriggerMode)
  {
    if (global.cameras[camera_serial].isImplementedTriggerSelector &&
        global.cameras[camera_serial].isImplementedTriggerMode)
    {
      arv_device_set_string_feature_value(global.cameras[camera_serial].pDevice, "TriggerSelector",
                                          "AcquisitionStart");
      arv_device_set_string_feature_value(global.cameras[camera_serial].pDevice, "TriggerMode",
                                          "Off");
      arv_device_set_string_feature_value(global.cameras[camera_serial].pDevice, "TriggerSelector",
                                          "FrameStart");
      arv_device_set_string_feature_value(global.cameras[camera_serial].pDevice, "TriggerMode",
                                          "Off");
    }
  }
}

void print_genicam_info(const std::string &camera_serial)
{
  // Print information.
  ROS_INFO("    Using Camera Configuration:");
  ROS_INFO("    ---------------------------");
  ROS_INFO("    Vendor name          = %s",
           arv_device_get_string_feature_value(global.cameras[camera_serial].pDevice,
                                               "DeviceVendorName"));
  ROS_INFO(
      "    Model name           = %s",
      arv_device_get_string_feature_value(global.cameras[camera_serial].pDevice, "DeviceModelName"));
  ROS_INFO("    Device id            = %s",
           arv_device_get_string_feature_value(global.cameras[camera_serial].pDevice, "DeviceID"));
  ROS_INFO("    Sensor width         = %d", global.cameras[camera_serial].widthSensor);
  ROS_INFO("    Sensor height        = %d", global.cameras[camera_serial].heightSensor);
  ROS_INFO("    ROI x,y,w,h          = %d, %d, %d, %d", global.cameras[camera_serial].xRoi,
           global.cameras[camera_serial].yRoi, global.cameras[camera_serial].widthRoi, global.cameras[camera_serial].heightRoi);
  ROS_INFO("    Pixel format         = %s", global.cameras[camera_serial].pszPixelformat);
  ROS_INFO("    BytesPerPixel        = %d", global.cameras[camera_serial].nBytesPixel);
  ROS_INFO("    Acquisition Mode     = %s",
           global.cameras[camera_serial].isImplementedAcquisitionMode
               ? arv_device_get_string_feature_value(global.cameras[camera_serial].pDevice,
                                                     "AcquisitionMode")
               : "(not implemented in camera)");
  ROS_INFO(
      "    Trigger Mode         = %s",
      global.cameras[camera_serial].isImplementedTriggerMode
          ? arv_device_get_string_feature_value(global.cameras[camera_serial].pDevice, "TriggerMode")
          : "(not implemented in camera)");
  ROS_INFO("    Trigger Source       = %s",
           global.cameras[camera_serial].isImplementedTriggerSource
               ? arv_device_get_string_feature_value(global.cameras[camera_serial].pDevice,
                                                     "TriggerSource")
               : "(not implemented in camera)");
  ROS_INFO("    Can set FrameRate:     %s",
           global.cameras[camera_serial].isImplementedAcquisitionFrameRate ? "True" : "False");
  if (global.cameras[camera_serial].isImplementedAcquisitionFrameRate)
  {
    global.cameras[camera_serial].config.AcquisitionFrameRate = arv_device_get_float_feature_value(
        global.cameras[camera_serial].pDevice, global.cameras[camera_serial].keyAcquisitionFrameRate);
    ROS_INFO("    AcquisitionFrameRate = %g hz",
             global.cameras[camera_serial].config.AcquisitionFrameRate);
  }

  ROS_INFO("    Can set Exposure:      %s",
           global.cameras[camera_serial].isImplementedExposureTimeAbs ? "True" : "False");
  if (global.cameras[camera_serial].isImplementedExposureTimeAbs)
  {
    ROS_INFO("    Can set ExposureAuto:  %s",
             global.cameras[camera_serial].isImplementedExposureAuto ? "True" : "False");
    ROS_INFO("    Exposure             = %g us in range [%g,%g]",
             global.cameras[camera_serial].config.ExposureTimeAbs, global.cameras[camera_serial].configMin.ExposureTimeAbs,
             global.cameras[camera_serial].configMax.ExposureTimeAbs);
  }

  ROS_INFO("    Can set Gain:          %s",
           global.cameras[camera_serial].isImplementedGain ? "True" : "False");
  if (global.cameras[camera_serial].isImplementedGain)
  {
    ROS_INFO("    Can set GainAuto:      %s",
             global.cameras[camera_serial].isImplementedGainAuto ? "True" : "False");
    ROS_INFO("    Gain                 = %f %% in range [%f,%f]",
             global.cameras[camera_serial].config.Gain, global.cameras[camera_serial].configMin.Gain,
             global.cameras[camera_serial].configMax.Gain);
  }

  ROS_INFO("    Can set FocusPos:      %s",
           global.cameras[camera_serial].isImplementedFocusPos ? "True" : "False");

  if (global.cameras[camera_serial].isImplementedMtu)
    ROS_INFO("    Network mtu          = %lu",
             (long int)arv_device_get_integer_feature_value(
                 global.cameras[camera_serial].pDevice, "GevSCPSPacketSize"));

  ROS_INFO("    ---------------------------");

}

int main(int argc, char** argv)
{
  ApplicationData applicationdata;
  int nInterfaces = 0;
  int nDevices = 0;
  int i = 0;

  global.bCancel = FALSE;
  global.idSoftwareTriggerTimer = 0;

  ros::init(argc, argv, "camera_aravis_node");

  global.phNode = std::make_shared<ros::NodeHandle>("~");

  // declaring SW trigger service
  ros::ServiceServer trigger_service = global.phNode->advertiseService(
      "trigger_acquisition", SoftwareTrigger_callback);

#if !GLIB_CHECK_VERSION(2, 35, 0)
  g_type_init();
#endif

  //prepare camera map
  global.cameras.reserve(10);
  std::unordered_map<std::string, std::string> available_cameras_serial;
  std::vector<std::string> requested_cameras_serial;

  // Print out some useful info.
  ROS_INFO("Attached cameras:");
  arv_update_device_list();
  nInterfaces = arv_get_n_interfaces();
  ROS_INFO("# Interfaces: %d", nInterfaces);

  nDevices = arv_get_n_devices();
  ROS_INFO("# Devices: %d", nDevices);
  for (i = 0; i < nDevices; i++)
  {
    std::string device_ID = arv_get_device_id(i);
    ROS_INFO("Device%d: %s", i, device_ID.c_str());
    size_t position = device_ID.rfind('-');
    if (position!=std::string::npos && device_ID.size()>(position+1))
    {
      available_cameras_serial[device_ID.substr(position+1)] = device_ID;
    } else
    {
      available_cameras_serial[device_ID] = device_ID;
    }

  }

  if (nDevices > 0)
  {
    // Get the camera guids as a parameter.
    if (argc > 2)
    {
      for(int i = 1; i < argc; i++)
      {
        requested_cameras_serial.emplace_back(argv[i]);
      }
    }
    else
    {
      if (global.phNode->hasParam(ros::this_node::getName() + "/camera_serials"))
      {
        global.phNode->getParam(ros::this_node::getName() + "/camera_serials",
                                requested_cameras_serial);
      }
    }

    if (requested_cameras_serial.size()==0)
    {
      for(auto &camera_serial : available_cameras_serial)
      {
        requested_cameras_serial.push_back(camera_serial.first);
      }
    }

    // create camera instance for every requested camera
    for(auto &requested_camera_serial : requested_cameras_serial)
    {
      global.cameras[requested_camera_serial] = GeniCam();
    }

    for (const auto &camera_serial : available_cameras_serial )
    {
      if(std::find(requested_cameras_serial.begin(),
                   requested_cameras_serial.end(),
                   camera_serial.first) != requested_cameras_serial.end())
      {
        // Open the camera, and set it up.
        ROS_INFO("Opening: %s", camera_serial.first.c_str());

        try
        {
          global.cameras[camera_serial.first].pCamera = arv_camera_new(camera_serial.second.c_str());
          if (!global.cameras[camera_serial.first].pCamera)
          {
            throw std::runtime_error(("camera with ID: "+ camera_serial.first +" could not be opened. skip."));
          }

          global.cameras[camera_serial.first].pDevice = arv_camera_get_device(global.cameras[camera_serial.first].pCamera);

          init_genicam_parameter(camera_serial.first);
          print_genicam_info(camera_serial.first);

          std::string name = ros::this_node::getName()+"/"+camera_serial.first;
          global.cameras[camera_serial.first].pCameraInfoManager= std::make_shared<camera_info_manager::CameraInfoManager>(
                                                                             ros::NodeHandle(name),
                                                                             camera_serial.first);

          ROS_INFO("Opened: %s-%s", arv_device_get_string_feature_value(
                                    global.cameras[camera_serial.first].pDevice, "DeviceVendorName"),
                 arv_device_get_string_feature_value(global.cameras[camera_serial.first].pDevice, "DeviceID"));

          global.cameras[camera_serial.first].pStream = NULL;

          global.cameras[camera_serial.first].pStream = CreateStream(camera_serial.first);
          if (!global.cameras[camera_serial.first].pStream)
          {
            throw std::runtime_error(("stream for camera with ID: "+ camera_serial.first +" could not be opened. skip."));
          }

          // Set up image_raw.
          global.cameras[camera_serial.first].pTransport = std::make_shared<image_transport::ImageTransport>(*global.phNode);
          global.cameras[camera_serial.first].publisher = global.cameras[camera_serial.first].pTransport->advertiseCamera(
              ros::this_node::getName() + "/" + camera_serial.first + "/image_raw", 1);

          // Connect signals with callbacks.
          g_signal_connect(global.cameras[camera_serial.first].pStream,
                           "new-buffer", G_CALLBACK(NewBuffer_callback),
                           &global.cameras[camera_serial.first]);
          g_signal_connect(global.cameras[camera_serial.first].pDevice, "control-lost",
                           G_CALLBACK(ControlLost_callback), NULL);

          arv_stream_set_emit_signals(
                (ArvStream*)global.cameras[camera_serial.first].pStream, TRUE);

          arv_device_execute_command(global.cameras[camera_serial.first].pDevice,
                                     "AcquisitionStart");

          global.cameras[camera_serial.first].isRunning = true;
        } catch (const std::exception& e)
        {
          ROS_WARN(e.what());
          global.cameras[camera_serial.first].isRunning = false;
          continue;
        }
      }
    }



    //WriteCameraFeaturesFromRosparam();

    /*
#ifdef TUNING
    ros::Publisher pubInt64 = global.phNode->advertise<std_msgs::Int64>(
        ros::this_node::getName() + "/dt", 100);
    global.ppubInt64 = &pubInt64;
#endif
*/
    // Start the camerainfo manager.
    /*std::string camera_info_url;
    if (!pnh.getParam("url", camera_info_url))
    {
      ROS_ERROR(
          "ros parameter url not set in camnode where's the camera_info file?");
    }
    global.pCameraInfoManager = new camera_info_manager::CameraInfoManager(
        ros::NodeHandle(ros::this_node::getName()),
        arv_device_get_string_feature_value(global.pDevice, "DeviceID"),
        camera_info_url);
     */
    // Start the dynamic_reconfigure server.
    //dynamic_reconfigure::Server<Config> reconfigureServer;
    //dynamic_reconfigure::Server<Config>::CallbackType reconfigureCallback;

    //reconfigureCallback = boost::bind(&RosReconfigure_callback, _1, _2);
    //reconfigureServer.setCallback(reconfigureCallback);
    //ros::Duration(2.0).sleep();

    // Get parameter current values.


    //		// Print the tree of camera features, with their values.
    //		ROS_INFO ("
    //----------------------------------------------------------------------------------");
    //		NODEEX		 nodeex;
    //		ArvGc	*pGenicam=0;
    //		pGenicam = arv_device_get_genicam(global.pDevice);
    //
    //		nodeex.szName = "Root";
    //		nodeex.pNode = (ArvDomNode	*)arv_gc_get_node(pGenicam,
    //nodeex.szName);
    //		nodeex.pNodeSibling = NULL;
    //		PrintDOMTree(pGenicam, nodeex, 0);
    //		ROS_INFO ("
    //----------------------------------------------------------------------------------");

    applicationdata.main_loop = 0;

    g_timeout_add_seconds(0.1, PeriodicTask_callback, &applicationdata);

    void (*pSigintHandlerOld)(int);
    pSigintHandlerOld = signal(SIGINT, set_cancel);

    applicationdata.main_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(applicationdata.main_loop);

    if (global.idSoftwareTriggerTimer)
    {
      g_source_remove(global.idSoftwareTriggerTimer);
      global.idSoftwareTriggerTimer = 0;
    }

    signal(SIGINT, pSigintHandlerOld);

    std::cout<<"Test six"<<std::endl;
    g_main_loop_unref(applicationdata.main_loop);
    std::cout<<"Test two"<<std::endl;
    for(auto &camera : global.cameras)
    {
      std::cout<<"Test three"<<std::endl;
      if(camera.second.isRunning == true)
      {
        std::cout<<"Test four"<<std::endl;
        guint64 n_completed_buffers;
        guint64 n_failures;
        guint64 n_underruns;
        guint64 n_resent;
        guint64 n_missing;
        arv_stream_get_statistics((ArvStream*)camera.second.pStream, &n_completed_buffers,
                                  &n_failures, &n_underruns);
        ROS_INFO("Completed buffers = %Lu",
                 (unsigned long long)n_completed_buffers);
        ROS_INFO("Failures          = %Lu", (unsigned long long)n_failures);
        ROS_INFO("Underruns         = %Lu", (unsigned long long)n_underruns);
        arv_gv_stream_get_statistics(camera.second.pStream, &n_resent, &n_missing);
        ROS_INFO("Resent buffers    = %Lu", (unsigned long long)n_resent);
        ROS_INFO("Missing           = %Lu", (unsigned long long)n_missing);

        arv_device_execute_command(camera.second.pDevice, "AcquisitionStop");

        camera.second.publisher.shutdown();

        g_object_unref(camera.second.pStream);
        g_object_unref(camera.second.pCamera);
      }
    }
  }
  else
    ROS_ERROR("No cameras detected.");

  global.cameras.clear();
  ros::shutdown();
  while(ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
} // main()
