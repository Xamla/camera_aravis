#ifndef GENICAM_H
#define GENICAM_H

#include <unordered_map>

#include <ros/ros.h>
#include <glib.h>
#include <arv.h>

#include <std_msgs/Int64.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

enum State{CREATED,LOST_CONNECTION,READY,STREAMING};

struct Config
{
  Config()
  {
    Acquire = true;
    ExposureAuto = "Off";
    GainAuto = "Off";
    ExposureTimeAbs = 2000.0;
    Gain = 1.0;
    AcquisitionMode = "Continous";
    AcquisitionFrameRate = 20.0;
    TriggerMode = "Off";
    TriggerSource = "Line1";
    FocusPos = 32767;
    frame_id = "camera";
    mtu = 576;
  }

  bool Acquire;
  std::string ExposureAuto;
  std::string GainAuto;
  double ExposureTimeAbs;
  double Gain;
  std::string AcquisitionMode;
  double AcquisitionFrameRate;
  std::string TriggerMode;
  std::string TriggerSource;
  int FocusPos;
  std::string frame_id;
  int mtu;
};

struct GeniCam
{
  GeniCam() = default;
  GeniCam(const GeniCam&) = default;

  ~GeniCam()
  {
    arv_device_execute_command(pDevice,
                               "AcquisitionStop");

    publisher.shutdown();
    g_object_unref(pStream);
    g_object_unref(pCamera);
  }


  State state;
  bool isNewImage;

  std::shared_ptr<image_transport::ImageTransport> pTransport;
  image_transport::CameraPublisher publisher;
  std::shared_ptr<camera_info_manager::CameraInfoManager> pCameraInfoManager;
  sensor_msgs::CameraInfo camerainfo;
  sensor_msgs::Image imageMsg;

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

  // time compensation related things
  uint64_t cm = 0L; // Camera time prev
  uint64_t tm = 0L; // Calculated image time prev
  int64_t em = 0L; // Error prev.

#ifdef TUNING
  std::unique_ptr<ros::Publisher> ppubInt64;
#endif

};

#endif // GENICAM_H
