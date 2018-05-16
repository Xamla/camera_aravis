#ifndef GENICAM_H
#define GENICAM_H

#include <glib.h>

#include <std_msgs/Int64.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "camera_aravis/genicamfeatures.h"

#define ARV_PIXEL_FORMAT_BIT_PER_PIXEL(pixel_format)                           \
  (((pixel_format) >> 16) & 0xff)
#define ARV_PIXEL_FORMAT_BYTE_PER_PIXEL(pixel_format)                          \
  ((((pixel_format) >> 16) & 0xff) >> 3)

enum State{CREATED,LOST_CONNECTION,READY,STREAMING};

struct GeniCam
{
  GeniCam() = default;
  GeniCam(const GeniCam&) = default;

  ~GeniCam()
  {
    if(state == State::READY ||
       state == State::STREAMING)
    {
      arv_device_execute_command(pDevice,
                                 "AcquisitionStop");

      publisher.shutdown();
      g_object_unref(pStream);
      g_object_unref(pCamera);
    }
  }

  bool init(const std::string& serial_number)
  {
    name = "camera_"+serial_number;

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

    state = State::READY;

    return true;
  }

  State state = State::CREATED;
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

  // genicam feaures
  genicam_features::GenicamFeatures genicamFeatures;

  // streaming genicam features
  std::string name;
  int xRoi;
  int yRoi;
  int widthRoi;
  int heightRoi;

  const char* pszPixelformat;
  unsigned nBytesPixel;

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
