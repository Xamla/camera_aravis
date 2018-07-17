#ifndef GENICAM_H
#define GENICAM_H

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <glib.h>

#include <std_msgs/Int64.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "camera_aravis/genicamfeatures.h"

enum CameraState{NOTINITIALIZED, READY, STREAMING, CAPTURING};

class GeniCam
{
public:
  GeniCam(std::shared_ptr<ros::NodeHandle> &phNode, const std::pair<const std::string, std::string> &serial_deviceid);

  ~GeniCam();

  static void newbuffer_callback(ArvStream* pStream,
                                 GeniCam* pCameradata);

  static void connectionlost_callback(ArvGvDevice* pGvDevice,
                                      GeniCam* pCameradata);

  bool reestablishConnection(std::shared_ptr<ros::NodeHandle> &phNode);

  enum CameraState getCameraState();

  void showStatistic();

  bool capture(std::vector<sensor_msgs::Image>& imageContainer);

  bool tryToSetFeatureValue(const std::string& feature, const std::string& value);
  bool tryToGetFeatureValue(const std::string& feature, std::string &value);

protected:

  //-----methods-----
  ArvGvStream* createStream(const std::string &camera_serial);

  void processNewBuffer(ArvStream* pStream);
  void handleConnectionLoss();

  void connectCallback();
  void disconnectCallback();

  void setStreamingConfiguration();
  void restoreConfiguration(const bool& wasStreaming=false);
  void setCaptureConfiguration(const bool& startCapture=false);

  //----attributes---
  std::string serialNumber;
  std::string deviceID;
  std::atomic<CameraState> cameraState;

  //conditional variable and mutex to wait for new image
  std::condition_variable newImageAvailable;
  std::mutex imageWaitMutex;
  std::mutex aquisitionChangeMutex;

  //ros specific
  std::shared_ptr<image_transport::ImageTransport> pImageTransport;
  image_transport::CameraPublisher publisher;
  std::shared_ptr<camera_info_manager::CameraInfoManager> pCameraInfoManager;
  sensor_msgs::CameraInfo camerainfo;
  sensor_msgs::Image imageMsg;

  // aravis g_objects
  ArvCamera* pCamera;
  ArvDevice* pDevice;
  ArvGvStream* pStream;

  gulong newbufferHandlerID;
  gulong connectionLostHandlerID;

  // genicam feaures
  genicam_features::GenicamFeatures genicamFeatures;

  // streaming genicam features
  std::string frame_id;
  int xRoi;
  int yRoi;
  int widthRoi;
  int heightRoi;

  const char* pszPixelformat;
  unsigned nBytesPixel;

  size_t sequenceCounter;
  size_t nBuffers; // Counter for Hz calculation.

  // time compensation related things
  uint64_t cm;// Camera time prev
  uint64_t tm;// Calculated image time prev
  int64_t em;// Error prev.
};

#endif // GENICAM_H
