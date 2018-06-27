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

#include <iostream>
#include <memory>
#include <string>
#include <algorithm>
#include <functional>

#include <stdlib.h>
#include <math.h>

#include <ros/time.h>
#include <ros/duration.h>

#include <sensor_msgs/image_encodings.h>

#include "camera_aravis/cameramanager.h"

// Global variables -------------------

struct Global
{
  GMainLoop* main_loop;
  gboolean bCancel;

} global;

// ------------------------------------

// Conversions from integers to Arv types.

static void set_cancel(int signal) { global.bCancel = TRUE; }


// PeriodicTask_callback()
// Check for termination, and spin for ROS.
static gboolean PeriodicTask_callback(void* cameraManager)
{
  CameraManager *pCameraManager = (CameraManager*) cameraManager;
  if (global.bCancel || ros::isShuttingDown())
  {
    g_main_loop_quit(global.main_loop);
    return FALSE;
  }

  ros::spinOnce();

  return TRUE;
} // PeriodicTask_callback()

int main(int argc, char** argv)
{
  global.bCancel = FALSE;

  ros::init(argc, argv, "camera_aravis_node");
  std::shared_ptr<ros::NodeHandle> phNode = std::make_shared<ros::NodeHandle>("~");

#if !GLIB_CHECK_VERSION(2, 35, 0)
  g_type_init();
#endif

  CameraManager cameraManager(phNode);

  g_timeout_add(20, PeriodicTask_callback, &cameraManager);

  void (*pSigintHandlerOld)(int);
  pSigintHandlerOld = signal(SIGINT, set_cancel);

  global.main_loop = 0;
  global.main_loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(global.main_loop);

  signal(SIGINT, pSigintHandlerOld);

  g_main_destroy(global.main_loop);

  ros::shutdown();
  arv_shutdown();

  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
} // main()
