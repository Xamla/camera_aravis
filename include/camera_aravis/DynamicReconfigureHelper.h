#ifndef DYNAMICRECONFIGUREHELPER_H
#define DYNAMICRECONFIGUREHELPER_H

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

#endif // DYNAMICRECONFIGUREHELPER_H
