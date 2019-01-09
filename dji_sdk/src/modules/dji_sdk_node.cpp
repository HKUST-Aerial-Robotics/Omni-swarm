/** @file dji_sdk_node.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the initialization functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

using namespace DJI::OSDK;

DJISDKNode::DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : telemetry_from_fc(USE_BROADCAST)
  , R_FLU2FRD(tf::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1))
  , R_ENU2NED(tf::Matrix3x3(0, 1, 0, 1, 0, 0, 0, 0, -1))
  , curr_align_state(UNALIGNED)
{
  nh_private.param("serial_name", serial_device, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate", baud_rate, 921600);
  nh_private.param("app_id", app_id, 123456);
  nh_private.param("app_version", app_version, 1);
  nh_private.param("enc_key", enc_key, std::string("abcd1234"));
  nh_private.param("drone_version", drone_version,
                   std::string("M100")); // choose M100 as default
  nh_private.param("gravity_const", gravity_const, 9.801);
  nh_private.param("align_time", align_time_with_FC, true);
  nh_private.param("use_broadcast", user_select_BC, false);
  nh_private.param("no_RTK", user_no_RTK, false);
  nh_private.param("broadcast_data", user_less_broadcast_data, !user_select_BC);

  //! Default values for local Position
  local_pos_ref_latitude = local_pos_ref_longitude = local_pos_ref_altitude = 0;
  local_pos_ref_set = false;
  //! RTK support check
  rtkSupport = false;

  // @todo need some error handling for init functions
  //! @note parsing launch file to get environment parameters
  if (!initVehicle(nh_private))
  {
    ROS_ERROR("Vehicle initialization failed");
  }

  else
  {

    if (!initFlightControl(nh_private))
    {
      ROS_ERROR("initFlightControl failed");
    }

    if (!initSubscriber(nh_private))
    {
      ROS_ERROR("initSubscriber failed");
    }

    if (!initPublisher(nh_private))
    {
      ROS_ERROR("initPublisher failed");
    }

    if (!initServices(nh_private))
    {
      ROS_ERROR("initServices failed");
    }
  }
  actions.init(vehicle);
}

DJISDKNode::~DJISDKNode()
{
  if (!isM100())
  {
    vehicle->hardSync->setSyncFreq(0);
    cleanUpSubscribeFromFC();
  }
  if (vehicle)
  {
    delete vehicle;
  }
}

bool
DJISDKNode::initVehicle(ros::NodeHandle& nh_private)
{
  bool threadSupport           = true;
  bool enable_advanced_sensing = false;

#ifdef ADVANCED_SENSING
  enable_advanced_sensing = true;
  ROS_INFO("Advanced Sensing is Enabled on M210.");
#endif

  LinuxSerialDevice* linuxSerialDevice =
    new LinuxSerialDevice(serial_device.c_str(), baud_rate);
  linuxSerialDevice->init();
  bool setupStatus = validateSerialDevice(linuxSerialDevice);
  if (!setupStatus)
  {
    delete (linuxSerialDevice);
    return false;
  }
  else
  {
    delete (linuxSerialDevice);
  }

  //! @note currently does not work without thread support
  vehicle = new Vehicle(serial_device.c_str(), baud_rate, threadSupport,
                        enable_advanced_sensing);

  /*!
   * @note activate the drone for the user at the beginning
   *        user can also call it as a service
   *        this has been tested by giving wrong appID in launch file
   */
  if (ACK::getError(this->activate(this->app_id, this->enc_key)))
  {
    ROS_ERROR("drone activation error");
    return false;
  }
  ROS_INFO("drone activated");

  // This version of ROS Node works for:
  //    1. A3/N3/M600 with latest FW
  //    2. M100 with FW version M100_31
  if (vehicle->getFwVersion() > INVALID_VERSION &&
      vehicle->getFwVersion() < mandatoryVersionBase && (!isM100()))
  {
    return false;
  }

  if (NULL != vehicle->subscribe && (!user_select_BC))
  {
    telemetry_from_fc = USE_SUBSCRIBE;
  }

  return true;
}

// clang-format off
bool DJISDKNode::initServices(ros::NodeHandle& nh) {
  // Common to A3/N3 and M100
  drone_activation_server   = nh.advertiseService("dji_sdk/activation",                     &DJISDKNode::droneActivationCallback,        this);
  drone_arm_server          = nh.advertiseService("dji_sdk/drone_arm_control",              &DJISDKNode::droneArmCallback,               this);
  drone_task_server         = nh.advertiseService("dji_sdk/drone_task_control",             &DJISDKNode::droneTaskCallback,              this);
  sdk_ctrlAuthority_server  = nh.advertiseService("dji_sdk/sdk_control_authority",          &DJISDKNode::sdkCtrlAuthorityCallback,       this);
  camera_action_server      = nh.advertiseService("dji_sdk/camera_action",                  &DJISDKNode::cameraActionCallback,           this);
  waypoint_upload_server    = nh.advertiseService("dji_sdk/mission_waypoint_upload",        &DJISDKNode::missionWpUploadCallback,        this);
  waypoint_action_server    = nh.advertiseService("dji_sdk/mission_waypoint_action",        &DJISDKNode::missionWpActionCallback,        this);
  waypoint_getInfo_server   = nh.advertiseService("dji_sdk/mission_waypoint_getInfo",       &DJISDKNode::missionWpGetInfoCallback,       this);
  waypoint_getSpeed_server  = nh.advertiseService("dji_sdk/mission_waypoint_getSpeed",      &DJISDKNode::missionWpGetSpeedCallback,      this);
  waypoint_setSpeed_server  = nh.advertiseService("dji_sdk/mission_waypoint_setSpeed",      &DJISDKNode::missionWpSetSpeedCallback,      this);
  hotpoint_upload_server    = nh.advertiseService("dji_sdk/mission_hotpoint_upload",        &DJISDKNode::missionHpUploadCallback,        this);
  hotpoint_action_server    = nh.advertiseService("dji_sdk/mission_hotpoint_action",        &DJISDKNode::missionHpActionCallback,        this);
  hotpoint_getInfo_server   = nh.advertiseService("dji_sdk/mission_hotpoint_getInfo",       &DJISDKNode::missionHpGetInfoCallback,       this);
  hotpoint_setSpeed_server  = nh.advertiseService("dji_sdk/mission_hotpoint_updateYawRate", &DJISDKNode::missionHpUpdateYawRateCallback, this);
  hotpoint_resetYaw_server  = nh.advertiseService("dji_sdk/mission_hotpoint_resetYaw",      &DJISDKNode::missionHpResetYawCallback,      this);
  hotpoint_setRadius_server = nh.advertiseService("dji_sdk/mission_hotpoint_updateRadius",  &DJISDKNode::missionHpUpdateRadiusCallback,  this);
  mission_status_server     = nh.advertiseService("dji_sdk/mission_status",                 &DJISDKNode::missionStatusCallback,          this);
  send_to_mobile_server     = nh.advertiseService("dji_sdk/send_data_to_mobile",            &DJISDKNode::sendToMobileCallback,           this);
  query_version_server      = nh.advertiseService("dji_sdk/query_drone_version",            &DJISDKNode::queryVersionCallback,           this);
  local_pos_ref_server      = nh.advertiseService("dji_sdk/set_local_pos_ref",              &DJISDKNode::setLocalPosRefCallback,         this);


  // A3/N3 only
  if(!isM100())
  {
#ifdef ADVANCED_SENSING
    subscribe_stereo_240p_server  = nh.advertiseService("dji_sdk/stereo_240p_subscription",   &DJISDKNode::stereo240pSubscriptionCallback, this);
    subscribe_stereo_depth_server = nh.advertiseService("dji_sdk/stereo_depth_subscription",  &DJISDKNode::stereoDepthSubscriptionCallback,this);
    subscribe_stereo_vga_server   = nh.advertiseService("dji_sdk/stereo_vga_subscription",    &DJISDKNode::stereoVGASubscriptionCallback,  this);
    camera_stream_server          = nh.advertiseService("dji_sdk/setup_camera_stream",        &DJISDKNode::setupCameraStreamCallback,      this);
#endif
    set_hardsync_server   = nh.advertiseService("dji_sdk/set_hardsyc",    &DJISDKNode::setHardsyncCallback,   this);
    mfio_config_server    = nh.advertiseService("dji_sdk/mfio_config",    &DJISDKNode::MFIOConfigCallback,    this);
    mfio_set_value_server = nh.advertiseService("dji_sdk/mfio_set_value", &DJISDKNode::MFIOSetValueCallback,  this);
  }
  return true;
}
// clang-format on

bool
DJISDKNode::initFlightControl(ros::NodeHandle& nh)
{
  flight_control_sub = nh.subscribe<sensor_msgs::Joy>(
    "dji_sdk/flight_control_setpoint_generic", 10,
    &DJISDKNode::flightControlSetpointCallback, this);

  flight_control_position_yaw_sub = nh.subscribe<sensor_msgs::Joy>(
    "dji_sdk/flight_control_setpoint_ENUposition_yaw", 10,
    &DJISDKNode::flightControlPxPyPzYawCallback, this);

  flight_control_velocity_yawrate_sub = nh.subscribe<sensor_msgs::Joy>(
    "dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10,
    &DJISDKNode::flightControlVxVyVzYawrateCallback, this);

  flight_control_rollpitch_yawrate_vertpos_sub = nh.subscribe<sensor_msgs::Joy>(
    "dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10,
    &DJISDKNode::flightControlRollPitchPzYawrateCallback, this);

  return true;
}

bool
DJISDKNode::isM100()
{
  return (vehicle->isM100());
}

ACK::ErrorCode
DJISDKNode::activate(int l_app_id, std::string l_enc_key)
{
  usleep(1000000);
  Vehicle::ActivateData testActivateData;
  char                  app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;

  ROS_DEBUG("called vehicle->activate(&testActivateData, WAIT_TIMEOUT)");
  return vehicle->activate(&testActivateData, WAIT_TIMEOUT);
}

bool
DJISDKNode::initSubscriber(ros::NodeHandle& nh)
{
  gimbal_angle_cmd_subscriber = nh.subscribe<dji_sdk::Gimbal>(
    "dji_sdk/gimbal_angle_cmd", 10, &DJISDKNode::gimbalAngleCtrlCallback, this);
  gimbal_speed_cmd_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>(
    "dji_sdk/gimbal_speed_cmd", 10, &DJISDKNode::gimbalSpeedCtrlCallback, this);
  return true;
}

bool
DJISDKNode::initDataSubscribeFromFC()
{
  ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  // 100 Hz package from FC
  Telemetry::TopicName topicList100Hz[] = {
    Telemetry::TOPIC_QUATERNION, Telemetry::TOPIC_ACCELERATION_GROUND,
    Telemetry::TOPIC_ANGULAR_RATE_FUSIONED
  };
  int nTopic100Hz = sizeof(topicList100Hz) / sizeof(topicList100Hz[0]);
  if (vehicle->subscribe->initPackageFromTopicList(
        PACKAGE_ID_100HZ, nTopic100Hz, topicList100Hz, 1, 100))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 100Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
        PACKAGE_ID_100HZ, publish100HzData, this);
    }
  }

  // 50 Hz package from FC
  Telemetry::TopicName topicList50Hz[] = {
    Telemetry::TOPIC_GPS_FUSED,          Telemetry::TOPIC_ALTITUDE_FUSIONED,
    Telemetry::TOPIC_HEIGHT_FUSION,      Telemetry::TOPIC_STATUS_FLIGHT,
    Telemetry::TOPIC_STATUS_DISPLAYMODE, Telemetry::TOPIC_GIMBAL_ANGLES,
    Telemetry::TOPIC_GIMBAL_STATUS,      Telemetry::TOPIC_RC,
    Telemetry::TOPIC_VELOCITY,           Telemetry::TOPIC_GPS_CONTROL_LEVEL,
    // Telemetry::TOPIC_RC_WITH_FLAG_DATA
  };
  int nTopic50Hz = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_50HZ, nTopic50Hz,
                                                   topicList50Hz, 1, 50))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 50Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
        PACKAGE_ID_50HZ, publish50HzData, (UserData) this);
    }
  }

  //! Check if RTK is supported in the FC

  if (!user_no_RTK)
  {
    Telemetry::TopicName topicRTKSupport[] = { Telemetry::TOPIC_RTK_POSITION };

    int nTopicRTKSupport = sizeof(topicRTKSupport) / sizeof(topicRTKSupport[0]);
    if (vehicle->subscribe->initPackageFromTopicList(
          PACKAGE_ID_10HZ, nTopicRTKSupport, topicRTKSupport, 1, 10))
    {
      ack = vehicle->subscribe->startPackage(PACKAGE_ID_10HZ, WAIT_TIMEOUT);
      if (ack.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE)
      {
        rtkSupport = false;
        ROS_INFO("Flight Controller does not support RTK");
      }
      else
      {
        rtkSupport = true;
        vehicle->subscribe->removePackage(PACKAGE_ID_10HZ, WAIT_TIMEOUT);
      }
    }
  }
  else
  {
    rtkSupport = false;
  }

  if (!rtkSupport)
  {
    // 10 Hz package from FC
    Telemetry::TopicName topicList10Hz[] = {
      Telemetry::TOPIC_GPS_DATE,      Telemetry::TOPIC_GPS_TIME,
      Telemetry::TOPIC_GPS_POSITION,  Telemetry::TOPIC_GPS_VELOCITY,
      Telemetry::TOPIC_GPS_DETAILS,   Telemetry::TOPIC_BATTERY_INFO,
      Telemetry::TOPIC_CONTROL_DEVICE
    };
    bool topicStartSuccess =
      topic10hzStart(topicList10Hz, sizeof(topicList10Hz));
    if (topicStartSuccess == false)
    {
      return false;
    }
  }
  else
  {
    Telemetry::TopicName topicList10Hz[] = {

      Telemetry::TOPIC_GPS_DATE,          Telemetry::TOPIC_GPS_TIME,
      Telemetry::TOPIC_GPS_POSITION,      Telemetry::TOPIC_GPS_VELOCITY,
      Telemetry::TOPIC_GPS_DETAILS,       Telemetry::TOPIC_BATTERY_INFO,
      Telemetry::TOPIC_RTK_POSITION,      Telemetry::TOPIC_RTK_VELOCITY,
      Telemetry::TOPIC_RTK_YAW,           Telemetry::TOPIC_RTK_YAW_INFO,
      Telemetry::TOPIC_RTK_POSITION_INFO, Telemetry::TOPIC_CONTROL_DEVICE
    };
    bool packageStart = topic10hzStart(topicList10Hz, sizeof(topicList10Hz));
    if (packageStart == false)
    {
      return false;
    }
  }

  // 400 Hz data from FC
  Telemetry::TopicName topicList400Hz[] = { Telemetry::TOPIC_HARD_SYNC };
  int nTopic400Hz = sizeof(topicList400Hz) / sizeof(topicList400Hz[0]);
  if (vehicle->subscribe->initPackageFromTopicList(
        PACKAGE_ID_400HZ, nTopic400Hz, topicList400Hz, 1, 400))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 400Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
        PACKAGE_ID_400HZ, publish400HzData, this);
    }
  }

  ros::Duration(1).sleep();
  return true;
}

bool
DJISDKNode::topic10hzStart(Telemetry::TopicName topicList10Hz[],
                           int                  sizeOfArray)
{
  int nTopic10Hz = sizeOfArray / sizeof(topicList10Hz[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_10HZ, nTopic10Hz,
                                                   topicList10Hz, 1, 10))
  {
    ACK::ErrorCode ack =
      vehicle->subscribe->startPackage(PACKAGE_ID_10HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_10HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 10Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
        PACKAGE_ID_10HZ, publish10HzData, this);
      return true;
    }
  }
}

void
DJISDKNode::cleanUpSubscribeFromFC()
{
  vehicle->subscribe->removePackage(0, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(1, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(2, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(3, WAIT_TIMEOUT);
}

bool
DJISDKNode::validateSerialDevice(LinuxSerialDevice* serialDevice)
{
  static const int BUFFER_SIZE = 2048;
  //! Check the serial channel for data
  uint8_t buf[BUFFER_SIZE];
  if (!serialDevice->setSerialPureTimedRead())
  {
    ROS_ERROR("Failed to set up port for timed read.\n");
    return (false);
  };
  usleep(100000);
  if (serialDevice->serialRead(buf, BUFFER_SIZE))
  {
    ROS_INFO("Succeeded to read from serial device");
  }
  else
  {
    ROS_ERROR("Failed to read from serial device. The Onboard SDK is not "
              "communicating with your drone.");
    return (false);
  }

  // All the tests passed and the serial device is properly set up
  serialDevice->unsetSerialPureTimedRead();
  return (true);
}

void
DJISDKNode::setUpM100DefaultFreq(uint8_t freq[16])
{
  freq[0]  = DataBroadcast::FREQ_100HZ;
  freq[1]  = DataBroadcast::FREQ_100HZ;
  freq[2]  = DataBroadcast::FREQ_100HZ;
  freq[3]  = DataBroadcast::FREQ_50HZ;
  freq[4]  = DataBroadcast::FREQ_100HZ;
  freq[5]  = DataBroadcast::FREQ_50HZ;
  freq[6]  = DataBroadcast::FREQ_10HZ;
  freq[7]  = DataBroadcast::FREQ_50HZ;
  freq[8]  = DataBroadcast::FREQ_50HZ;
  freq[9]  = DataBroadcast::FREQ_50HZ;
  freq[10] = DataBroadcast::FREQ_10HZ;
  freq[11] = DataBroadcast::FREQ_10HZ;
}

void
DJISDKNode::setUpA3N3DefaultFreq(uint8_t freq[16])
{
  if (user_less_broadcast_data)
  {
    freq[0]  = DataBroadcast::FREQ_1HZ;  // time
    freq[1]  = DataBroadcast::FREQ_0HZ;  // q
    freq[2]  = DataBroadcast::FREQ_0HZ;  // a
    freq[3]  = DataBroadcast::FREQ_0HZ;  // v
    freq[4]  = DataBroadcast::FREQ_0HZ;  // w
    freq[5]  = DataBroadcast::FREQ_0HZ;  // pos
    freq[6]  = DataBroadcast::FREQ_0HZ;  // gps
    freq[7]  = DataBroadcast::FREQ_0HZ;  // rtk
    freq[8]  = DataBroadcast::FREQ_0HZ;  // mag
    freq[9]  = DataBroadcast::FREQ_50HZ; // rc
    freq[10] = DataBroadcast::FREQ_0HZ;  // gimbal
    freq[11] = DataBroadcast::FREQ_0HZ;  // status
    freq[12] = DataBroadcast::FREQ_0HZ;  // battery
    freq[13] = DataBroadcast::FREQ_0HZ;  // device
  }
  else
  {
    //! @note both Broadcast and subscription high payload would let data stream
    //! crash into a mis align status
    freq[0]  = DataBroadcast::FREQ_100HZ; // time
    freq[1]  = DataBroadcast::FREQ_100HZ; // q
    freq[2]  = DataBroadcast::FREQ_100HZ; // a
    freq[3]  = DataBroadcast::FREQ_50HZ;  // v
    freq[4]  = DataBroadcast::FREQ_100HZ; // w
    freq[5]  = DataBroadcast::FREQ_50HZ;  // pos
    freq[6]  = DataBroadcast::FREQ_50HZ;  // gps
    freq[7]  = DataBroadcast::FREQ_50HZ;  // rtk
    freq[8]  = DataBroadcast::FREQ_10HZ;  // mag
    freq[9]  = DataBroadcast::FREQ_50HZ;  // rc
    freq[10] = DataBroadcast::FREQ_50HZ;  // gimbal
    freq[11] = DataBroadcast::FREQ_50HZ;  // status
    freq[12] = DataBroadcast::FREQ_10HZ;  // battery
    freq[13] = DataBroadcast::FREQ_10HZ;  // device
  }
}

void
DJISDKNode::gpsConvertENU(double& ENU_x, double& ENU_y, double gps_t_lon,
                          double gps_t_lat, double gps_r_lon, double gps_r_lat)
{
  double d_lon = gps_t_lon - gps_r_lon;
  double d_lat = gps_t_lat - gps_r_lat;
  ENU_y        = DEG2RAD(d_lat) * C_EARTH;
  ENU_x        = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}
