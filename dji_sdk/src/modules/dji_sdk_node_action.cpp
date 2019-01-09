#include <dji_sdk/dji_sdk_node_action.hpp>

using namespace DJISDK;

Actions::Actions()
  : nh("~")
  , authorityServerName("action/authority")
  , authorityServer(nh, authorityServerName,
                    ::boost::bind(&Actions::authorityCallback, this, ::_1),
                    false)
  , motorSwitchServerName("action/motor_switch")
  , motorSwitchServer(nh, motorSwitchServerName,
                      ::boost::bind(&Actions::motorSwitchCallback, this, ::_1),
                      false)
  , flightTaskServerName("action/flight_task")
  , flightTaskServer(nh, flightTaskServerName,
                     ::boost::bind(&Actions::flightTaskCallback, this, ::_1),
                     false)
{
  authorityFeedback.isSDKControl = false;
  authorityResult.accept         = false;
}

Actions::~Actions()
{
}

void
Actions::init(DJI::OSDK::Vehicle* v)
{
  vehicle = v;
  authorityServer.start();
  motorSwitchServer.start();
  flightTaskServer.start();
}

void
Actions::authorityCallback(const dji_sdk::AuthorityGoalConstPtr& goal)
{
  DJI::OSDK::ACK::ErrorCode ack;
  bool                      result = false;
  if (authorityServer.isPreemptRequested())
    authorityServer.setPreempted();
  if (goal->reqSDKControl)
  {
    ack    = vehicle->obtainCtrlAuthority(500);
    result = true;
    ROS_DEBUG("called vehicle->obtainCtrlAuthority");
  }
  else
  {
    ack    = vehicle->releaseCtrlAuthority(500);
    result = false;
    ROS_DEBUG("called vehicle->releaseCtrlAuthority");
  }
  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  bool success = false;
  if (DJI::OSDK::ACK::getError(ack))
  {
    success = false;
    DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    success = true;

    authorityFeedback.isSDKControl = result;
  }
  authorityResult.accept = true;

  authorityServer.setSucceeded(authorityResult);
}

void
Actions::motorSwitchCallback(const dji_sdk::MotorSwitchGoalConstPtr& goal)
{
  DJI::OSDK::ACK::ErrorCode ack;

  bool isOn = false;

  if (motorSwitchServer.isPreemptRequested())
    motorSwitchServer.setPreempted();

  if (goal->startMotor)
  {
    ack  = vehicle->control->armMotors(500);
    isOn = true;
  }
  else
  {
    ack  = vehicle->control->disArmMotors(500);
    isOn = false;
  }
  //! @todo implement
  bool success = false;
  if (DJI::OSDK::ACK::getError(ack))
  {
    success = false;
    DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    success = true;

    motorSwitchFeedback.isMotorOn = isOn;
  }

  motorSwitchResult.accept = true;
  motorSwitchServer.setSucceeded(motorSwitchResult);
}

void
Actions::flightTaskCallback(const dji_sdk::FlightTaskGoalConstPtr& goal)
{
  DJI::OSDK::ACK::ErrorCode ack;
  bool                      result = false;
  if (authorityServer.isPreemptRequested())
    authorityServer.setPreempted();

  switch (goal->task)
  {
    case dji_sdk::FlightTaskGoal::TASK_LANDING:
      ack = vehicle->control->land(500);
      break;
    defalut:
      ROS_WARN("unknown task enum %d", goal->task);
      flightTaskResult.accept = false;
      flightTaskServer.setSucceeded(flightTaskResult);
      return; //! @note returned here
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  bool success = false;
  if (DJI::OSDK::ACK::getError(ack))
  {
    success = false;
    DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    success = true;

    flightTaskFeedback.result = result;
  }
  flightTaskResult.accept = true;

  flightTaskServer.setSucceeded(flightTaskResult);
}
