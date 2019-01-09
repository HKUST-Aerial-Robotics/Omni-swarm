#include <actionlib/server/simple_action_server.h>
#include <dji_sdk/AuthorityAction.h>
#include <dji_sdk/FlightTaskAction.h>
#include <dji_sdk/MotorSwitchAction.h>
#include <dji_vehicle.hpp>
#include <ros/ros.h>

namespace DJISDK
{

class Actions
{
public:
  typedef actionlib::SimpleActionServer<dji_sdk::AuthorityAction>
    AuthorityServer;

  typedef actionlib::SimpleActionServer<dji_sdk::MotorSwitchAction>
    MotorSwitchServer;

  typedef actionlib::SimpleActionServer<dji_sdk::FlightTaskAction>
    FlightTaskServer;

public:
  Actions();
  ~Actions();

  void init(DJI::OSDK::Vehicle* v);

public:
  void authorityCallback(const dji_sdk::AuthorityGoalConstPtr& goal);
  void motorSwitchCallback(const dji_sdk::MotorSwitchGoalConstPtr& goal);
  void flightTaskCallback(const dji_sdk::FlightTaskGoalConstPtr& goal);

private:
  DJI::OSDK::Vehicle* vehicle;

protected:
  ros::NodeHandle nh;
  //!@ note order cannot be changed or the constructor will fail
  std::string                authorityServerName;
  AuthorityServer            authorityServer;
  dji_sdk::AuthorityFeedback authorityFeedback;
  dji_sdk::AuthorityResult   authorityResult;

  std::string                  motorSwitchServerName;
  MotorSwitchServer            motorSwitchServer;
  dji_sdk::MotorSwitchFeedback motorSwitchFeedback;
  dji_sdk::MotorSwitchResult   motorSwitchResult;

  std::string                 flightTaskServerName;
  FlightTaskServer            flightTaskServer;
  dji_sdk::FlightTaskFeedback flightTaskFeedback;
  dji_sdk::FlightTaskResult   flightTaskResult;
};

} // namespace  DJISDK
