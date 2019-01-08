#include <ros/ros.h>

#include <dji_sdk/SetLocalPosRef.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Eigen>

//! @attention for simulation only

ros::ServiceClient          set_local_pos_reference;
geometry_msgs::PointStamped last_local_position;
nav_msgs::Odometry          odom;
nav_msgs::Odometry          last_odom;

ros::Publisher odom_pub;
ros::Publisher imu_pub;

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
int
main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_to_odom");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  odom_pub =
    nh.advertise<nav_msgs::Odometry>("/dji_sdk_1/dji_sdk/simulation/odom", 10);
  imu_pub =
    nh.advertise<sensor_msgs::Imu>("/dji_sdk_1/dji_sdk/simulation/imu", 10);
  set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>(
    "/dji_sdk_1/dji_sdk/set_local_pos_ref");
  set_local_pos_reference.waitForExistence();
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);

  ros::Subscriber local_pos_sub = nh.subscribe("/dji_sdk_1/dji_sdk/local_position",
                                               10, &local_position_callback);
  ros::Subscriber imu_sub =
    nh.subscribe("/dji_sdk_1/dji_sdk/imu", 10, &imu_callback);
  ros::Subscriber velocity_sub =
    nh.subscribe("/dji_sdk_1/dji_sdk/velocity", 10, &velocity_callback);

  ros::Rate r(50);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
void
imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

  Eigen::Quaterniond q_in(msg->orientation.w, msg->orientation.x,
                          msg->orientation.y, msg->orientation.z);

  odom.pose.pose.orientation.w = q_in.w();
  odom.pose.pose.orientation.x = q_in.x();
  odom.pose.pose.orientation.y = q_in.y();
  odom.pose.pose.orientation.z = q_in.z();

  odom.twist.twist.angular.x = msg->angular_velocity.x;
  odom.twist.twist.angular.y = msg->angular_velocity.y;
  odom.twist.twist.angular.z = msg->angular_velocity.z;

  sensor_msgs::Imu ans;
  ans.header = msg->header;

  ans.angular_velocity.x = msg->angular_velocity.x;
  ans.angular_velocity.y = msg->angular_velocity.y;
  ans.angular_velocity.z = msg->angular_velocity.z;

  Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x,
                       msg->orientation.y, msg->orientation.z);
  Eigen::Vector3d acc_b(msg->linear_acceleration.x, msg->linear_acceleration.y,
                        msg->linear_acceleration.z);

  /** @note the following code should be the right way to use imu data,
   * however, DJI simulation hardware in loop did not implement the Sensor level
   * imu data
   * therefore, the acc_b are incorrect.
   * the simulation imu out put from dji FC is Eigen::Vector3d(1,1,G) in world
   * frame (NEU).
   *
  Eigen::Vector3d acc_w =
    q.toRotationMatrix().transpose() * acc_b - Eigen::Vector3d(0, 0, 1) * 9.81;
  ans.linear_acceleration.x = acc_w.y();
  ans.linear_acceleration.y = acc_w.x();
  ans.linear_acceleration.z = acc_w.z();
   * */
  // dt is based on GPS time which is not simulated by SDK
  //  double dt = odom.header.stamp.toSec() - last_odom.header.stamp.toSec();
  //! @note velocity freq should be 50 hz

  
  ans.linear_acceleration.x =
    (odom.twist.twist.linear.x - last_odom.twist.twist.linear.x) * 50;
  ans.linear_acceleration.y =
    (odom.twist.twist.linear.y - last_odom.twist.twist.linear.y) * 50;
  ans.linear_acceleration.z =
    (odom.twist.twist.linear.z - last_odom.twist.twist.linear.z) * 50;

  ans.orientation.w = msg->orientation.w;
  ans.orientation.x = msg->orientation.x;
  ans.orientation.y = msg->orientation.y;
  ans.orientation.z = msg->orientation.z;
  imu_pub.publish(ans);
}

void
velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  //Need convert ENU to FLU, thatis NWU

  odom.twist.twist.linear.x = msg->vector.y;
  odom.twist.twist.linear.y = - msg->vector.x;
  odom.twist.twist.linear.z = msg->vector.z;
}

void
local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  odom.header = msg->header;

  double dt =
    msg->header.stamp.toSec() - last_local_position.header.stamp.toSec();

  //Need convert ENU to FLU, thatis NWU

  odom.pose.pose.position.x = msg->point.y;
  odom.pose.pose.position.y = - msg->point.x;
  odom.pose.pose.position.z = msg->point.z;

  // odom.twist.twist.linear.x = (msg->point.x - last_local_position.point.x) /
  // dt;
  // odom.twist.twist.linear.y = (msg->point.y - last_local_position.point.y) /
  // dt;
  // odom.twist.twist.linear.z = (msg->point.z - last_local_position.point.z) /
  // dt;

  last_local_position = *msg;

  odom_pub.publish(odom);
  last_odom = odom;
}