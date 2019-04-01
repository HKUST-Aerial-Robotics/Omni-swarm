#!/usr/bin/env python
from __future__ import print_function

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion

pub = rospy.Publisher("/vins_estimator/odometry_ned", Odometry, queue_size=1)

def on_odometry(odom):

    latency = rospy.get_time() - odom.header.stamp.to_sec()

    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Lantency", latency*1000, " ms ")
    q_rot = Quaternion(axis=[0, 1, 0], angle=3.14159265)
    pos = np.array([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z])
    vel = np.array([
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z])

    quat = Quaternion(
            odom.pose.pose.orientation.w,
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z
            )
    
    quat = q_rot * quat * q_rot

    pos = q_rot.rotate(pos)
    vel = q_rot.rotate(vel)

    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pos[0], pos[1], pos[2]
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = vel[0], vel[1], vel[2]
    odom.pose.pose.orientation.w = quat.q[0]
    odom.pose.pose.orientation.x = quat.q[1]
    odom.pose.pose.orientation.y = quat.q[2]
    odom.pose.pose.orientation.z = quat.q[3]

    odom.header.frame_id = "world_ned"
    odom.child_frame_id = ""

    pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("odometry_transfer")
    sub = rospy.Subscriber("/vins_estimator/imu_propagate", Odometry, on_odometry, queue_size=1)
    rospy.spin()