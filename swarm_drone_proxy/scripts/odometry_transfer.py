#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import *
import math
import numpy as np

pub = rospy.Publisher("/vins_estimator/odometry_ned", Odometry, queue_size=0)

def on_odometry(odom):
    q_rot = quaternion_from_euler(0, math.pi, 0)
    pos = np.array([
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z])
    vel = np.array([
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z])

    quat = np.array([
            odom.pose.pose.oriention.x,
            odom.pose.pose.oriention.y,
            odom.pose.pose.oriention.z,
            odom.pose.pose.oriention.w
            ])
    
    quat = q_rot * quat

    pos = q_rot * pos
    vel = q_rot * vel

    odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pos[0], pos[1], pos[2]
    odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = vel[0], vel[1], vel[2]
    odom.pose.pose.oriention.x = quat[0]
    odom.pose.pose.oriention.y = quat[1]
    odom.pose.pose.oriention.z = quat[2]
    odom.pose.pose.oriention.w = quat[3]

    pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("odometry_transfer")
    sub = rospy.Subscriber("/vins_estimator/odometry", Odometry, on_odometry, queue_size=0)
    rospy.spin()