#!/usr/bin/env python
from __future__ import print_function

from swarm_msgs.msg import swarm_drone_source_data
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import matplotlib
import numpy.linalg as LA
import random
import copy
import rospy

self_odom_pub =  rospy.Publisher("/vins_estimator/imu_propagate", Odometry, queue_size=1)
pose = Pose()
pose.orientation.w = 1
pose.orientation.x = 0
pose.orientation.y = 0
pose.orientation.z = 0
pose.position.x = 0
pose.position.y = 0
pose.position.z = 0


odom = Odometry()
odom.pose.pose = pose

odom.header.frame_id = "world"
odom.child_frame_id = "world"
odom.twist.twist.linear.x = 0
odom.twist.twist.linear.y = 0
odom.twist.twist.linear.z = 0




if __name__ == "__main__":
    rospy.init_node("swarm_fix_odom")
    rate = rospy.Rate(50)
    print("Starting vo data generation")
    while not rospy.is_shutdown():
        self_odom_pub.publish(odom) 
        rate.sleep()
