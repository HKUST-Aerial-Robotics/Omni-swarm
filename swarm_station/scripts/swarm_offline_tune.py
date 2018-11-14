#!/usr/bin/env python2
from __future__ import print_function

import matplotlib
matplotlib.use('Qt5Agg') 
import rospy
import pymavlink
import sys
import time
from swarm_msgs.msg import data_buffer, swarm_drone_source_data, remote_uwb_info, swarm_fused_relative
import time
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading
import math
import matplotlib.pyplot as plt

def dis_over_pose(pose1, pose2):
    pos1 = pose1.pose.position
    pos2 = pose2.pose.position
    dx = pos1.x - pos2.x
    dy = pos1.y - pos2.y
    dz = pos1.z - pos2.z
    return math.sqrt(dx**2 + dy**2 + dz**2)


class SwarmOfflineTune:
    def __init__(self, main_id, est_ids, display=False):

        self.main_id = main_id
        self.main_pose = PoseStamped()
        self.swarm_source_data_sub = rospy.Subscriber("/swarm_drones/swarm_drone_source_data", swarm_drone_source_data, self.on_swarm_source_data, queue_size=1)
        self.swarm_relatived_sub = rospy.Subscriber("/swarm_drones/swarm_drone_fused_relative", swarm_fused_relative, self.on_swarm_fused_relative, queue_size=1)
        self.odom_sub= rospy.Subscriber("/vins_estimator/odometry", Odometry, self.on_vo_odom, queue_size=1)
        
        self.swarm_vicon_pose_sub = {}
        self.swarm_vicon_pose = {}
        self.swarm_est_in_vicon = {}
        self.on_vicon_pose_func = {}
        
        self.swarm_vicon_pose_sub[0] = rospy.Subscriber("/SwarmNode{}/pose".format(0), PoseStamped, self.on_vicon_pose_0 , queue_size=1)
        self.swarm_vicon_pose_sub[3] = rospy.Subscriber("/SwarmNode{}/pose".format(3), PoseStamped, self.on_vicon_pose_3 , queue_size=1)
        self.swarm_vicon_pose_sub[7] = rospy.Subscriber("/SwarmNode{}/pose".format(7), PoseStamped, self.on_vicon_pose_7 , queue_size=1)
        self.swarm_vicon_pose_sub[8] = rospy.Subscriber("/SwarmNode{}/pose".format(8), PoseStamped, self.on_vicon_pose_8 , queue_size=1)


        # self.main_odom_sub = rospy.Subscriber("/vins_estimator/odometry", odometry)
        
        print(self.swarm_vicon_pose_sub)

        self.yaw_vicon = 0
        self.yaw_odom = 0

        self.dis_vicon = {}
        self.display = display

        self.dis_err = {}
        self.dis_uwb = {}

        for i in est_ids:
            self.dis_uwb[i] = {}
            for j in est_ids:
                self.dis_uwb[i][j]  = []
        
    
    def on_vicon_pose_0(self, _pose):
        self.on_vicon_pose(0, _pose)

    def on_vicon_pose_3(self, _pose):
        self.on_vicon_pose(3, _pose)

    def on_vicon_pose_7(self, _pose):
        self.on_vicon_pose(7, _pose)

    def on_vicon_pose_8(self, _pose):
        self.on_vicon_pose(8, _pose)

    def on_vicon_pose(self, _id, _pose):
        if _id == self.main_id:
            self.main_pose = _pose
            quat = _pose.pose.orientation
            quaternion = (quat.x, quat.y, quat.z, quat.w)
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            self.yaw_vicon = yaw

        self.swarm_vicon_pose[_id] = _pose
        for _id_j in self.swarm_vicon_pose:
            # print(_id_j)
            _dis = dis_over_pose(_pose, self.swarm_vicon_pose[_id_j])
            
            if _id in self.dis_vicon:
                self.dis_vicon[_id][_id_j] = _dis
                if not (_id_j in self.dis_err[_id]):
                    self.dis_err[_id][_id_j] = []
            else:
                self.dis_vicon[_id] = {_id_j : _dis}
                self.dis_err[_id] = {_id_j : []}

            if _id_j in self.dis_vicon:
                self.dis_vicon[_id_j][_id] = _dis
                if not (_id in self.dis_err[_id_j]):
                    self.dis_err[_id_j][_id] = []
            else:
                self.dis_vicon[_id_j] = {_id : _dis}
                self.dis_err[_id_j] = {_id : []}

        # print(self.dis_vicon)

    def on_vo_odom(self, odom):
        quat = odom.pose.pose.orientation
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.yaw_odom = yaw

    def on_swarm_source_data(self, ssd):
        

        try:
            ids = ssd.ids
            num = len(ssd.ids)
            print("Dis UWB/Vicon")
            for i in range(num):
                pos = ssd.drone_self_odoms[i].pose.pose.position
                print("Self{} {:3.2f} {:3.2f} {:3.2f}".format(ids[i], pos.x, pos.y, pos.z),end="\t")
                for j in range(num):
                    _dis = 0.5 * (ssd.distance_matrix[i*num + j] + ssd.distance_matrix[j*num + i])        
                    
                    vicon_dis = self.dis_vicon[ids[i]][ids[j]]
                    self.dis_err[ids[i]][ids[j]].append(vicon_dis - _dis)
                    
                    self.dis_uwb[ids[i]][ids[j]].append(_dis)
                    print('{:3.2f}/{:3.2f}'.format(_dis, vicon_dis), end="\t")
                    
                print("")

            print("\n")
        except:
            # print(ssd)
            raise
            exit(0)
    
    def on_swarm_fused_relative(self, swarm_rel):
        rel_poses = swarm_rel.relative_drone_position
        for i in range(len(swarm_rel.ids)):
            target_id = swarm_rel.ids[i]
            rel_pos = rel_poses[i]
            if target_id == 8:
                _dis = rel_pos.x ** 2 + rel_pos.y**2 + rel_pos.z**2 
                print("Dis {}to8 {}".format(self.main_id, math.sqrt(_dis)))
            main_position = self.main_pose.pose.position
            if target_id not in self.swarm_est_in_vicon:
                self.swarm_est_in_vicon[target_id] = rospy.Publisher("/swarm_drone/estimate_pose_{}".format(target_id), PoseStamped)
            _pose = PoseStamped()
            remote_pose = _pose.pose
            _pose.header.frame_id = "world"
            _pose.header.stamp = rospy.Time.now()

            yaw_off_set = self.yaw_vicon - self.yaw_odom
            print("Yaw vicon {} odom {}".format(self.yaw_vicon, self.yaw_odom))
            sx = math.sin(yaw_off_set)
            cx = math.cos(yaw_off_set)
            remote_pose.position.x = main_position.x + rel_pos.x * cx + rel_pos.y * sx
            remote_pose.position.y = main_position.y + rel_pos.y * cx - rel_pos.x * sx
            remote_pose.position.z = main_position.z + rel_pos.z

            remote_pose.orientation.w = 1
            self.swarm_est_in_vicon[target_id].publish(_pose)


if __name__ == "__main__":
    rospy.init_node("SwarmOfflineTune")
    # plt.ion()
    tune = SwarmOfflineTune(7, [7, 0, 3, 8], display=True)
    while not rospy.is_shutdown():

        # plt.figure("Distance Error")
        # plt.clf()

        for idx in tune.dis_err:
            for idy in tune.dis_err[idx]:
                if idx < idy:
                    plt.figure("Figure {} to {} Error".format(idx, idy))
                    plt.clf()
                    plt.plot(tune.dis_err[idx][idy],'o', label="Figure {} to {} Error".format(idx, idy))
                    plt.legend()
                    plt.grid(which="both")
                    plt.pause(0.05)

                    plt.figure("Hist Figure {} to {} Error".format(idx, idy))
                    plt.clf()
                    plt.hist(tune.dis_err[idx][idy], 50)
                    plt.grid(which="both")
                    plt.pause(0.05)

        plt.figure("Distance")
        plt.clf()

        for idx in tune.dis_uwb:
            for idy in tune.dis_uwb[idx]:
                if idx < idy:
                    plt.plot(tune.dis_uwb[idx][idy], label="Figure {} to {}".format(idx, idy))

        plt.legend()
        plt.grid(which="both")
        plt.pause(0.05)

    rospy.spin()