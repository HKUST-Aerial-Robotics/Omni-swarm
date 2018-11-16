#!/usr/bin/env python2
from __future__ import print_function

import matplotlib
matplotlib.use('Qt5Agg') 
import rospy
import pymavlink
import sys
import time
from swarm_msgs.msg import data_buffer, swarm_drone_source_data, remote_uwb_info, swarm_fused_relative, swarm_fused
import time
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading
import math
import matplotlib.pyplot as plt
import numpy as np

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
        self.swarm_relatived_sub = rospy.Subscriber("/swarm_drones/swarm_drone_fused", swarm_fused, self.on_swarm_fused, queue_size=1)
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

        self.vicon_pose = {}

        self.planar_err = {}
        self.planar_x_err = {}
        self.planar_y_err = {}
        self.vertical_err = {}

        self.odom_offset_sum = Point(0, 0, 0)
        self.odom_offset_count = 0
        self.odom_yaw_offset_sum = 0

        for i in est_ids:
            self.dis_uwb[i] = {}
            self.dis_vicon[i] = {}
            self.dis_err[i] = {}
            self.planar_err[i] = []
            self.planar_x_err[i] = []
            self.planar_y_err[i] = []
            self.vertical_err[i] = []
            for j in est_ids:
                self.dis_uwb[i][j]  = []
                self.dis_vicon[i][j] = []
                self.dis_err[i][j] = []

    
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
            
            self.dis_vicon[_id][_id_j] = _dis
            self.dis_vicon[_id_j][_id] = _dis

        self.vicon_pose[_id] = _pose

        # print(self.dis_vicon)

    def on_vo_odom(self, odom):
        quat = odom.pose.pose.orientation
        odom_position = odom.pose.pose.position
        quaternion = (quat.x, quat.y, quat.z, quat.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.yaw_odom = yaw

        vicon_main_position = self.vicon_pose[self.main_id].pose.position

        self.odom_offset_sum.x += vicon_main_position.x - odom_position.x
        self.odom_offset_sum.y += vicon_main_position.y - odom_position.y
        self.odom_offset_sum.z += vicon_main_position.z - odom_position.z

        self.odom_yaw_offset_sum = self.yaw_vicon - self.yaw_odom
        
        self.odom_offset_count = self.odom_offset_count + 1

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
    
    def on_swarm_fused(self, swarm_fused):
        positions = swarm_fused.remote_drone_position
        for i in range(len(swarm_fused.ids)):
            target_id = swarm_fused.ids[i]
            pos_in_odom = positions[i]
            if target_id not in self.swarm_est_in_vicon:
                self.swarm_est_in_vicon[target_id] = rospy.Publisher("/swarm_drone/estimate_pose_{}".format(target_id), PoseStamped)
            _pose = PoseStamped()
            remote_pose = _pose.pose
            _pose.header.frame_id = "world"
            _pose.header.stamp = rospy.Time.now()

            yaw_off_set = self.odom_yaw_offset_sum / self.odom_offset_count
            # print("Yaw vicon {} odom {}".format(self.yaw_vicon, self.yaw_odom))
            sx = math.sin(yaw_off_set)
            cx = math.cos(yaw_off_set)
            posx = remote_pose.position.x = self.odom_offset_sum.x/self.odom_offset_count + pos_in_odom.x * cx + pos_in_odom.y * sx
            posy = remote_pose.position.y = self.odom_offset_sum.y/self.odom_offset_count + pos_in_odom.y * cx - pos_in_odom.x * sx
            posz = remote_pose.position.z = self.odom_offset_sum.z/self.odom_offset_count + pos_in_odom.z

            # posx = remote_pose.position.x = self.odom_offset_sum.x/self.odom_offset_count + pos_in_odom.x
            # posy = remote_pose.position.y = self.odom_offset_sum.y/self.odom_offset_count + pos_in_odom.y
            # posz = remote_pose.position.z = self.odom_offset_sum.z/self.odom_offset_count + pos_in_odom.z

            vicon_pos = self.vicon_pose[target_id].pose.position
            planar_err = math.sqrt((posx - vicon_pos.x)**2 + (posy - vicon_pos.y)**2)
            planar_x_err = posx - vicon_pos.x
            planar_y_err = posy - vicon_pos.y
            vertical_err = posz - vicon_pos.z
            self.planar_err[target_id].append(planar_err*100)
            self.planar_x_err[target_id].append(planar_x_err*100)
            self.planar_y_err[target_id].append(planar_y_err*100)
            self.vertical_err[target_id].append(vertical_err*100)
            # print(self.planar_err)

            # if target_id == 8:
                # print("!!!!!!!!!!!!!!!!!err: {:3.2f} {:3.2f} {:3.2f}".format(planar_x_err, planar_y_err, vertical_err))

            # remote_pose.orientation.w = 1
            self.swarm_est_in_vicon[target_id].publish(_pose)

    
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
            # print("Yaw vicon {} odom {}".format(self.yaw_vicon, self.yaw_odom))
            sx = math.sin(yaw_off_set)
            cx = math.cos(yaw_off_set)
            posx = remote_pose.position.x = main_position.x + rel_pos.x * cx + rel_pos.y * sx
            posy = remote_pose.position.y = main_position.y + rel_pos.y * cx - rel_pos.x * sx
            posz = remote_pose.position.z = main_position.z + rel_pos.z


            vicon_pos = self.vicon_pose[target_id].pose.position
            planar_err = math.sqrt((posx - vicon_pos.x)**2 + (posy - vicon_pos.y)**2)
            vertical_err = (posz - vicon_pos.z)
            # self.planar_err[target_id].append(planar_err)
            # self.vertical_err[target_id].append(vertical_err)
            # print(self.planar_err)

            # remote_pose.orientation.w = 1
            # self.swarm_est_in_vicon[target_id].publish(_pose)


if __name__ == "__main__":
    rospy.init_node("SwarmOfflineTune")
    # plt.ion()
    tune = SwarmOfflineTune(7, [7, 0, 3, 8], display=True)
    while not rospy.is_shutdown():
        for idx in tune.dis_err:
            if idx == tune.main_id:
                continue
            plt.figure("Estimation Err {}".format(idx),figsize=(12,8))
            plt.clf()
            plt.subplot(231)

            plt.title("Planar Err")
            plt.plot(tune.planar_err[idx], label="Planar")
            plt.plot(tune.planar_x_err[idx], label="PlanarErrX")
            plt.plot(tune.planar_y_err[idx], label="PlanarErrY")
            plt.legend()
            plt.grid(which="both")

            plt.subplot(232)
            plt.title("Vertical Err")
            plt.plot(tune.vertical_err[idx])
            plt.grid(which="both")

            plt.subplot(233)
            plt.title("Planar Err {:3.1f}cm   |$\sigma$:{:3.1f}cm".format(np.mean(tune.planar_err[idx]),math.sqrt(np.var(tune.planar_err[idx]))))
            plt.hist(tune.planar_err[idx], 50, normed=True)
            plt.grid(which="both")

            # print(tune.planar_x_err[idx])
            plt.subplot(234)
            plt.title("Planar X Err {:3.1f}cm |$\sigma$:{:3.1f}cm".format(np.mean(tune.planar_x_err[idx]),math.sqrt(np.var(tune.planar_x_err[idx]))))
            plt.hist(tune.planar_x_err[idx], 50, normed=True)
            plt.grid(which="both")

            plt.subplot(235)
            plt.title("Planar Y Err {:3.1f}cm |$\sigma$:{:3.1f}cm".format(np.mean(tune.planar_y_err[idx]),math.sqrt(np.var(tune.planar_y_err[idx]))))
            plt.hist(tune.planar_y_err[idx], 50, normed=True)
            plt.grid(which="both")
            
            plt.subplot(236)
            plt.title("Vertical Err {:3.1f}cm |$\sigma$:{:3.1f}cm".format(np.mean(tune.vertical_err[idx]),math.sqrt(np.var(tune.vertical_err[idx]))))
            plt.hist(tune.vertical_err[idx], 50, normed=True)
            plt.grid(which="both")

            plt.tight_layout()

            plt.pause(0.05)


            for idy in tune.dis_err[idx]:
                if idx < idy:
                    plt.figure("Figure {} to {} Error".format(idx, idy))
                    plt.clf()

                    plt.subplot(211)
                    plt.plot(tune.dis_err[idx][idy],'o', label="Figure {} to {} Error".format(idx, idy))
                    plt.legend()
                    plt.grid(which="both")

                    plt.subplot(212)
                    plt.hist(tune.dis_err[idx][idy], 50)
                    plt.title("Dis error {}".format(np.mean(tune.dis_err[idx][idy] )))
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