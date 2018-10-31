#!/usr/bin/env python
from __future__ import print_function
import rospy
import pymavlink
import sys
import time
from swarm_msgs.msg import data_buffer, swarm_drone_source_data, remote_uwb_info, swarm_fused_relative
import time
from geometry_msgs.msg import Pose, PoseStamped
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading
import math

def dis_over_pose(pose1, pose2):
    pos1 = pose1.pose.position
    pos2 = pose2.pose.position
    dx = pos1.x - pos2.x
    dy = pos1.y - pos2.y
    dz = pos1.z - pos2.z
    return math.sqrt(dx**2 + dy**2 + dz**2)


class SwarmOfflineTune:
    def __init__(self, main_id, est_ids):

        self.main_id = main_id
        self.main_pose = PoseStamped()
        self.swarm_source_data_sub = rospy.Subscriber("/swarm_drones/swarm_drone_source_data", swarm_drone_source_data, self.on_swarm_source_data, queue_size=1)
        self.swarm_relatived_sub = rospy.Subscriber("/swarm_drones/swarm_drone_fused_relative", swarm_fused_relative, self.on_swarm_fused_relative, queue_size=1)
        
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

        self.dis_vicon = {}
    
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

        self.swarm_vicon_pose[_id] = _pose
        for _id_j in self.swarm_vicon_pose:
            # print(_id_j)
            _dis = dis_over_pose(_pose, self.swarm_vicon_pose[_id_j])
            
            if _id in self.dis_vicon:
                self.dis_vicon[_id][_id_j] = _dis
            else:
                self.dis_vicon[_id] = {_id_j : _dis}

            if _id_j in self.dis_vicon:
                self.dis_vicon[_id_j][_id] = _dis
            else:
                self.dis_vicon[_id_j] = {_id : _dis}
        # print(self.dis_vicon)

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
                    print('{:3.2f}/{:3.2f}'.format(_dis, vicon_dis), end="\t")
                print("")

            print("\n")
        except:
            print(ssd)
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
            remote_pose.position.x = main_position.x + rel_pos.x
            remote_pose.position.y = main_position.y + rel_pos.y
            remote_pose.position.z = main_position.z + rel_pos.z

            remote_pose.orientation.w = 1
            self.swarm_est_in_vicon[target_id].publish(_pose)


if __name__ == "__main__":
    rospy.init_node("SwarmOfflineTune")
    tune = SwarmOfflineTune(7, [7, 0, 3, 8])
    rospy.spin()