#!/usr/bin/env python
from __future__ import print_function

from swarm_drone_proxy.msg import swarm_drone_source_data
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


class SimulateDronesEnv(object):
    def __init__(self, drone_num = 10, enable_pub_swarm = True, self_id = 0, pattern="RANDOM_WALK"):
        self.drone_pos =  np.array([[ 43.70093261,  10.61551406,   2.36836231],
            [  1.5152637, -19.0210859,   18.42497852],
            [ -6.87888708,  -7.30557088,  -2.98264606],
            [-14.95149762, -28.57279825,  -3.99203142],
            [ -5.6960475,   24.00769547,   3.45203488],
            [ -2.68213756,  23.92113782, -37.85933024],
            [ -5.47757494, -33.02566431,   8.23437282],
            [-34.94482282, -19.9569567,  -10.87550255],
            [-19.89705788,  -0.04917075, -44.4819556 ],
            [ 10.18059028,   1.6724144,  -34.09760646]])[0:drone_num]
 
        self.drone_vel = np.zeros((drone_num, 3))
        self.drone_num = drone_num
        self.drone_dis = np.zeros((drone_num, drone_num))
        self.colors = matplotlib.cm.rainbow(np.linspace(0, 1, drone_num))
        self.count = 0
        self.self_id = self_id
        self.enable_pub_swarm = enable_pub_swarm
        # self.base_coor = self.drone_pos + np.random.randn(drone_num, 3)*0.2
        # print(self.base_coor)
        self.base_coor = np.array([
            [0, 0, 0],
            [-21.48147416 , 12.7661877,   21.20343478],
            [ -3.3077791 ,  -9.87719654,  14.30700117],
            [-35.50198334, -21.12191708,  32.77340531],
            [-22.59650833,  -2.95609427, -20.10679965],
            [-31.24850392,  19.58565513,  -4.74885159],
            [ 33.30543244, -13.61200742, -10.19166553],
            [  7.25038821, -20.35836745,   5.3823983 ],
            [  8.73040171,  -5.20697205,  23.1825567 ],
            [ 11.51975686,   3.42533134,   3.74197347]])[0:drone_num]

        self.base_coor[[self.self_id,0]] = self.base_coor[[0,self.self_id]]
        self.base_yaw = (np.random.randn(drone_num) * 10 + np.pi) % (2*np.pi) - np.pi
        # self.base_yaw = np.zeros(drone_num)
        self.base_yaw[self.self_id] = 0
        self.est_err_norm = []

        # print("Initial pos", self.drone_pos)
        print("Base_coor", self.base_coor)


        self.poses_pub = rospy.Publisher("/swarm_drones/swarm_drone_source_data", swarm_drone_source_data, queue_size=1)
        self.self_odom_pub =  rospy.Publisher("/vins_estimator/odometry", Odometry, queue_size=1)
        self.tm = rospy.Timer(rospy.Duration(0.01), self.update)


    def update_vel(self, dt):
        move_num = 10
        # self.drone_vel = self.drone_vel + np.random.randn(self.drone_num, 3) *2* dt - self.drone_pos * 0.05*dt
        self.drone_vel[0:move_num] = self.drone_vel[0:move_num] + np.random.randn(move_num, 3) *2* dt - self.drone_pos[0:move_num] * 0.2*dt
        


    def update(self, e, dt=0.01, show=False):
        self.update_vel(dt)
        self.drone_pos = self.drone_pos + self.drone_vel * dt
        for i in range(self.drone_num):
            for j in range(self.drone_num):
                if  i!=j:
                    dx = self.drone_pos[i][0] - self.drone_pos[j][0]
                    dy = self.drone_pos[i][1] - self.drone_pos[j][1]
                    dz = self.drone_pos[i][2] - self.drone_pos[j][2]
                    # self.drone_dis[i][j] = self.drone_dis[j][i] = np.clip(math.sqrt(dx*dx + dy*dy + dz*dz),0,None)
                    self.drone_dis[i][j] = self.drone_dis[j][i] = np.clip(math.sqrt(dx*dx + dy*dy + dz*dz) + np.random.randn()*0.1,0,None)
                else:
                   self.drone_dis[i][j] = self.drone_dis[j][i] = 0
        if show and self.count % 10 == 0:
            # ax.set_title(f"Time: {self.count*dt:4.2f}")
            ax.clear()
            ax.scatter(self.drone_pos[:,0], self.drone_pos[:,1], self.drone_pos[:,2],s=100, c=self.colors)
            ax.quiver(self.drone_pos[:,0], self.drone_pos[:,1], self.drone_pos[:,2],
                self.drone_vel[:,0], self.drone_vel[:,1], self.drone_vel[:,2])
            fig.tight_layout(pad=0.1, w_pad=0.1, h_pad=0.1)

            plt.pause(0.1)

        Xii = self.drone_pos - self.base_coor[0:self.drone_num]
        Vii = self.drone_vel


        for i in range(self.drone_num):
            th = self.base_yaw[i]
            mat = np.array(
            [[math.cos(th), +math.sin(th), 0],
            [-math.sin(th), math.cos(th), 0],
            [0, 0, 1]])
            Xii[i] = np.dot(mat, Xii[i])
            Vii[i] = np.dot(mat, Vii[i])

        # print("DronePos", self.drone_pos)
        # print("Xii", Xii)
        rpos = swarm_drone_source_data()
        rpos.self_id = self.self_id
        rpos.drone_num = self.drone_num
        rpos.ids = range(self.drone_num)
        rpos.active = [True for i in range(self.drone_num)]
        rpos.distance_matrix = [0 for j in range(self.drone_num*self.drone_num)]
        rpos.distance_time = [0 for j in range(self.drone_num*self.drone_num)]
        for i in range(self.drone_num):
            pose = Pose()
            pose.orientation.w = 1
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.position.x = Xii[i][0]
            pose.position.y = Xii[i][1]
            pose.position.z = Xii[i][2]


            odom = Odometry()
            odom.pose.pose = pose

            odom.twist.twist.linear.x = Vii[i][0]
            odom.twist.twist.linear.y = Vii[i][1]
            odom.twist.twist.linear.z = Vii[i][2]

            rpos.drone_self_poses.append(odom)

            if i == self.self_id:
                self.self_odom_pub.publish(odom) 

        for i in range(self.drone_num):
            for j in range(self.drone_num):
                rpos.distance_matrix[i*self.drone_num + j]  = self.drone_dis[i][j]
                rpos.distance_time[i*self.drone_num + j] = 0

        self.count = self.count + 1
        if self.enable_pub_swarm:
            self.poses_pub.publish(rpos)


if __name__ == "__main__":
    # plt.ion()
    print("Starting vo data generation")
    rospy.init_node("vo_data_gen")
    drone_num = rospy.get_param('~drone_num', 10)
    self_id = rospy.get_param("~self_id", 0)
    enable_pub_swarm = rospy.get_param("~enable_pub_drone_source", True)
    env = SimulateDronesEnv(drone_num=drone_num, self_id=self_id, enable_pub_swarm=enable_pub_swarm)
    rospy.spin()