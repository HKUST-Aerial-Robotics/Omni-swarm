#!/usr/bin/env python
from __future__ import print_function

import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from swarm_msgs.msg import swarm_frame, node_frame


class SimulateDronesEnv(object):
    def __init__(self, drone_num = 10, enable_pub_swarm = True, self_id = 0, pattern="RANDOM_WALK"):
        self.drone_pos =  np.array([
            [0, 0, 0],
            [-1.48147416 , 2.7661877,   .20343478],
            [ -3.3077791 ,  -1.87719654,  2.30700117],
            [-3.50198334, -2.12191708,  .77340531],
            [-2.59650833,  -.95609427, 1.10679965],
            [-1.24850392,  .58565513,  .74885159],
            [ 3.30543244, -1.61200742, 1.19166553],
            [  1.25038821, -2.35836745,   1.3823983 ],
            [  1.73040171,  -5.20697205,  2.1825567 ],
            [ 1.51975686,   3.42533134,   1.74197347]])[0:drone_num]
 
        # self.drone_vel = np.random.randn(drone_num,3) * 5
        self.drone_vel = np.zeros((drone_num, 3))
        self.drone_vel[0] = np.array([1,-0.5,0.1])

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
            [-1.48147416 , 2.7661877,   .20343478],
            [ -3.3077791 ,  -1.87719654,  2.30700117],
            [-3.50198334, -2.12191708,  .77340531],
            [-2.59650833,  -.95609427, 1.10679965],
            [-1.24850392,  .58565513,  .74885159],
            [ 3.30543244, -1.61200742, 1.19166553],
            [  1.25038821, -2.35836745,   1.3823983 ],
            [  1.73040171,  -5.20697205,  2.1825567 ],
            [ 1.51975686,   3.42533134,   1.74197347]])[0:drone_num]

        self.base_coor[[self.self_id,0]] = self.base_coor[[0,self.self_id]]
        self.base_yaw = (np.random.randn(drone_num) * 10 + np.pi) % (2*np.pi) - np.pi
        # self.base_yaw = np.zeros(drone_num)
        self.base_yaw[self.self_id] = 0
        self.est_err_norm = []

        # print("Initial pos", self.drone_pos)
        print("Base_coor", self.base_coor)


        self.sf_pub = rospy.Publisher("/swarm_drones/swarm_frame", swarm_frame, queue_size=1)
        self.self_odom_pub =  rospy.Publisher("/vins_estimator/odometry", Odometry, queue_size=1)
        self.tm = rospy.Timer(rospy.Duration(0.01), self.update)


    def update_vel(self, dt, move_num=None):
        # move_num = 1
        if move_num is None:
            move_num = self.drone_num
        self.drone_vel[:,0:2] = self.drone_vel[:,0:2] + np.random.randn(move_num, 2) *0.05 * dt - self.drone_pos[:, 0:2] * 0.01*dt
        self.drone_vel[:,2] = self.drone_vel[:,2] + np.random.randn(1) *0.02* dt - self.drone_pos[:,2] * 0.1*dt
        # print(self.drone_vel[:,2] + np.random.randn(1) *0.02* dt)
        print(self.drone_pos)

    def generate_node_frame(self, i, Xii, Vii, ts):
        _nf = node_frame()

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

        # odom.header.frame_id = "my_frame"
        odom.twist.twist.linear.x = Vii[i][0]
        odom.twist.twist.linear.y = Vii[i][1]
        odom.twist.twist.linear.z = Vii[i][2]

        _nf.header.stamp = ts
        _nf.odometry = odom
        _nf.vo_available = True
        _nf.id = i

        for j in range(drone_num):
            if i!=j:
                _nf.dismap_ids.append(j)
                _nf.dismap_dists.append(self.drone_dis[i][j])
        return _nf

    def update(self, e, dt=0.1, show=False):
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
            ax = plt.subplot(111)
            ax.clear()
            ax.scatter(self.drone_pos[:,0], self.drone_pos[:,1], self.drone_pos[:,2],s=100, c=self.colors)
            ax.quiver(self.drone_pos[:,0], self.drone_pos[:,1], self.drone_pos[:,2],
                self.drone_vel[:,0], self.drone_vel[:,1], self.drone_vel[:,2])
            # fig.tight_layout(pad=0.1, w_pad=0.1, h_pad=0.1)

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
        ts = rospy.get_rostime()
        sf = swarm_frame()
        sf.header.stamp = ts
        sf.self_id = self.self_id
        for i in range(self.drone_num):
            _nf = self.generate_node_frame(i, Xii, Vii, ts)
            sf.node_frames.append(_nf)

        self.count = self.count + 1
        if self.enable_pub_swarm:
            self.sf_pub.publish(sf)


if __name__ == "__main__":
    # plt.ion()
    print("Starting vo data generation")
    rospy.init_node("vo_data_gen")
    drone_num = rospy.get_param('~drone_num', 4)
    self_id = rospy.get_param("~self_id", 0)
    enable_pub_swarm = rospy.get_param("~enable_pub_drone_source", True)
    env = SimulateDronesEnv(drone_num=drone_num, self_id=self_id, enable_pub_swarm=enable_pub_swarm)
    rospy.spin()