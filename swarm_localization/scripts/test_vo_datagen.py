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
from tf.transformations import quaternion_from_euler

def parse_csv_data(csv_path, lt=0, rt=1000000):
    data =  np.genfromtxt(csv_path, delimiter=',')
    l = 0
    r = len(data[:,0]) - 1
    while data[l, 0] < lt:
        l += 1

    while data[r, 0] > rt:
        r -= 1

    # print(f"Find time {lt}:{rt}s with {l}:{r}")

    ans = {}

    """
    ts : 0
    ctrl_mode 1
    posx, posy, posz 2:5
    velx, vely, velz 5:8
    att r p y 8:11
    pos_sp x y z 11:14
    vel_sp x y z 14:17
    acc_sp x y z 17:20
    attout rpy 20:23
    thrsp 24
    """
    ans["ts"] = data[l:r,0]
    ans["ctrl_mode"] = data[l:r,1]
    ans['pos'] = data[l:r,2:5]
    ans['vel'] = data[l:r,5:8]
    ans['rpy'] = data[l:r,8:11]
    ans['pos_sp'] = data[l:r,11:14]
    ans['vel_sp'] = data[l:r,14:17]
    ans['acc_sp'] = data[l:r,17:20]
    ans["rpy_sp"] = data[l:r,20:23]
    ans["thr_sp"] = data[l:r,23]
    ans['rpy_fc'] = data[l:r,24:26]
    return ans


class SimulateDronesEnv(object):
    def __init__(self, drone_num = 10, enable_pub_swarm = True, self_id = 0):
        self.drone_vel = np.zeros((drone_num, 3))
        self.data_path = "/home/xuhao/swarm_ws/src/swarm_pkgs/swarm_localization/data/"
        self.data_paths = [
            ("log_2019-10-15-2-17-circle.csv", 102),
            # ("2019-3-6-fast-circle.csv", 262),
            ("2019-3-6-sweep-hover-y.csv", 48),
            ("realsense_2019_5_15_loop.csv", 20),
            ("circle-3s-no-gc-fix.csv", 18)
        ]

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
            [2.48147416 , 1.7661877,   .20343478],
            [ -3.3077791 ,  -1.87719654,  .30700117],
            [3.50198334, -2.12191708,  .17340531],
            [-2.59650833,  -.95609427, .10679965],
            [-1.24850392,  .58565513,  .1],
            [ 3.30543244, -1.61200742, 0],
            [  1.25038821, -2.35836745,   1.3823983 ],
            [  -1.73040171,  -5.20697205,  2.1825567 ],
            [ 1.51975686,   3.42533134,   1.74197347]])[0:drone_num]
        # self.base_coor = np.random.rand(4, 3) * 0.1

        self.drone_pos = self.base_coor.copy()

        self.base_yaw = (np.random.randn(drone_num) * 10 + np.pi) % (2*np.pi) - np.pi
        # self.base_yaw = np.zeros(drone_num)
        self.base_yaw[self.self_id] = 0
        self.est_err_norm = []

        # print("Initial pos", self.drone_pos)
        print("Base_coor", self.base_coor)


        self.sf_pub = rospy.Publisher("/swarm_drones/swarm_frame", swarm_frame, queue_size=1)
        self.self_odom_pub =  rospy.Publisher("/vins_estimator/odometry", Odometry, queue_size=1)
        self.odom_pubs = [
            rospy.Publisher("/swarm_drones_sim/odom0", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom1", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom2", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom3", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom4", Odometry, queue_size=1)
        ]

        self.tick = 0
        self.load_datas()

        self.tm = rospy.Timer(rospy.Duration(0.1), self.update)


    def load_datas(self):
        self.data = []
        for p, l in self.data_paths:
            self.data.append(parse_csv_data(self.data_path + p, l))
            print("{} len {}".format(p, len(self.data[-1]['pos'])))

    def generate_node_frame(self, i, Xii, Vii, ts, tick):
        _nf = node_frame()

        pose = Pose()
        roll = self.data[i]["rpy"][tick][0]
        pitch = self.data[i]["rpy"][tick][1]
        yaw = self.data[i]["rpy"][tick][2]
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.w = qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.position.x = Xii[i][0]
        pose.position.y = Xii[i][1]
        pose.position.z = Xii[i][2]


        odom = Odometry()
        odom.header.stamp = ts
        odom.header.frame_id = "world"
        odom.pose.pose = pose

        # odom.header.frame_id = "my_frame"
        odom.twist.twist.linear.x = Vii[i][0]
        odom.twist.twist.linear.y = Vii[i][1]
        odom.twist.twist.linear.z = Vii[i][2]




        _nf.header.stamp = ts
        _nf.odometry = odom
        _nf.vo_available = True
        _nf.id = i




        posew = Pose()
        posew.orientation.w = qw
        posew.orientation.x = qx
        posew.orientation.y = qy
        posew.orientation.z = qz
        posew.position.x = self.drone_pos[i][0]
        posew.position.y = self.drone_pos[i][1]
        posew.position.z = self.drone_pos[i][2]


        odomw = Odometry()
        odomw.header.stamp = ts
        odomw.header.frame_id = "world"
        odomw.pose.pose = posew

        # odom.header.frame_id = "my_frame"
        odomw.twist.twist.linear.x = Vii[i][0]
        odomw.twist.twist.linear.y = Vii[i][1]
        odomw.twist.twist.linear.z = Vii[i][2]
        self.odom_pubs[i].publish(odomw)

        for j in range(drone_num):
            if i!=j:
                _nf.dismap_ids.append(j)
                _nf.dismap_dists.append(self.drone_dis[i][j])
        return _nf

    def update(self, e, dt=0.1, show=False):
        if e.last_real is not None:
            dt = (e.current_real - e.last_real).to_sec()
            self.tick = self.tick + int(dt*50)
            # print(dt)
        for i in range(self.drone_num):
            # print(self.data[i])
            self.drone_pos[i] = self.data[i]["pos"][self.tick] + self.base_coor[i]
            self.drone_vel[i] = self.data[i]["vel"][self.tick]
            # print(self.tick, self.drone_pos[i], self.data[i]["pos"][self.tick])

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


        # for i in range(self.drone_num):
        #     th = self.base_yaw[i]
        #     mat = np.array(
        #     [[math.cos(th), +math.sin(th), 0],
        #     [-math.sin(th), math.cos(th), 0],
        #     [0, 0, 1]])
        #     Xii[i] = np.dot(mat, Xii[i])
        #     Vii[i] = np.dot(mat, Vii[i])

        # print("DronePos", self.drone_pos)
        # print("Xii", Xii)
        ts = rospy.get_rostime()
        sf = swarm_frame()
        sf.header.stamp = ts
        sf.self_id = self.self_id
        for i in range(self.drone_num):
            _nf = self.generate_node_frame(i, Xii, Vii, ts, self.tick)
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