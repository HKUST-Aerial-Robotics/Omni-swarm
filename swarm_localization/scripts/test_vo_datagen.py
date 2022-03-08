#!/usr/bin/env python
from __future__ import print_function

import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from swarm_msgs.msg import swarm_frame, node_frame, node_detected, swarm_detected
from tf.transformations import quaternion_from_euler
import random

def parse_csv_data(csv_path, lt=0, rt=1000000, zero_yaw= True, yaw_only=True):
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
    if zero_yaw:
        ans["rpy"][:,2] = 0
    if yaw_only:
        ans["rpy"][:,0] = 0
        ans["rpy"][:,1] = 0

    ans['pos_sp'] = data[l:r,11:14]
    ans['vel_sp'] = data[l:r,14:17]
    ans['acc_sp'] = data[l:r,17:20]
    ans["rpy_sp"] = data[l:r,20:23]
    ans["thr_sp"] = data[l:r,23]
    ans['rpy_fc'] = data[l:r,24:26]
    return ans


class SimulateDronesEnv(object):
    def __init__(self, drone_num = 10, self_id = 0, enable_detection = True, zero_yaw_offset = True, is_static = False, yaw_only=True):
        self.drone_vel = np.zeros((drone_num, 3))
        self.data_path = "/home/xuhao/swarm_ws/src/swarm_localization/swarm_localization/data/"
        self.data_paths = [
            ("log_2019-10-15-2-17-circle.csv", 102), #0
            ("2019-3-6-sweep-hover-y.csv", 48), #1
            ("realsense_2019_5_15_loop.csv", 20), #2
            ("circle-3s-no-gc-fix.csv", 18), #3
            ("2019-3-6-sweep-hover-y.csv", 38),#4
            ("realsense_2019_5_15_loop.csv", 15), #5
            ("circle-3s-no-gc-fix.csv", 18),# 6
            ("2019-3-6-sweep-hover-y.csv", 43), # 7
            ("realsense_2019_5_15_loop.csv", 10), #8
            ("circle-3s-no-gc-fix.csv", 25)#9
        ]

        self.drone_num = drone_num
        self.drone_dis = np.zeros((drone_num, drone_num))
        self.colors = matplotlib.cm.rainbow(np.linspace(0, 1, drone_num))
        self.count = 0
        self.self_id = self_id
        self.enable_detection = enable_detection
        self.use_unidentify_id = True
        self.yaw_only = yaw_only

        self.is_static = is_static

        # self.base_coor = self.drone_pos + np.random.randn(drone_num, 3)*0.2
        # print(self.base_coor)
        self.base_coor = np.array([
            [0, 0, 0], #0
            [1.48147416 , 1.7661877,   .20343478],#1
            [ 1.3077791 ,  -0.87719654,  .30700117], #2
            [0.50198334, -0.12191708,  .17340531],#3
            [0.39650833,  -.95609427, .10679965],#4

            [-1.24850392,  .58565513,  .1],
            [ 3.30543244, -1.61200742, 0],
            [  1.25038821, -2.35836745,   1.3823983 ],
            [  -1.73040171,  -5.20697205,  2.1825567 ],
            [ 1.51975686,   3.42533134,   1.74197347]])[0:drone_num]
        self.base_coor = np.random.rand(drone_num, 3) * 1.0
        self.base_coor[:, 2] = 0
        # self.base_coor = np.zeros((drone_num, 3))

        self.drone_pos = self.base_coor.copy()

        if zero_yaw_offset:
            self.base_yaw = np.zeros(drone_num)
        else:
            self.base_yaw = (np.random.randn(drone_num) * 10 + np.pi) % (2*np.pi) - np.pi
        
        self.base_yaw[self.self_id] = 0

        self.est_err_norm = []

        # print("Initial pos", self.drone_pos)
        print("Base_coor", self.base_coor)

        self.distance_noise = 0.05
        self.static = []

        self.zero_yaw_offset = zero_yaw_offset


        self.sf_pub = rospy.Publisher("/swarm_drones/swarm_frame", swarm_frame, queue_size=1)
        self.sf_pre_pub = rospy.Publisher("/swarm_drones/swarm_frame_predict", swarm_frame, queue_size=1)
        self.self_odom_pub =  rospy.Publisher("/vins_estimator/odometry", Odometry, queue_size=1)
        self.odom_pubs = [
            rospy.Publisher("/swarm_drones_sim/odom0", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom1", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom2", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom3", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom4", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom5", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom6", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom7", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom8", Odometry, queue_size=1),
            rospy.Publisher("/swarm_drones_sim/odom9", Odometry, queue_size=1)
        ]

        self.tick = 0
        self.load_datas()

        self.tstart = rospy.get_rostime()

        self.sf_sld_win = []

    def global_yaw(self, i):
        return self.data[i]["rpy"][self.tick][2] + self.base_yaw[i]

    def load_datas(self):
        self.data = []
        for p, l in self.data_paths:
            if p is None:
                self.data.append({
                    "rpy":np.zeros((10000,3)),
                    "pos":np.zeros((10000,3)),
                    "vel":np.zeros((10000,3))
                })
                continue
                self.static.append(True)
            self.static.append(False)
            self.data.append(parse_csv_data(self.data_path + p, l))

            print("{} len {}".format(p, len(self.data[-1]['pos'])))

    def generate_relpose(self, target, source, tick, noise_dir = 0.05, noise_invdep=0.1):
        """    
            double dyaw = b.yaw() - a.yaw();
            Eigen::Vector3d dp = b.position - a.position;
            p.attitude = (Eigen::Quaterniond) AngleAxisd(dyaw, Vector3d::UnitZ());

            p.position.x() = cos(-a.yaw()) * dp.x() - sin(-a.yaw()) * dp.y();
            p.position.y() = sin(-a.yaw()) * dp.x() + cos(-a.yaw()) * dp.y();
            p.position.z() = dp.z();
        """

         
        pos_target = self.drone_pos[target]
        pos_source = self.drone_pos[source]

        dp = pos_target - pos_source
        ayaw = self.global_yaw(source)
        dyaw = self.global_yaw(target) - self.global_yaw(source)
        px = math.cos(-ayaw) * dp[0] - math.sin(-ayaw) * dp[1]
        py = math.sin(-ayaw) * dp[0] + math.cos(-ayaw) * dp[1]
        pz = dp[2]

        is_in_range = False

        # if 4 > px > -3 and 4 > py > -4 and 3 > pz > -3:
        is_in_range = True

        pose = Pose()
        qx, qy, qz, qw = quaternion_from_euler(0, 0, dyaw)
        pose.orientation.w = qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        
        # Add noise individual on direction and distance

        _dir = np.array([px, py, pz])
        distance = np.linalg.norm(_dir)
        inv_dep = 1 / distance
        inv_dep = inv_dep + np.random.randn(1) * noise_invdep

        _dir = _dir / np.linalg.norm(_dir)
        _dir = _dir + np.random.randn(3)*noise_dir
        _dir = _dir / np.linalg.norm(_dir)

        pose.position.x = px + _dir[0]/inv_dep
        pose.position.y = py + _dir[1]/inv_dep
        pose.position.z = pz + _dir[2]/inv_dep

        # print("S ", source, "T ", target, "DP", px, py, pz, "dp", dp, "P0")
        
        return pose, is_in_range

    def generate_node_frame(self, i, Xii, Vii, ts, tick):
        _nf = node_frame()

        pose = Pose()
        roll = self.data[i]["rpy"][tick][0]
        pitch = self.data[i]["rpy"][tick][1]
        yaw = self.data[i]["rpy"][tick][2]

        _nf.position.x = Xii[i][0]
        _nf.position.y = Xii[i][1]
        _nf.position.z = Xii[i][2]
        
        _nf.yaw = yaw

        # odom.header.frame_id = "my_frame"
        _nf.velocity.x = Vii[i][0]
        _nf.velocity.y = Vii[i][1]
        _nf.velocity.z = Vii[i][2]

        _nf.header.stamp = ts
        if not self.static[i]:
            _nf.vo_available = True
        else:
            _nf.vo_available = False
        _nf.drone_id = i


        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, self.global_yaw(i))

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


        sd = []

        for j in range(drone_num):
            if i!=j:
                _nf.dismap_ids.append(j)
                _nf.dismap_dists.append(self.drone_dis[i][j])
                dpose, in_range = self.generate_relpose(j, i, tick)
                if in_range:
                    nd = node_detected()
                    nd.dpos = dpose.position
                    nd.dyaw = 0
                    nd.remote_drone_id = j + (i) *100
                    nd.header.stamp = ts
                    sd.append(nd)
                        # print("In range add detected node")
        if self.enable_detection:
            # print("dete")
            if len(sd) > 0:
                _nf.detected = sd
        return _nf

    def anntena_pos(self, i):
        # ann = [-0.083, 0, 0.078]
        ann = [-0.083, 0, 0.078]

        yaw = self.global_yaw(i)
        x = self.drone_pos[i][0] + math.cos(yaw) * ann[0] - math.sin(yaw) * ann[1]
        y = self.drone_pos[i][1] + math.sin(yaw) * ann[0] + math.cos(yaw) * ann[1]
        z = self.drone_pos[i][2] + ann[2]
        return x, y, z

    def update(self, dt, show=False):
        self.tick = int((rospy.get_rostime() - self.tstart).to_sec()*50)
        # print("Tick", self.tick)

        if self.is_static:
            self.tick = 0
        for i in range(self.drone_num):
            # print(self.data[i])
            self.drone_pos[i] = self.data[i]["pos"][self.tick] + self.base_coor[i]
            if not self.is_static:
                self.drone_vel[i] = self.data[i]["vel"][self.tick]
            else:
                self.drone_vel[i] = np.zeros(3)

            for j in range(self.drone_num):
                if  i!=j:
                    xi, yi, zi = self.anntena_pos(i)
                    xj, yj, zj = self.anntena_pos(j)
                    dx, dy, dz = xi - xj, yi - yj, zi - zj
                    self.drone_dis[i][j] = self.drone_dis[j][i] = np.clip(math.sqrt(dx*dx + dy*dy + dz*dz) + np.random.randn()*self.distance_noise, 0, None)
                else:
                   self.drone_dis[i][j] = self.drone_dis[j][i] = 0

        if show and self.count % 10 == 0:
            # ax.set_title(f"Time: {self.count*dt:4.2f}")
            ax = plt.subplot(111)
            ax.clear()
            ax.scatter(self.drone_pos[:,0], self.drone_pos[:,1], self.drone_pos[:,2], c=self.colors)
            ax.quiver(self.drone_pos[:,0], self.drone_pos[:,1], self.drone_pos[:,2],
                self.drone_vel[:,0], self.drone_vel[:,1], self.drone_vel[:,2])
            # fig.tight_layout(pad=0.1, w_pad=0.1, h_pad=0.1)

            plt.pause(0.1)

        Xii = self.drone_pos - self.base_coor[0:self.drone_num]
        Vii = self.drone_vel

        # for i in range(self.drone_num):
            # print("ID ", i, "P", self.drone_pos[i][0], self.drone_pos[i][1], self.drone_pos[i][2])


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
            _nf = self.generate_node_frame(i, Xii, Vii, ts, self.tick)
            sf.node_frames.append(_nf)

        self.count = self.count + 1
        self.sf_pre_pub.publish(sf)

        self.sf_sld_win.append(sf)

        if len(self.sf_sld_win) > 5:
            self.sf_pub.publish(self.sf_sld_win.pop(0) )



if __name__ == "__main__":

    import numpy, time
    seed = int(time.time())

    numpy.random.seed(seed)

    print("Starting vo data generation seed", seed)
    rospy.init_node("test_vo_datagen")
    drone_num = rospy.get_param('~drone_num', 5)
    self_id = rospy.get_param("~self_id", 0)
    enable_detection = rospy.get_param("~detection", True)
    is_static = rospy.get_param("~is_static", True)
    print("ENABLE DETECTION", enable_detection)
    env = SimulateDronesEnv(drone_num=drone_num, self_id=self_id, enable_detection=enable_detection, is_static=True)
    rate = rospy.Rate(50) # 10hz

    try:
        while not rospy.is_shutdown():
            env.update(0.02)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    