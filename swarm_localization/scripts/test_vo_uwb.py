#!/usr/bin/env python

import copy
import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as LA
import random
from scipy.optimize import minimize

matplotlib.rc('figure', figsize=(20, 15))
fig = plt.figure()
fig.tight_layout(pad=0.1, w_pad=0.1, h_pad=0.1)
ax = fig.add_subplot(111, projection='3d')

class UWBFuser(object):
    def __init__(self, self_id, drone_num=10):
        self.self_id = self_id
        self.x_dim = 4*(drone_num - 1)
        self.x_est = np.zeros(self.x_dim)
        self.x_res = np.zeros(self.x_dim)
        self.inited = False
        self.drone_num = drone_num
        self.diss_seq = []
        self.Xii_seq = []

        self.count = 0
        self.iter = 0
        self.last_lost = 0

    
    def get_Zjk_theta(self,j):
        
        if j == self.self_id:
            return np.array([0, 0, 0]), 0

        ptr = j if j < self.self_id else j - 1

        # dzjk = self.x_est[ptr*4]
        # phizjk = self.x_est[ptr*4 + 1] 
        # psizjk = self.x_est[ptr*4 + 2] 
        # z_theta = self.x_est[ptr*4 + 3]

        # zjk_x = dzjk*math.cos(phizjk)*math.cos(psizjk)
        # zjk_y = dzjk*math.cos(phizjk)*math.sin(psizjk)
        # zjk_z = dzjk*math.sin(phizjk)
        zjk_x = self.x_est[ptr*4]
        zjk_y = self.x_est[ptr*4 + 1] 
        zjk_z = self.x_est[ptr*4 + 2] 
        z_theta = self.x_est[ptr*4 + 3]
        
        # if j == 9:
            # print(j,f"ptr {ptr}!!!!!", np.array([zjk_x, zjk_y, zjk_z]))


        return np.array([zjk_x, zjk_y, zjk_z]), z_theta

    def rho_mat(self, th):
        return np.array(
            [[math.cos(th), math.sin(th), 0],
            [-math.sin(th), math.cos(th), 0],
            [0, 0, 1]])

    def get_Zji_th_ji(self, j, i):
        Zjk,th_jk = self.get_Zjk_theta(j)
        Zik,th_ik = self.get_Zjk_theta(i)

        rho_inv = np.transpose(self.rho_mat(th_ik))
        Zji = np.dot(rho_inv, (Zjk - Zik))
        return Zji, th_jk - th_ik

    def get_lost_ij(self,d_bar, i, j, Xii):
        Zji,Ztheji = self.get_Zji_th_ji(j, i)
        rho_ji = self.rho_mat(Ztheji)
        d_hat = LA.norm(Zji + np.dot(rho_ji, Xii[j]) - Xii[i])
        cost = (d_bar - d_hat)**2

        # print(f"{i}:{j}, dist:{d_bar},d_hat:{d_hat} cost{cost}")
        return cost

    def get_lost_all(self, d_bar_mat, Xii):
        lost = 0
        for i in range(self.drone_num):
            for j in range(i+1,self.drone_num):
                lost += 2*self.get_lost_ij(d_bar_mat[i][j], i, j, Xii)
                # print(f"tag1 i:{i} j:{j} lost:{self.get_lost_ij(d_bar_mat[i][j], i, j, Xii)}")
        # print("Lost!!! is ", lost)
        return lost /(self.drone_num*(self.drone_num - 1)) 

    def get_lost_all_seq_by_x(self, x):
        self.x_est = x
        lost = 0
        for Xii,diss in zip(self.Xii_seq,self.diss_seq):
            lost = lost + self.get_lost_all(diss, Xii)/ len(self.Xii_seq)
        return lost

    def init_x_by_dismat(self, dis_mat):
        for i in range(self.drone_num):
            if i == self.self_id:
                continue
            ptr = i if i < self.self_id else i - 1
            d =  dis_mat[i][self.self_id]
            self.x_res[ptr*4] = random.uniform(0.1,d)
            self.x_res[ptr*4 + 1] = random.uniform(0.1,d)
            self.x_res[ptr*4 + 2] = random.uniform(0.1,d)
            self.x_res[ptr*4 + 3] = random.uniform(0.1,d)

    def try_solve_Z_theta(self):
        # lost_all = self.get_lost_all_seq_by_x(self.x_res)
        # print(lost_all)
        print("Try to minimize")
        def cb_of_minize(xk):
            # print(xk)
            print(f"lost is {self.get_lost_all_seq_by_x(xk)}")

        ret = minimize(self.get_lost_all_seq_by_x, self.x_res,callback=cb_of_minize, options={'maxiter':200,'gtol': 1e-3})

        print("Res:", ret.x)

        self.last_lost = ret.fun
        x = self.constrain_to_x(ret.x)
        return x

    def xj_in_k(self, j, Xii):
        Zjk, Zjk_the = self.get_Zji_th_ji(j, self.self_id)
        xj = Zjk + np.dot(self.rho_mat(Zjk_the), Xii[j])
        return xj,Zjk_the

    def add_noise_to_x_now(self):
        for i in range(self.drone_num):
            if i == self.self_id:
                continue
            ptr = i if i < self.self_id else i - 1
            self.x_res[ptr*4] = self.x_res[ptr*4] + np.random.randn()*0.02
            self.x_res[ptr*4 + 1] = self.x_res[ptr*4 + 1] + np.random.randn()*0.02
            self.x_res[ptr*4 + 2] = self.x_res[ptr*4 + 2] + np.random.randn()*0.02
            self.x_res[ptr*4 + 3] = (self.x_res[ptr*4] + np.random.randn()*0.01) % (np.pi * 2)

    def constrain_to_x(self, x):
        for ptr in range(self.drone_num - 1):
            x[ptr*4 + 3] = x[ptr*4 + 3] % (np.pi * 2)
        return x

    def update_by_distances(self, dis_mat, Xii):
        self.count = self.count + 1

        if not self.inited or self.last_lost > 10:
            self.init_x_by_dismat(dis_mat)
            self.inited = True
        if self.count % 20 ==0:
            self.diss_seq.append(copy.deepcopy(dis_mat))
            self.Xii_seq.append(copy.deepcopy(Xii))

        if len(self.Xii_seq) > 5 and self.count % 50 == 0:
            print(f"{self.iter} EST  DronePos in {self.self_id} use seq {len(self.Xii_seq)}")

            self.iter = self.iter + 1
            self.x_res = self.x_est = copy.deepcopy(self.try_solve_Z_theta())
            x_ests = []
            for i in range(self.drone_num):
                Xi,th = self.xj_in_k(i, Xii)
                # yawi = self.
                # d = LA.norm
                print(f"id {i}, coorEst{Xi} th {th*57.6}")
                x_ests.append(Xi)
            return x_ests
        return None

    def verify_by_base_coor(self, base_coor_diff, base_yaw_diff, Xii,diss):
        if len(self.Xii_seq) < 10:
            return 
        #Self id must be zero here!!!!
        for i in range(1,10):
            Dx = base_coor_diff[i-1][0]
            Dy = base_coor_diff[i-1][1]
            Dz = base_coor_diff[i-1][2]
            Dtheta = base_yaw_diff[i-1]

            d = math.sqrt(Dx*Dx + Dy*Dy + Dz*Dz)

            # zjk_x = dzjk*math.cos(phizjk)*math.cos(psizjk)
            # zjk_y = dzjk*math.cos(phizjk)*math.sin(psizjk)
            # zjk_z = dzjk*math.sin(phizjk)
            phi = math.asin(Dz/d)
            psi = math.atan2(Dy, Dx)
            ptr = i-1
            self.x_est[ptr*4] = d
            self.x_est[ptr*4 + 1] = phi
            self.x_est[ptr*4 + 2] = psi
            self.x_est[ptr*4 + 3] = Dtheta
        lost = 0

        for i in range(1,10):
            Zik,Ztheik = self.get_Zji_th_ji(i, 0)
            rho_ik = self.rho_mat(Ztheik)
            d_hat = LA.norm(Zik + np.dot(rho_ik, Xii[i]) - Xii[0])
            lostik = self.get_lost_ij(diss[i][0], i, 0, Xii)

            # print(f"{i} in 0 coor {Zik + np.dot(rho_ik, Xii[i])} , Zik {Zik} BaseDiff {base_coor_diff[i-1]}")
            # print(f"d {diss[i][0]}, dhat, {d_hat} losti0 {lostik}")
        # for i in range(10):
            # print("")
            # for j in range(10):
                # lostij = self.get_lost_ij(diss[i][j], i, j, Xii)
                # print(f"{lostij:3.2f}", end="\t")
        # print("")

        for _Xii,_diss in zip(self.Xii_seq,self.diss_seq):
            lost = lost + self.get_lost_all(_diss, _Xii)/ len(self.Xii_seq)
            for i in range(self.drone_num):
                for j in range(self.drone_num):
                    pass
                    # print(f"Tag2{i} {j} {self.get_lost_ij(_diss[i][j], i, j, _Xii)}")
            
        print(f" verify lost is {lost}")




    

class SimulateDronesEnv(object):
    def __init__(self, drone_num = 10, pattern="RANDOM_WALK"):
        self.drone_pos =  np.random.randn(drone_num, 3) *20
        self.drone_vel = np.random.randn(drone_num, 3) * 5
        self.drone_num = drone_num
        self.drone_dis = np.zeros((drone_num, drone_num))
        self.colors = matplotlib.cm.rainbow(np.linspace(0, 1, drone_num))
        self.count = 0

        self.base_coor = self.drone_pos + np.random.randn(drone_num, 3)*0.2
        self.base_yaw = (np.random.randn(drone_num) * 10 + np.pi) % (2*np.pi) - np.pi
        # self.base_yaw[0] = np.pi/2 
        self.fuser = UWBFuser(0 ,drone_num=drone_num)

        self.est_err_norm = []


    def update_vel(self, dt):
        self.drone_vel = self.drone_vel + np.random.randn(self.drone_num, 3) *2* dt - self.drone_pos * 0.05*dt
        # self.drone_vel =  np.random.randn(self.drone_num, 3) * 5


    def update(self, dt=0.02, show=True):
        self.update_vel(dt)
        self.drone_pos = self.drone_pos + self.drone_vel * dt
        for i in range(self.drone_num):
            for j in range(self.drone_num):
                if  i!=j:
                    dx = self.drone_pos[i][0] - self.drone_pos[j][0]
                    dy = self.drone_pos[i][1] - self.drone_pos[j][1]
                    dz = self.drone_pos[i][2] - self.drone_pos[j][2]

                    self.drone_dis[i][j] = self.drone_dis[j][i] = np.clip(math.sqrt(dx*dx + dy*dy + dz*dz) + np.random.randn()*0.1,0,None)
                else:
                   self.drone_dis[i][j] = self.drone_dis[j][i] = 0
        if show and self.count % 10 == 0:
            ax.set_title(f"Time: {self.count*dt:4.2f}")
            ax.clear()
            ax.scatter(self.drone_pos[:,0], self.drone_pos[:,1], self.drone_pos[:,2],s=100, c=self.colors)
            ax.quiver(self.drone_pos[:,0], self.drone_pos[:,1], self.drone_pos[:,2],
                self.drone_vel[:,0], self.drone_vel[:,1], self.drone_vel[:,2])
            fig.tight_layout(pad=0.1, w_pad=0.1, h_pad=0.1)

            plt.pause(0.1)

        Xii = self.drone_pos - self.base_coor
        for i in range(self.drone_num):
            th = self.base_yaw[i]
            mat = np.array(
            [[math.cos(th), +math.sin(th), 0],
            [-math.sin(th), math.cos(th), 0],
            [0, 0, 1]])
            Xii[i] = np.dot(mat, Xii[i])

        ret = self.fuser.update_by_distances(self.drone_dis, Xii)


        if ret is not None:
            pos_in_base0 = self.drone_pos[0:self.drone_num] - self.base_coor[0]
            th = self.base_yaw[0]
            mat = np.array([[math.cos(th), +math.sin(th), 0],
                            [-math.sin(th), math.cos(th), 0],
                            [0, 0, 1]])
            for  i in range(len(pos_in_base0)):
                pos_in_base0[i] = np.dot(mat,pos_in_base0[i])
            print("Pos in base 0",pos_in_base0)
            print("Coor Yaw offset  ", self.base_yaw)

            est_err = ret - pos_in_base0
            est_err_norm = LA.norm(est_err,axis = (0,1))
            print("Pos Est err in cm", est_err*100)
            print("Est err norm", est_err_norm)
            self.est_err_norm.append(est_err_norm)
            plt.figure("EstErrNorm")
            plt.clf()
            plt.semilogy(self.est_err_norm)
            plt.grid(which="both")
            plt.pause(0.5 )


        # self.fuser.verify_by_base_coor(self.base_coor[1:10] - self.base_coor[0], np.zeros(9), Xii, self.drone_dis)
        # print("Relative pos")



        self.count = self.count + 1


if __name__ == "__main__":
    plt.ion()
    print("Hello, world!")

    env = SimulateDronesEnv(drone_num=2)

    try:
        while True:
            env.update(show=False)
    except KeyboardInterrupt:
        exit(0)
        raise