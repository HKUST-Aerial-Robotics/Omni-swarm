#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from transformations import * 
import argparse
from numpy.linalg import norm
import scipy.stats as stats
from bagparse import *
from utils import *

def plot_fused(poses, poses_fused, poses_vo, poses_path, loops, detections, nodes, groundtruth = True, \
    use_offline=False, output_path="/home/xuhao/output/", id_map = None, figsize=(6, 6), plot_each=True):
    fig = plt.figure("Traj2", figsize=figsize)
    ax = fig.add_subplot(111, projection='3d')
    ax = fig.gca(projection='3d')
    if id_map is None:
        id_map = {}
        for i in nodes:
            id_map[i] = i

    for i in nodes:
        # ax.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1],poses[i]["pos"][:,2], label=f" Traj{i}")
        ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1],poses_fused[i]["pos"][:,2], label=f"Estimate {id_map[i]}")
    
    ax.set_xlabel('$X$')
    ax.set_ylabel('$Y$')
    ax.set_zlabel('$Z$')
    
    #Plot Loops
    quivers = []
    quivers_det = []
    for loop in loops:
        posa_ = poses_fused[loop["id_a"]]["pos_func"](loop["ts_a"])
        posb_ = poses_fused[loop["id_b"]]["pos_func"](loop["ts_b"])
        if norm(loop["dpos"]) < 2.0 and norm(posb_-posa_) < 2.0:
            quivers.append([posa_[0], posa_[1], posa_[2], posb_[0]-posa_[0], posb_[1]-posa_[1], posb_[2]-posa_[2]])
            #quivers.append([posa_[0], posa_[1], posa_[2], loop["dpos"][0], loop["dpos"][1], loop["dpos"][2]])

    
    for det in detections:
        posa_ = poses_fused[det["id_a"]]["pos_func"](det["ts"])
        yawa_ = poses_fused[det["id_a"]]["ypr_func"](det["ts"])[0]
        dpos = yaw_rotate_vec(yawa_, det["dpos"])
        if np.linalg.norm(dpos) < 2:
            quivers_det.append([posa_[0], posa_[1], posa_[2], dpos[0], dpos[1], dpos[2]])
    
    quivers = np.array(quivers)
    quivers_det = np.array(quivers_det)

    # c = np.arctan2(quivers[:,4], quivers[:,3])
    # c = (c.ravel() - c.min()) / c.ptp()
    # c = np.concatenate((c, np.repeat(c, 2)))
    # c = plt.cm.hsv(c)

    step = 1
    if len(quivers) > 0:   
        ax.quiver(quivers[::step,0], quivers[::step,1], quivers[::step,2], quivers[::step,3], quivers[::step,4], quivers[::step,5], 
            arrow_length_ratio=0.1, color="black",linewidths=1.0, label="Map-based Mea.")

    step_det = 5
    if len(quivers_det) > 0:   
        ax.quiver(quivers_det[::step_det,0], quivers_det[::step_det,1], quivers_det[::step_det,2], quivers_det[::step_det,3],
            quivers_det[::step_det,4], quivers_det[::step_det,5], 
            arrow_length_ratio=0.1, color="gray",linewidths=1.0, label="Vis. Mea.")

    plt.legend()
    plt.savefig(output_path+"Traj2.pdf")

    #Plot Fused Vs GT 3D
    fig = plt.figure("FusedVsGT3D")
    # fig.suptitle("Fused Vs GT 3D")
    for k in range(len(nodes)):
        i = nodes[k]
        _id = id_map[i]

        ax = fig.add_subplot(1, len(nodes), k+1, projection='3d')
        ax.set_title(f"Traj {_id}, length: {poses_length(poses_fused[i]):3.3f}")
        if groundtruth:
            ax.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1],poses[i]["pos"][:,2], label=f"Ground Truth ${_id}$")
        ax.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1],poses_vo[i]["pos"][:,2], label=f"Aligned VIO ${_id}$")
        ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1],poses_fused[i]["pos"][:,2], label=f"Estimate ${_id}$")
        
        plt.legend()
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')
    plt.savefig(output_path+"FusedVsGT3D.pdf")

    fig = plt.figure("Fused Multi 2d", figsize=figsize)
    plt.gca().set_aspect('equal')
    
    step_det = 1
    qview_width = 0.9
    if len(quivers) > 0:   
        for i in range(0,len(quivers),step_det):
            xs = [quivers[i,0],quivers[i,0]+quivers[i,3]]
            ys = [quivers[i,1], quivers[i,1]+quivers[i,4]]
            if i == 0:
                plt.plot(xs, ys, color="black", label="Map-based Mea.", linewidth=qview_width)
            else:
                plt.plot(xs, ys, color="black", linewidth=qview_width)
    step_det = 1
    if len(quivers_det) > 0: 
        for i in range(0,len(quivers_det),step_det):
            xs = [quivers_det[i,0],quivers_det[i,0]+quivers_det[i,3]]
            ys = [quivers_det[i,1], quivers_det[i,1]+quivers_det[i,4]]
            if i == 0:
                plt.plot(xs, ys, color="gray", label="Vis. Mea.", linewidth=qview_width)
            else:
                plt.plot(xs, ys, color="gray", linewidth=qview_width)
    for i in nodes:
        _id = id_map[i]

        if use_offline:
            plt.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], label=f"Final {_id}")
        plt.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Real-time {_id}")

    for i in nodes:
        _id = id_map[i]
        final_vio = norm(poses_vo[i]["pos"][-1,:])
        if use_offline:
            final_path = norm(poses_path[i]["pos"][-1,:])
        else:
            final_path = norm(poses_fused[i]["pos"][-1,:])
            
        total_len = poses_length(poses_fused[i])
        print(f"Final drift {i} VIO {final_vio:3.2f}m {final_vio/total_len*100:3.1f}% Fused {final_path:3.2f}m {final_path/total_len*100:3.1f}% total_len {total_len:.1f}m")
    plt.ylabel('$Y$')
    plt.xlabel('$X$')
    plt.legend()
    plt.grid()
    plt.savefig(output_path+"fused2d.pdf")

    for k in range(len(nodes)):
        # fig.suptitle("Fused Vs GT 2D")
        i = nodes[k]
        _id = id_map[i]

        fig = plt.figure(f"Fused Vs GT 2D {i}", figsize=figsize)
        plt.gca().set_aspect('equal', adjustable="datalim", anchor="SE")

        if groundtruth:
            plt.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1], label=f"Ground Truth {_id}")
        plt.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1], label=f"VIO {_id}")
        # plt.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")
        if i in poses_path and use_offline:
            plt.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], label=f"Final {_id}")
        plt.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Estimate {_id}")
        plt.grid()
        plt.ylabel('$Y$')
        plt.xlabel('$X$')
        plt.legend()

        plt.savefig(output_path+f"fusedvsgt2d_{i}.pdf")
    if not plot_each:
        return
    for i in nodes:
        _id = id_map[i]
        fig = plt.figure(f"Drone {i} fused Vs GT Pos", figsize=figsize)
        #fig.suptitle(f"Drone {i} fused Vs GT 1D")
        ax1, ax2, ax3 = fig.subplots(3, 1)

        t_ = poses_fused[i]["t"]
        if groundtruth:
            pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"])
            yaw_gt =  poses[i]["ypr_func"](poses_fused[i]["t"])[:, 0]
            pitch_gt =  poses[i]["ypr_func"](poses_fused[i]["t"])[:, 1]
            roll_gt =  poses[i]["ypr_func"](poses_fused[i]["t"])[:, 2]
        if groundtruth:
            ax1.plot(t_, pos_gt[:,0], label=f"Ground Truth ${i}$")
        ax1.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,0], label=f"Aligned VO Traj{_id}")
        ax1.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,0], label=f"Estimate {_id}")
        if use_offline:
            ax1.plot(poses_path[i]["t"], poses_path[i]["pos"][:,0], '.', label=f"Fused Offline Traj{i}")
        ax1.tick_params( axis='x', which='both', bottom=False, top=False, labelbottom=False) 
        ax1.set_ylabel("x")

        if groundtruth:
            ax2.plot(t_, pos_gt[:,1], label=f"Ground Truth ${i}$")
        ax2.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,1], label=f"Aligned VO Traj{_id}")
        ax2.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,1], label=f"Estimate {_id}")
        if use_offline:
            ax2.plot(poses_path[i]["t"], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")
        ax2.tick_params( axis='x', which='both', bottom=False, top=False, labelbottom=False) 
        ax2.set_ylabel("y")

        if groundtruth:
            ax3.plot(t_, pos_gt[:,2], label=f"Ground Truth ${i}$")
        ax3.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,2], label=f"Aligned VIO ${_id}$")
        ax3.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,2], label=f"Estimate {_id}")
        if use_offline:
            ax3.plot(poses_path[i]["t"], poses_path[i]["pos"][:,2], '.', label=f"Fused Offline Traj{i}")
        ax3.set_ylabel("z")
        ax3.set_xlabel("t")

        # ax1.legend()
        # ax2.legend()
        ax3.legend()
        ax1.grid()
        ax2.grid()
        ax3.grid()
        plt.savefig(output_path+f"est_by_t{i}_position.png")

        fig = plt.figure(f"Drone {i} fused Vs GT Att", figsize=figsize)
        ax1, ax2, ax3 = fig.subplots(3, 1)

        if groundtruth:
            ax1.plot(t_, yaw_gt*57.3, label=f"Ground Truth ${i}$")
        ax1.plot(poses_vo[i]["t"], poses_vo[i]["ypr"][:,0]*57.3, label=f"Aligned VIO ${_id}$")
        ax1.plot(poses_fused[i]["t"], poses_fused[i]["ypr"][:,0]*57.3, label=f"Estimate {_id}")
        if use_offline:
            ax1.plot(poses_path[i]["t"], poses_path[i]["ypr"][:,0]*57.3, '.', label=f"EstKF {_id}")

        ax1.set_ylabel("Yaw (deg)")
        ax1.set_xlabel("t")
        ax1.legend()
        ax1.grid()
        
        if groundtruth:
            ax2.plot(t_, pitch_gt*57.3, label=f"Ground Truth ${i}$")
        ax2.plot(poses_vo[i]["t"], poses_vo[i]["ypr"][:,1]*57.3, label=f"Aligned VIO ${_id}$")
        ax2.plot(poses_fused[i]["t"], poses_fused[i]["ypr"][:,1]*57.3, label=f"Estimate {_id}")
        if use_offline:
            ax2.plot(poses_path[i]["t"], poses_path[i]["ypr"][:,1]*57.3, '.', label=f"EstKF {_id}")

        ax2.set_ylabel("Pitch (deg)")
        ax2.set_xlabel("t")
        ax2.legend()
        ax2.grid()

        if groundtruth:
            ax3.plot(t_, roll_gt*57.3, label=f"Ground Truth ${i}$")
        ax3.plot(poses_vo[i]["t"], poses_vo[i]["ypr"][:,2]*57.3, label=f"Aligned VIO ${_id}$")
        ax3.plot(poses_fused[i]["t"], poses_fused[i]["ypr"][:,2]*57.3, label=f"Estimate {_id}")
        if use_offline:
            ax3.plot(poses_path[i]["t"], poses_path[i]["ypr"][:,2]*57.3, '.', label=f"EstKF {_id}")
        ax3.set_xlabel("t")
        ax3.legend()
        ax3.grid()
        plt.savefig(output_path+f"est_by_t{i}_attitude.png")

def plot_distance_err(poses, poses_fused, distances, main_id, nodes, calib = {}, is_show=False):
    for main_id in nodes:
        for i in nodes:
            if i == main_id:
                continue
            t_ = np.array(distances[i][main_id]["t"])
            mask = t_ > poses_fused[i]["t"][0]
            t_ = t_[mask]
            pos_gt = poses[i]["pos_func"](t_)
            pos_fused = poses_fused[i]["pos_func"](t_)

            main_pos_gt = poses[main_id]["pos_func"](t_)
            main_pos_fused = poses_fused[main_id]["pos_func"](t_)

            #pos_vo = poses_vo[i]["pos"]
            #pos_path = poses_path[i]["pos"](t_)
            
            dis_raw = np.array(distances[i][main_id]["dis"])[mask]
            dis_gt = norm(pos_gt  - main_pos_gt, axis=1)
            dis_fused = norm(pos_fused  - main_pos_fused, axis=1)
            #dis_vo = norm(pos_path  - main_pos_path, axis=0)

            if is_show:
                fig = plt.figure(f"Distance {i}->{main_id}")
                fig.suptitle(f"Distance {i}->{main_id}")
                ax1, ax2, ax3 = fig.subplots(3, 1)

                ax1.plot(t_, dis_gt, label="Distance GT")
                ax1.plot(t_, dis_fused,label="Distance Fused")
                ax1.plot(t_, dis_raw, ".", label="Distance UWB")


                ax2.plot(t_, dis_gt-dis_raw, ".", label="Distance Error of UWB")
                ax2.plot(t_, dis_gt-dis_fused, label="Distance Error of Fused")

                ax3.plot(dis_gt, dis_raw,'+',label="GT VS UWB")
                ax1.legend()
                ax1.grid()
                
                ax2.legend()
                ax2.grid()
                
                ax3.legend()
                ax3.grid()
                plt.show()


            print(f"Distance {i}->{main_id} RMSE {RMSE(dis_raw, dis_gt)}")
            # z = np.polyfit(dis_gt, dis_raw, 1)
            if i not in calib:
                z = np.polyfit(dis_raw, dis_gt, 1)
                print(f"Fit {z[0]}, {z[1]}")
            else:
                z = calib[i]
                print(f"Use fit {z[0]}, {z[1]}")

            dis_calibed = z[1]+z[0]*np.array(dis_raw)
            err_calibed_filter = np.fabs(dis_gt-dis_calibed) < 1.0
            err_calibed = (dis_gt-dis_calibed)[err_calibed_filter]
            print("Cov Fitted", np.cov(err_calibed))
            print("Cov Raw", np.cov(dis_gt-dis_raw))

            mu, std = stats.norm.fit(err_calibed)
            
            if is_show:
                plt.figure(f"Dist Hist Raw {i}->{main_id}")
                plt.hist(dis_gt-dis_raw, 50, (-0.5, 0.5), density=True, facecolor='g', alpha=0.75)
                xmin, xmax = plt.xlim()
                x = np.linspace(xmin, xmax, 100)
                mu, std = stats.norm.fit(dis_gt-dis_raw)
                p = stats.norm.pdf(x, mu, std)
                plt.plot(x, p, 'k', linewidth=2)
                title = "raw mu = %.2f,  std = %.2f" % (mu, std)
                plt.title(title)
                plt.show()

                plt.figure(f"Dist Hist err_calibed {i}->{main_id}")
                plt.hist(err_calibed, 50, (-0.5, 0.5), density=True, facecolor='g', alpha=0.75)
                xmin, xmax = plt.xlim()
                mu, std = stats.norm.fit(err_calibed)
                x = np.linspace(xmin, xmax, 100)
                p = stats.norm.pdf(x, mu, std)
                plt.plot(x, p, 'k', linewidth=2)
                title = "Calibed mu = %.2f,  std = %.2f" % (mu, std)
                plt.title(title)
                plt.show()
        

def plot_relative_pose_err(poses, poses_fused, poses_vo, main_id, target_ids, outlier_thres=100, dte=1000000, 
    groundtruth = True, show=True, figsize=(6, 6), verbose=True):
    ts = poses_fused[main_id]["t"]
    ts = ts[ts<dte]
    posa_vo =  poses_vo[main_id]["pos_func"](ts)
    posa_fused = poses_fused[main_id]["pos_func"](ts)
    yawa_fused = poses_fused[main_id]["ypr_func"](ts)[:,0]
    yawa_vo = poses_vo[main_id]["ypr_func"](ts)[:,0]
    if verbose:
        print("Relative Trajectory Statistics\nEST RMSE:\t\tPOS\t\tYAW\t|\tBIAS: POS\t\t\tYAW\t|VO\tRMSE:\tPOS\t\tYAW")
    avg_rmse = 0
    avg_rmse_yaw = 0.0
    
    avg_rmse_vo = 0
    avg_rmse_vo_yaw = 0.0

    for target_id in target_ids:
        posb_vo =  poses_vo[target_id]["pos_func"](ts)
        posb_fused = poses_fused[target_id]["pos_func"](ts)
        yawb_fused = poses_fused[target_id]["ypr_func"](ts)[:, 0]
        yawb_vo = poses_vo[target_id]["ypr_func"](ts)[:, 0]
        
        dp_fused = posb_fused - posa_fused
        dyaw_fused = wrap_pi(yawb_fused - yawa_fused)
        dyaw_vo = wrap_pi(yawb_vo - yawa_vo)
        dp_vo = posb_vo - posa_vo
        if groundtruth:
            posa_gt =  poses[main_id]["pos_func"](ts)
            yawa_gt = poses[main_id]["ypr_func"](ts)[:,0]
            posb_gt =  poses[target_id]["pos_func"](ts)
            yawb_gt = poses[target_id]["ypr_func"](ts)[:,0]
            dp_gt = posb_gt - posa_gt
            dyaw_gt = wrap_pi(yawb_gt - yawa_gt)
        
        for i in range(len(yawa_fused)):
            yaw = yawa_fused[i]
            dp_fused[i] = yaw_rotate_vec(-yaw, dp_fused[i])

        if groundtruth:
            for i in range(len(yawa_fused)):
                yaw = yawa_gt[i]
                dp_gt[i] = yaw_rotate_vec(-yaw, dp_gt[i])

        for i in range(len(yawa_vo)):
            yaw = yawa_vo[i]
            dp_vo[i] = yaw_rotate_vec(-yaw, dp_vo[i])
        mask = np.linalg.norm(dp_gt - dp_vo, axis=1) < outlier_thres
        if groundtruth:
            rmse_yaw = RMSE(wrap_pi(yawb_fused - yawa_fused - yawb_gt + yawa_gt), 0)
            rmse_x = RMSE(dp_gt[mask,0] , dp_fused[mask,0])
            rmse_y = RMSE(dp_gt[mask,1] , dp_fused[mask,1])
            rmse_z = RMSE(dp_gt[mask,2] , dp_fused[mask,2])

            avg_rmse += ATE_POS(dp_gt[mask], dp_fused[mask])
            avg_rmse_yaw += rmse_yaw

            if verbose:
                #ERROR
                print(f"{main_id}->{target_id}\t{rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}\t{rmse_yaw*180/pi:3.2f}°", end="\t|")
                #BIAS
                print(f"{np.mean(dp_gt[mask,0] - dp_fused[mask,0]):3.3f},{np.mean(dp_gt[mask,1] - dp_fused[mask,1]):+3.3f},{np.mean(dp_gt[mask,2] - dp_fused[mask,2]):+3.3f}\t{np.mean(dyaw_gt - dyaw_fused)*180/3.14:+3.2f}°",end="\t")

            rmse_yaw = RMSE(wrap_pi(yawb_vo - yawa_vo - yawb_gt + yawa_gt), 0)
            rmse_x = RMSE(dp_gt[mask,0] , dp_vo[mask,0])
            rmse_y = RMSE(dp_gt[mask,1] , dp_vo[mask,1])
            rmse_z = RMSE(dp_gt[mask,2] , dp_vo[mask,2])

            avg_rmse_vo += ATE_POS(dp_gt[mask], dp_vo[mask])
            avg_rmse_vo_yaw += rmse_yaw

            if verbose:
                print(f"|\t{rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}\t{rmse_yaw*180/pi:3.1f}°")
    

        if show:
            fig = plt.figure(f"Relative Pose 2D {main_id}->{target_ids}", figsize=figsize)

            if groundtruth:
                plt.plot(dp_gt[:, 0], dp_gt[:, 1], label=f"Relative Pose GT {main_id}->{target_id}")
            plt.plot(dp_fused[:, 0], dp_fused[:, 1], label=f"Relative Pose EST {main_id}->{target_id}")
            # plt.plot(dp_vo[:, 0], dp_vo[:, 1], label=f"Relative Pose VO {main_id}->{target_id}")
            plt.legend()
            if target_id == target_ids[0]:
                plt.grid()


            fig = plt.figure("Relative Pose PolarErr", figsize=figsize)
            fig.suptitle("Relative Pose PolarErr")
            ax1, ax2 = fig.subplots(2, 1)

            if groundtruth:
                ax1.plot(ts[mask], wrap_pi(np.arctan2(dp_gt[:, 0], dp_gt[:, 1]) - np.arctan2(dp_fused[:, 0], dp_fused[:, 1]))[mask], label=f"Relative Pose Angular Err {main_id}->{target_id}")

            if groundtruth:
                ax2.plot(ts[mask], (norm(dp_gt, axis=1) - norm(dp_fused, axis=1))[mask], label=f"Relative Pose Length Err {main_id}->{target_id}")


            ax1.legend()
            ax1.grid()
            ax2.legend()
            ax2.grid()
            plt.tight_layout()

            fig = plt.figure("Relative Pose", figsize=figsize)
            fig.suptitle(f"Relative Pose {main_id}->{target_ids}")
            ax1, ax2, ax3, ax4 = fig.subplots(4, 1)

            if groundtruth:
                ax1.plot(ts, dp_gt[:,0], label="$X_{gt}^" + str(target_id) + "$")
                ax2.plot(ts, dp_gt[:,1], label="$Y_{gt}^" + str(target_id) + "$")
                ax3.plot(ts, dp_gt[:,2], label="$Z_{gt}^" + str(target_id) + "$")
                ax4.plot(ts, wrap_pi(dyaw_gt), label="$Yaw_{gt}^" + str(target_id) + "$")

            ax1.plot(ts, dp_fused[:,0], label="$X_{fused}^" + str(target_id) + "$")
            ax2.plot(ts, dp_fused[:,1], label="$Y_{fused}^" + str(target_id) + "$")
            ax3.plot(ts, dp_fused[:,2], label="$Z_{fused}^" + str(target_id) + "$")
            ax4.plot(ts, wrap_pi(dyaw_fused), label="$Yaw_{fused}^" + str(target_id) + "$")
            
            ax1.plot(ts, dp_vo[:,0], label="$X_{vo}^" + str(target_id) + "$")
            ax2.plot(ts, dp_vo[:,1], label="$Y_{vo}^" + str(target_id) + "$")
            ax3.plot(ts, dp_vo[:,2], label="$Z_{vo}^" + str(target_id) + "$")
            ax4.plot(ts, wrap_pi(dyaw_vo), label="$Yaw_{vo}^" + str(target_id) + "$")

            ax1.legend()
            ax2.legend()
            ax3.legend()
            ax4.legend()
            ax1.grid()
            ax2.grid()
            ax3.grid()
            ax4.grid()
            plt.tight_layout()
                
            fig = plt.figure("Fused Relative Error", figsize=figsize)
            fig.suptitle(f"Fused Relative Error {main_id}->{target_ids}")
            ax1, ax2, ax3 = fig.subplots(3, 1)

            ax1.plot(ts[mask], dp_gt[mask,0] - dp_fused[mask,0], label="$E_{xfused}^" + str(target_id) + f"$ RMSE:{rmse_x:3.3f}")
            ax2.plot(ts[mask], dp_gt[mask,1] - dp_fused[mask,1], label="$E_{yfused}^" + str(target_id) + f"$ RMSE:{rmse_y:3.3f}")
            ax3.plot(ts[mask], dp_gt[mask,2] - dp_fused[mask,2], label="$E_{zfused}^" + str(target_id) + f"$ RMSE:{rmse_z:3.3f}")
            ax4.plot(ts[mask], wrap_pi(dyaw_gt[mask] - dyaw_fused[mask]), label="$E_{yawfused}^" + str(target_id) + f"$ RMSE:{rmse_z:3.3f}")

            ax1.legend()
            ax2.legend()
            ax3.legend()
            ax1.grid()
            ax2.grid()
            ax3.grid()
            plt.tight_layout()

    if show:
        plt.show()

    return avg_rmse/len(target_ids), avg_rmse_yaw/len(target_ids), avg_rmse_vo/len(target_ids), avg_rmse_vo_yaw/len(target_ids)

def plot_fused_err(poses, poses_fused, poses_vo, poses_path, nodes, main_id=1,dte=100000,show=True, outlier_thres=100, verbose=True):
    #Plot Fused Vs GT absolute error
    ate_vo_sum = 0
    rmse_vo_ang_sum = 0

    ate_fused_sum = 0
    rmse_fused_ang_sum = 0
    if verbose:
        print("""Absolute Trajectory Statistics\n\
EST:\tATE P\tAng\tYaw\tPitch\tRoll\t\tRMSE\t\t\t\
COV/m\t\tPOS\t\t\tYAW\t\t\tKF:\tATE P\tAng\t|\
\tVO:ATE_P\tAng\tYaw\tPitch\tRoll\t\t\tATE\t\t\tCOV/m\tPOS\t\tYAW\t""")
    for i in nodes:
        t_ = poses_fused[i]["t"]
        mask = t_<dte
        t_ = t_[t_<dte]
        pos_gt =  poses[i]["pos_func"](t_)
        pos_fused = poses_fused[i]["pos"][mask]
        ypr_fused = poses_fused[i]["ypr"][mask]
        pos_vo = poses_vo[i]["pos"]
        ypr_gt = poses[i]["ypr_func"](t_)
        ypr_vo = poses_vo[i]["ypr"]
        
        mask_fused = np.linalg.norm(pos_fused - pos_gt, axis=1) < outlier_thres
        pos_gt =  pos_gt[mask_fused]
        ypr_gt = ypr_gt[mask_fused]
        pos_fused = pos_fused[mask]
        ypr_fused = ypr_fused[mask]

        _i = str(i) 

        fused_cov_per_meter, fused_yaw_cov_per_meter = odometry_covariance_per_meter(pos_fused, ypr_fused[:,0], pos_gt, ypr_gt[:,0])
        rmse_x = RMSE(pos_fused[:,0] , pos_gt[:,0])
        rmse_y = RMSE(pos_fused[:,1] , pos_gt[:,1])
        rmse_z = RMSE(pos_fused[:,2] , pos_gt[:,2])

        fused_cov_x = fused_cov_per_meter[0][0]
        fused_cov_y = fused_cov_per_meter[1][1]
        fused_cov_z = fused_cov_per_meter[2][2]

        if np.isnan(pos_fused).any():
            print("pos_fused has nan")
        if np.isnan(pos_gt).any():
            print("pos_gt has nan")
        
        ate_fused = ATE_POS(pos_fused, pos_gt)
        rmse_yaw_fused = RMSE(wrap_pi(ypr_gt[:,0]-ypr_fused[:,0]), 0)
        rmse_pitch_fused = RMSE(wrap_pi(ypr_gt[:,1]-ypr_fused[:,1]), 0)
        rmse_roll_fused = RMSE(wrap_pi(ypr_gt[:,2]-ypr_fused[:,2]), 0)
        rmse_angular_fused = RMSE(angular_error_ypr_array(ypr_gt, ypr_fused), 0)

        ate_fused_sum += ate_fused
        
        rmse_fused_ang_sum += rmse_angular_fused
        
        pos_path_gt =  poses[i]["pos_func"](poses_path[i]["t"])
        pos_path = poses_path[i]["pos"]
        ypr_path_gt =  poses[i]["ypr_func"](poses_path[i]["t"])
        ypr_path = poses_path[i]["ypr"]
        mask_path = np.linalg.norm(pos_path_gt - pos_path, axis=1) < outlier_thres
        t_path = poses_path[i]["t"][mask_path]
        pos_path_gt, pos_path, ypr_path_gt, ypr_path = pos_path_gt[mask_path], pos_path[mask_path], ypr_path_gt[mask_path], ypr_path[mask_path]

        ate_path = ATE_POS(pos_path, pos_path_gt)
        rmse_angular_path = RMSE(angular_error_ypr_array(ypr_path_gt, ypr_path), 0)

        pos_gt_vo=  poses[i]["pos_func"](poses_vo[i]["t"])
        ypr_gt_vo =  poses[i]["ypr_func"](poses_vo[i]["t"])


        mask_vo = np.linalg.norm(pos_gt_vo - pos_vo, axis=1) < outlier_thres
        pos_vo = pos_vo[mask_vo]
        pos_gt_vo = pos_gt_vo[mask_vo]
        ypr_vo = ypr_vo[mask_vo]
        ypr_gt_vo = ypr_gt_vo[mask_vo]
        t_vo = poses_vo[i]["t"][mask_vo]

        rmse_vo_x = RMSE(pos_vo[:,0] , pos_gt_vo[:,0])
        rmse_vo_y = RMSE(pos_vo[:,1] , pos_gt_vo[:,1])
        rmse_vo_z = RMSE(pos_vo[:,2] , pos_gt_vo[:,2])

        vo_cov_per_meter, vo_yaw_cov_per_meter = odometry_covariance_per_meter(pos_vo, ypr_vo[:,0], pos_gt_vo, ypr_gt_vo[:,0], show=False)
        
        ate_vo = ATE_POS(pos_vo, pos_gt_vo)
        rmse_yaw_vo = RMSE(wrap_pi(ypr_vo[:,0]-ypr_gt_vo[:,0]), 0)
        rmse_pitch_vo = RMSE(wrap_pi(ypr_vo[:,1]-ypr_gt_vo[:,1]), 0)
        rmse_roll_vo = RMSE(wrap_pi(ypr_vo[:,2]-ypr_gt_vo[:,2]), 0)
        rmse_angular_vo = RMSE(angular_error_ypr_array(ypr_vo, ypr_gt_vo), 0)

        ate_vo_sum += ate_vo
        rmse_vo_ang_sum += rmse_angular_vo

        if verbose:
            if i == main_id:
                print(f"Ego{main_id}",end="\t")
            else:
                print(f"{i}by{main_id}",end="\t")
            print(f"{ate_fused:3.3f}\t{rmse_angular_fused*180/pi:3.3f}°\t{rmse_yaw_fused*180/pi:3.3f}°\t{rmse_pitch_fused*180/pi:3.3f}°\t{rmse_roll_fused*180/pi:3.3f}°\t{rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}\t{fused_cov_x:.1e},{fused_cov_y:.1e},{fused_cov_z:.1e}\t{fused_yaw_cov_per_meter:.1e}rad/m\t\t{ate_path:3.3f}\t{rmse_angular_path*180/pi:3.3f}°\t|\t",end="")

            print(f"{ate_vo:3.3f}\t{rmse_angular_vo*180/pi:3.3f}°\t{rmse_yaw_vo*180/pi:3.3f}°\t{rmse_pitch_vo*180/pi:3.3f}°\t{rmse_roll_vo*180/pi:3.3f}°\t{rmse_vo_x:3.3f},{rmse_vo_y:3.3f},{rmse_vo_z:3.3f}\t{vo_cov_per_meter[0][0]:.1e},{vo_cov_per_meter[1][1]:.1e},{vo_cov_per_meter[2][2]:.1e}\t{vo_yaw_cov_per_meter:.1e}rad")

        # print("VO COV POS\n", vo_cov_per_meter, 'yaw', vo_yaw_cov_per_meter)

        if show:
            fig = plt.figure(f"Fused Absolute Error Pos {i}")
            fig.suptitle(f"Fused Absolute Error Pos {i}")
            ax1, ax2, ax3 = fig.subplots(3, 1)
            label = f"$errx_{i}$ RMSE{i}:{rmse_x:3.3f}"
            ax1.plot(t_, pos_gt[:,0]  - pos_fused[:,0], label=label)

            label = f"$erry_{i}$ RMSE{i}:{rmse_y:3.3f}"
            ax2.plot(t_, pos_gt[:,1]  - pos_fused[:,1], label=label)

            label = f"$erry_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax3.plot(t_,  pos_gt[:,2]  - pos_fused[:,2], label=label)


            label = f"$VO errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
            ax1.plot(t_vo, pos_gt_vo[:,0]  - pos_vo[:,0], label=label)

            label = f"$VO erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
            ax2.plot(t_vo, pos_gt_vo[:,1]  - pos_vo[:,1], label=label)
            
            label = f"$VO errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
            ax3.plot(t_vo, pos_gt_vo[:,2]  - pos_vo[:,2], label=label)

            label = f"$KF errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
            ax1.plot(t_path, pos_path_gt[:,0]  - pos_path[:,0], label=label)

            label = f"$KF erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
            ax2.plot(t_path, pos_path_gt[:,1]  - pos_path[:,1], label=label)
            
            label = f"$KF errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
            ax3.plot(t_path, pos_path_gt[:,2]  - pos_path[:,2], label=label)

            ax1.legend()
            ax2.legend()
            ax3.legend()
            ax1.grid()
            ax2.grid()
            ax3.grid()

            fig = plt.figure(f"Fused Absolute Error Att {i}")
            fig.suptitle(f"Fused Absolute Error Att {i}")
            ax1, ax2, ax3 = fig.subplots(3, 1)
            label = f"$VO yaw_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax1.plot(t_vo, wrap_pi(ypr_gt_vo[:,0]-ypr_vo[:,0]), label=label)
            label = f"$yaw_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax1.plot(t_, wrap_pi(ypr_gt[:,0]-ypr_fused[:,0]), label=label)

            label = f"$VO pitch_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax2.plot(t_vo, wrap_pi(ypr_gt_vo[:,1]-ypr_vo[:,1]), label=label)
            label = f"$pitch_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax2.plot(t_, wrap_pi(ypr_gt[:,1]-ypr_fused[:,1]), label=label)

            label = f"$VO roll_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax3.plot(t_vo, wrap_pi(ypr_gt_vo[:,2]-ypr_vo[:,2]), label=label)
            label = f"$roll_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax3.plot(t_, wrap_pi(ypr_gt[:,2]-ypr_fused[:,2]), label=label)

            # ax1.legend()
            # ax2.legend()


            label = f"$Path yaw_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
            ax1.plot(t_path, wrap_pi(ypr_path_gt[:,0]  - ypr_path[:,0]), ".", label=label)

            label = f"$Path pitch_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
            ax2.plot(t_path, wrap_pi(ypr_path_gt[:,1]  - ypr_path[:,1]), ".", label=label)
            
            label = f"$Path roll_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
            ax3.plot(t_path, wrap_pi(ypr_path_gt[:,2]  - ypr_path[:,2]), ".", label=label)
            ax1.legend()
            ax2.legend()
            ax3.legend()
            ax1.grid()
            ax2.grid()
            ax3.grid()

        
        num = len(nodes)
    if verbose:
        print(f"Avg\t{ate_fused_sum/num:3.3f}\t{rmse_yaw_fused*180/pi/num:3.3f}°\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t|\t{ate_vo_sum/num:3.3f}\t\t{rmse_yaw_vo/num*180/pi:3.3f}°")
    return ate_fused_sum/num, rmse_fused_ang_sum/num, ate_vo_sum/num, rmse_vo_ang_sum/num


def plot_detections_error(poses, poses_vo, detections, main_id, enable_dpose, inlier_file=""):
    _dets_data = []
    dpos_dets = []
    dpos_gts = []
    dpos_gt_norms= []
    dpos_det_norms= []
    dpos_errs = []
    inv_dep_errs = []
    inv_deps = []
    dpos_errs_norm = []
    posa_gts = []
    ts_a = []
    dyaws = []
    yawa_gts = []
    self_pos_a = []
    self_pos_b = []
    inv_deps_gt = []
    det_ids = []
    print("Total detection", len(detections))
    CG = np.array([-0.06, 0, 0])
    good_det_id = set()
    if inlier_file != "":
        with open(inlier_file, "r") as f:
            lines = f.readlines()
            for line in lines:
                good_det_id.add(int(line))


    for det in detections:
        if det["id_a"] != main_id:
            continue
        yawa_gt = poses[det["id_a"]]["ypr_func"](det["ts"])[0]
        yawb_gt = poses[det["id_b"]]["ypr_func"](det["ts"])[0]

        posa_gt = poses[det["id_a"]]["pos_func"](det["ts"])
        posb_gt = poses[det["id_b"]]["pos_func"](det["ts"]) # + yaw_rotate_vec(yawb_gt, np.array([-0.04, 0, 0.02]))

        if enable_dpose:
            # dpose_self_a = yaw_rotate_vec(yawa_gt, yaw_rotate_vec(-yawa_vo, posa_vo - det["pos_a"]))
            posa_gt = posa_gt + yaw_rotate_vec(yawa_gt, det["extrinsic"])
            posb_gt = posb_gt + yaw_rotate_vec(yawb_gt, CG)

        dpos_gt = yaw_rotate_vec(-yawa_gt, posb_gt - posa_gt)
        inv_dep_gt = 1/norm(dpos_gt)
        dpos_gt = dpos_gt * inv_dep_gt
        
        dpos_det = np.array(det["dpos"])
        inv_dep_det = det["inv_dep"]
        _dets_data.append({
            "dpos_det": dpos_det,
            "dpos_gt": dpos_gt,
            "dpos_err": dpos_gt - dpos_det,
            "inv_dep_err": inv_dep_gt - inv_dep_det
            })
        #TMP
        inv_deps.append(det["inv_dep"])
        self_pos_a.append(det["pos_a"])
        self_pos_b.append(det["pos_b"])

        inv_dep_errs.append(inv_dep_gt - inv_dep_det)
        inv_deps_gt.append(inv_dep_gt)
        dpos_dets.append(dpos_det)
        dpos_gts.append(dpos_gt)
        dpos_errs.append(dpos_gt - dpos_det)    
        dpos_gt_norms.append(norm(dpos_gt))
        dpos_det_norms.append(norm(dpos_det))
        dpos_errs_norm.append(norm(dpos_gt - dpos_det))
        posa_gts.append(posa_gt)
        ts_a.append(det["ts"])
        yawa_gts.append(yawa_gt)
        det_ids.append(det["id"])

        # pa = det['pos_a']
        # pb = det["pos_b"]
        # dp = det["dpos"]
        # print(f"Det {det['id_a']} -> {det['id_b']}")
        # print(f"SELF POSE A {pa} B {pb} p {dp}")
        # print(f"POSE A {posa_gt} B {posb_gt} PB-PA {posb_gt-posa_gt}")
        # print(f"det dp {dpos_det} est dp{dpos_gt} yawa {yawa_gt*57.3}deg")
        
    posa_gts = np.array(posa_gts)
    dpos_errs = np.array(dpos_errs)
    dpos_gts = np.array(dpos_gts)
    dpos_dets = np.array(dpos_dets)
    self_pos_a = np.array(self_pos_a)
    self_pos_b = np.array(self_pos_b)
    ts_a = np.array(ts_a)
    fig = plt.figure()

    plt.subplot(311)
    plt.plot(ts_a, dpos_errs_norm, '.', label="Err NORM")
    plt.plot(ts_a, np.abs(dpos_errs[:,0]), '+', label="ERR X")
    plt.plot(ts_a, np.abs(dpos_errs[:,1]), '+', label="ERR Y")
    plt.plot(ts_a, dpos_errs[:,2], '+', label="ERR Z")
    # plt.ylim((-0.2, 0.2))
    plt.title("Error Pos Detection vs Vicon")
    plt.grid(which="both")
    plt.legend()

    plt.subplot(312)
    plt.plot(ts_a, dpos_gts[:,0], '.', label="GT X")
    plt.plot(ts_a, dpos_gts[:,1], '.', label="GT Y")
    plt.plot(ts_a, dpos_gts[:,2], '.', label="GT Z")
    
    plt.plot(ts_a, dpos_dets[:,0], '+', label="Detection X")
    plt.plot(ts_a, dpos_dets[:,1], '+', label="Detection Y")
    plt.plot(ts_a, dpos_dets[:,2], '+', label="Detection Z")
    plt.legend()
    plt.grid()
    plt.subplot(313)
    plt.plot(ts_a, inv_dep_errs, '.', label="INV DEP ERR")

    plt.grid(which="both")
    plt.legend()

    plt.figure("Direction Err hist")
    plt.subplot(131)
    plt.hist(dpos_errs[:,0], 50, (-0.1, 0.1), density=True, facecolor='g', alpha=0.75)
    xmin, xmax = plt.xlim()
    
    mu, std = stats.norm.fit(dpos_errs[:,0])
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "X mu = %.2f,  std = %.2f" % (mu, std)
    plt.title(title)

    plt.subplot(132)
    plt.hist(dpos_errs[:,1], 50, (-0.1, 0.1), density=True, facecolor='g', alpha=0.75)
    xmin, xmax = plt.xlim()

    mu, std = stats.norm.fit(dpos_errs[:,1])
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "Y mu = %.2f,  std = %.2f" % (mu, std)
    plt.title(title)

    plt.subplot(133)
    plt.hist(dpos_errs[:,2], 50, (-0.1, 0.1), density=True, facecolor='g', alpha=0.75)
    xmin, xmax = plt.xlim()
    
    mu, std = stats.norm.fit(dpos_errs[:,2])
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "Z mu = %.2f,  std = %.2f" % (mu, std)
    plt.title(title)

    filter_dpos = np.array(dpos_errs_norm) < 0.2
    print(f"Mean {np.mean(dpos_errs[filter_dpos], axis=0)}")

    print("Pos cov", np.cov(dpos_errs[:,0]), np.cov(dpos_errs[:,1]), np.cov(dpos_errs[:,2]) )


    inv_dep_err = np.array(inv_deps) - np.array(inv_deps_gt)
    mask = np.fabs(inv_dep_err) < 0.3
    plt.figure("INV DEPS")
    plt.title("INV DEPS")
    plt.plot(ts_a, np.array(inv_deps), "+", label="INV DEP DET")
    plt.plot(ts_a, np.array(inv_deps_gt), "x", label="INV DEP GT")

    if len(good_det_id) > 0:
        for i in range(len(ts_a)):
            if det_ids[i] not in good_det_id:
                plt.text(ts_a[i], inv_deps[i], "x", color="red")

    plt.legend()
    plt.grid()

    plt.figure("INV DEPS ERR Inliers")
    plt.title("INV DEPS ERR Inliers")
    plt.plot(ts_a[mask], inv_dep_err[mask], "+", label="INV DEP DET")

    plt.legend()
    plt.grid()


    plt.figure("INV DEPS ERR HIST of inliners")
    plt.hist(np.array(inv_deps)[mask] - np.array(inv_deps_gt)[mask], 50, (-0.3, 0.3), density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(np.array(inv_deps)[mask] - np.array(inv_deps_gt)[mask])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "INV DEPS ERR  Fit results: mu = %.2f,  std = %.2f" % (mu, std)
    print("INV DEPS ERR Variance", np.mean((np.array(inv_deps) - np.array(inv_deps_gt))**2))

    plt.title(title)
    plt.legend()
    plt.grid()


def plot_loops_error(poses, loops, good_loop_id=None, outlier_show_thres=0.5, show_outlier=True):
    _loops_data = []
    dpos_loops = []
    dpos_gts = []
    dpos_gt_norms= []
    dpos_loop_norms= []
    dpos_errs = []
    dpos_errs_norm = []
    posa_gts = []
    distances = []
    ts_a = []
    dyaws = []
    dyaw_gts = []
    yawa_gts = []
    yawb_gts = []
    dyaw_errs = []
    pnp_inlier_nums = []
    idas = []
    idbs = []
    dts = []
    count_inter_loop = 0

    loops_error = {}
    loop_ids = []
    for loop in loops:
        # print(loop["id_a"], "->", loop["id_b"])
        if loop["id_a"] != loop["id_b"]:
            count_inter_loop += 1
        if loop["id_a"] not in poses or loop["id_b"] not in poses:
            continue
        posa_gt = poses[loop["id_a"]]["pos_func"](loop["ts_a"])
        posb_gt = poses[loop["id_b"]]["pos_func"](loop["ts_b"])
        yawa_gt = poses[loop["id_a"]]["ypr_func"](loop["ts_a"])[0]
        yawb_gt = poses[loop["id_b"]]["ypr_func"](loop["ts_b"])[0]
        dpos_gt = yaw_rotate_vec(-yawa_gt, posb_gt - posa_gt)
        dpos_loop = np.array(loop["dpos"])
        if np.linalg.norm(dpos_gt - dpos_loop) > outlier_show_thres and good_loop_id is not None and loop["id"] not in good_loop_id:
            continue
        _loops_data.append({
            "dpos_loop": dpos_loop,
            "dpos_gt": dpos_gt,
            "dpos_err": dpos_gt - dpos_loop})
        dpos_loops.append(dpos_loop)
        dpos_gts.append(dpos_gt)
        dpos_errs.append(dpos_gt - dpos_loop)    
        dpos_gt_norms.append(norm(dpos_gt))
        dpos_loop_norms.append(norm(dpos_loop))
        dpos_errs_norm.append(norm(dpos_gt - dpos_loop))
        
        posa_gts.append(posa_gt)
        dyaws.append(wrap_pi(loop["dyaw"]))
        dyaw_gts.append(wrap_pi(yawb_gt-yawa_gt))
        if loop["ts_a"] > loop["ts_b"]:
            ts_a.append(loop["ts_a"])
        else:
            ts_a.append(loop["ts_b"])
        yawa_gts.append(wrap_pi(yawa_gt))
        yawb_gts.append(wrap_pi(yawb_gt))
        dyaw_errs.append(wrap_pi(yawb_gt-yawa_gt-loop["dyaw"]))
        pnp_inlier_nums.append(loop["pnp_inlier_num"])
        idas.append(loop["id_a"])
        idbs.append(loop["id_b"])
        loop_ids.append(loop["id"])
        dts.append(fabs(loop["ts_b"]-loop["ts_a"]))

        if loop["id"] in loops_error:
            print("Duplicate loop", loop["id"])
        else:
            loops_error[loop["id"]] = {
                "ida": loop["id_a"],
                "idb": loop["id_b"],
                "gt_pos": dpos_gt, 
                "gt_pos_a": posa_gt,
                "gt_pos_b": posb_gt,
                "est_pos": dpos_loop, 
                "err_pos": norm(dpos_gt - dpos_loop), 
                "err_yaw": wrap_pi(yawb_gt-yawa_gt-loop["dyaw"]), 
                "dt": fabs(loop["ts_b"]-loop["ts_a"])
            }
        # if np.linalg.norm(dpos_gt - dpos_loop) > 1.0:
            # print(loop["id"], loops_error[loop["id"]])
    
    outlier_num = (np.array(dpos_errs_norm)>0.5).sum()
    total_loops = len(dpos_errs_norm)
    print(f"Outlier rate {outlier_num/total_loops*100:3.2f}% total loops {total_loops} inter_loops {count_inter_loop} outlier_num {outlier_num}")
    posa_gts = np.array(posa_gts)
    dpos_errs = np.array(dpos_errs)
    dyaw_errs = np.array(dyaw_errs)
    distances = np.array(distances)
    dpos_loops = np.array(dpos_loops)
    dpos_gts = np.array(dpos_gts)
    dyaws = np.array(dyaws)
    dyaw_gts = np.array(dyaw_gts)
    dpos_loop_norms = np.array(dpos_loop_norms)
    dpos_errs_norm = np.array(dpos_errs_norm)
    dts = np.array(dts)

    fig = plt.figure("Loop Error")
    plt.subplot(211)
    plt.tight_layout()
    plt.plot(ts_a, dpos_errs_norm, '.', label="Loop Error")
    plt.plot(ts_a, dpos_errs[:,0], '.', label="Loop Error X")
    plt.plot(ts_a, dpos_errs[:,1], '.', label="Loop Error Y")
    plt.plot(ts_a, dpos_errs[:,2], '.', label="Loop Error Z")
    for i in range(len(loop_ids)):
        if (good_loop_id is not None and loop_ids[i] not in good_loop_id) or dpos_errs_norm[i] > outlier_show_thres:
            plt.text(ts_a[i], dpos_errs_norm[i], f"x{short_loop_id(loop_ids[i])}", fontsize=12, color="red")

    plt.title(f"Error Pos Loop vs Vicon. ErrNorm max {np.max(dpos_errs_norm):.2f}m")
    plt.ylim(-np.min(dpos_errs_norm)*1.2, np.max(dpos_errs_norm)*1.2)
    plt.grid(which="both")
    plt.legend()

    plt.subplot(212)
    plt.plot(ts_a, dyaws*57.3, '.', label="DYaw Gt")
    plt.plot(ts_a, dyaw_gts*57.3, '+', label="DYaw Loop")
    plt.plot(ts_a, np.abs(dyaw_errs)*57.3, "x", label="DYaw Error")
    
    plt.title("Loop Yaw (deg)")
    plt.grid(which="both")
    plt.legend()

    # plt.subplot(313)
    # plt.plot(ts_a, pnp_inlier_nums, "x", label="pnp_inlier_nums")
    # plt.grid()

    fig = plt.figure("Loop Comp")

    plt.subplot(311)
    plt.plot(ts_a, dpos_loops[:,0], '+', label="RelPose Est")
    plt.plot(ts_a, dpos_gts[:,0], '.', label="RelPose GT")
    plt.grid(which="both")
    plt.ylabel("X")
    plt.legend()

    plt.subplot(312)
    plt.plot(ts_a, dpos_loops[:,1], '+', label="RelPose Est")
    plt.plot(ts_a, dpos_gts[:,1], '.', label="RelPose GT")
    plt.grid(which="both")
    plt.legend()
    plt.ylabel("Y")

    plt.subplot(313)
    plt.plot(ts_a, dpos_loops[:,2], '+', label="RelPose Est")
    plt.plot(ts_a, dpos_gts[:,2], '.', label="RelPose GT")
    plt.ylabel("Z")

    plt.grid(which="both")
    plt.legend()

    # plt.figure("InliersVSErr")
    # plt.title("InliersVSErr")
    # plt.plot(pnp_inlier_nums, dpos_errs_norm, "x", label="")
    # plt.grid(which="both")
    # for i in range(len(pnp_inlier_nums)):
    #     if dpos_errs_norm[i]>0.2:
    #         plt.text(pnp_inlier_nums[i], dpos_errs_norm[i], f"{short_loop_id(loop_ids[i])}|{idas[i]}->{idbs[i]}", fontsize=12)

    plt.figure("Distance vs PosErr")
    plt.title("Distance vs PosErr")
    plt.subplot(221)
    plt.plot(dpos_loop_norms, dpos_errs_norm, ".", label="")
    plt.grid(which="both")

    mask = []
    if good_loop_id is not None:
        for i in range(len(loop_ids)):
            mask.append(loop_ids[i] in good_loop_id)

    for i in range(len(pnp_inlier_nums)):
        if good_loop_id is not None and loop_ids[i] not in good_loop_id:
            plt.text(dpos_loop_norms[i], dpos_errs_norm[i], f"x{short_loop_id(loop_ids[i])}", fontsize=12, color="red")
        elif dpos_errs_norm[i]>outlier_show_thres:
            plt.text(dpos_loop_norms[i], dpos_errs_norm[i], f"{short_loop_id(loop_ids[i])}", fontsize=12)

    plt.subplot(222)
    plt.plot(dpos_loop_norms[mask], dpos_errs_norm[mask], ".", label="")
    plt.grid(which="both")


    plt.subplot(223)
    plt.plot(dpos_loop_norms, dyaw_errs*57.3, ".", label="")
    plt.grid(which="both")
    for i in range(len(pnp_inlier_nums)):
        if good_loop_id is not None and loop_ids[i] not in good_loop_id:
            plt.text(dpos_loop_norms[i], dyaw_errs[i]*57.3, f"x{short_loop_id(loop_ids[i])}", fontsize=12, color="red")
        elif dpos_errs_norm[i]>outlier_show_thres:
            plt.text(dpos_loop_norms[i], dyaw_errs[i]*57.3, f"{short_loop_id(loop_ids[i])}", fontsize=12)

    plt.subplot(224)
    plt.plot(dpos_loop_norms[mask], dyaw_errs[mask]*57.3, ".", label="")
    plt.grid(which="both")

    plt.figure("Dt vs YawErr (deg)")
    plt.plot(ts_a, dpos_errs_norm, ".", label="")
    for i in range(len(dts)):
        if good_loop_id is not None and loop_ids[i] not in good_loop_id:
            plt.text(ts_a[i], dpos_errs_norm[i], f"x{short_loop_id(loop_ids[i])}", fontsize=12, color="red")
    plt.grid()

    if good_loop_id is not None:
        dpos_errs=dpos_errs[mask]
        dyaw_errs = dyaw_errs[mask]

    plt.figure("Loop Hist")
    plt.subplot(141)
    plt.hist(dpos_errs[:,0], 50, density=True, facecolor='g', alpha=0.75)

    mu, std = stats.norm.fit(dpos_errs[:,0])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "mu = %.2e,  std = %.2e\ncov(mu=0) = %.2e" % (mu, std, RMSE(dpos_errs[:,0], 0)**2)
    plt.title(title)

    plt.subplot(142)
    plt.hist(dpos_errs[:,1], 50, density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(dpos_errs[:,1])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "mu = %.2e,  std = %.2e\ncov(mu=0) = %.2e" % (mu, std, RMSE(dpos_errs[:,1], 0)**2)
    plt.title(title)

    plt.subplot(143)
    plt.hist(dpos_errs[:,2], 50, density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(dpos_errs[:,2])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "mu = %.2e,  std = %.2e\ncov(mu=0) = %.2e" % (mu, std, RMSE(dpos_errs[:,2], 0)**2)
    plt.title(title)

    plt.subplot(144)
    plt.hist(dyaw_errs, 50, density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(dpos_errs[:,2])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "mu = %.2e,  std = %.2e\ncov(mu=0) = %.2e" % (mu, std, RMSE(dpos_errs[:,2], 0)**2)
    plt.title(title)

    print(f"Pos cov {np.cov(dpos_errs[:,0]):.1e}, {np.cov(dpos_errs[:,1]):.1e}, {np.cov(dpos_errs[:,2]):.1e}")
    print(f"Yaw cov {np.cov(dyaw_errs):.1e}")

    print(f"Pos std {np.sqrt(np.cov(dpos_errs[:,0])):.1e}, {np.sqrt(np.cov(dpos_errs[:,1])):.1e}, {np.sqrt(np.cov(dpos_errs[:,2])):.1e}")
    print(f"Yaw std {np.sqrt(np.cov(dyaw_errs)):.1e}")

    return loops_error

def debugging_pcm(pcm_folder, good_loop_id, loops_error, pcm_threshold):
    pcm_errors = {}
    pcm_errors_sum = {}
    pcm_out_thres_count = {}
    with open(pcm_folder+"/pcm_errors.txt", "r") as f:
        lines = f.readlines()
        count = 0
        for line in lines:
            nums = line.split(" ")
            if len(nums) > 1:
                loop_id_a = int(nums[0])
                loop_id_b = int(nums[1])
                pcm_error = float(nums[2])
                if loop_id_a not in pcm_errors:
                    pcm_errors[loop_id_a] = {}
                    pcm_errors_sum[loop_id_a] = 0.0
                    pcm_out_thres_count[loop_id_a] = 0
                if loop_id_b not in pcm_errors:
                    pcm_errors[loop_id_b] = {}
                    pcm_errors_sum[loop_id_b] = 0.0
                    pcm_out_thres_count[loop_id_b] = 0

                pcm_errors[loop_id_a][loop_id_b] = pcm_error
                pcm_errors[loop_id_b][loop_id_a] = pcm_error
                pcm_errors_sum[loop_id_b] += pcm_error
                pcm_errors_sum[loop_id_a] += pcm_error

                if pcm_error > pcm_threshold:
                    pcm_out_thres_count[loop_id_a] += 1
                    pcm_out_thres_count[loop_id_b] += 1
    pcm_errors_sum_array = []
    pcm_out_thres_count_array = []
    loop_error_T = []
    loop_error_yaw = []
    loop_id_array = []
    loop_dt = []
    loop_ta = []
    loop_tb = []
    for loop_id in pcm_errors_sum:
        if loop_id not in loops_error:
            # print(f"Loop {loop_id} not found")
            continue
        loop_id_array.append(loop_id)
        pcm_errors_sum_array.append(pcm_errors_sum[loop_id])
        pcm_out_thres_count_array.append(pcm_out_thres_count[loop_id])
        loop_error_T.append(loops_error[loop_id]["err_pos"])
        loop_error_yaw.append(loops_error[loop_id]["err_yaw"])
        loop_dt.append(loops_error[loop_id]["dt"])
    loop_error_yaw = np.abs(wrap_pi(np.array(loop_error_yaw)))*57.3
    plt.figure("pcm_errors_sum_array vs loop_error_T")
    plt.plot(pcm_errors_sum_array, loop_error_T, ".")
    plt.xlabel("pcm_errors_sum_array")
    plt.ylabel("loop error T")

    for i in range(len(pcm_errors_sum_array)):
        if loop_error_T[i]>0.4:
            plt.text(pcm_errors_sum_array[i], loop_error_T[i], f"{short_loop_id(loop_id_array[i])},{loop_dt[i]:.1f}s", fontsize=12)
        if loop_id_array[i] not in good_loop_id:
            plt.text(pcm_errors_sum_array[i], loop_error_T[i], "x", fontsize=12, color="red")
            

    plt.grid()

    plt.figure("pcm_errors_sum_array vs loop_error_yaw")
    plt.plot(pcm_errors_sum_array, loop_error_yaw, ".")
    plt.xlabel("pcm_errors_sum_array")
    plt.ylabel("loop error yaw")
    plt.grid()

    for i in range(len(pcm_errors_sum_array)):
        if loop_id_array[i] not in good_loop_id:
            plt.text(pcm_errors_sum_array[i], loop_error_yaw[i], "x", fontsize=12, color="red")
        if loop_error_yaw[i]>5:
            plt.text(pcm_errors_sum_array[i], loop_error_yaw[i], f"{short_loop_id(loop_id_array[i])},{loop_dt[i]:.1f}s", fontsize=12)
        

    plt.figure("pcm_errors_count vs loop_error_T")
    plt.plot(pcm_out_thres_count_array, loop_error_T, ".")
    plt.xlabel("pcm_out_thres_count_array")
    plt.ylabel("loop error T")
    plt.grid()
    for i in range(len(pcm_errors_sum_array)):
        if loop_error_T[i]>0.4:
            plt.text(pcm_out_thres_count_array[i], loop_error_T[i], f"{short_loop_id(loop_id_array[i])},{loop_dt[i]:.1f}s", fontsize=12)
            print(f"{short_loop_id(loop_id_array[i])} {loop_dt[i]:.1f} error T {loop_error_T[i]}")
        if loop_id_array[i] not in good_loop_id:
            plt.text(pcm_out_thres_count_array[i], loop_error_T[i], "x", fontsize=12, color="red")
        if pcm_out_thres_count_array[i]>100:
            plt.text(pcm_out_thres_count_array[i], loop_error_T[i], f"{short_loop_id(loop_id_array[i])},{loop_dt[i]:.1f}s", fontsize=12)

    plt.figure("pcm_errors_count vs loop_error_yaw")
    plt.plot(pcm_out_thres_count_array, loop_error_yaw, ".")
    for i in range(len(pcm_errors_sum_array)):
        if loop_id_array[i] not in good_loop_id:
            plt.text(pcm_out_thres_count_array[i], loop_error_yaw[i], "x", fontsize=12, color="red")
        
        if loop_error_yaw[i]>5:
            plt.text(pcm_out_thres_count_array[i], loop_error_yaw[i], f"{short_loop_id(loop_id_array[i])},{loop_dt[i]:.1f}s", fontsize=12)
            print(f"{(loop_id_array[i])} {loop_dt[i]:.1f} error Yaw {loop_error_yaw[i]}")
    plt.xlabel("pcm_out_thres_count_array")
    plt.ylabel("loop error yaw")
    plt.grid()
    plt.plot()
    return pcm_errors

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Localization Plot and Accuracy Analysis')
    parser.add_argument('bagname', metavar='bagname', type=str,help="Bag path")
    parser.add_argument('-r', "--replay", nargs='?', const=True, default=False, type=bool)
    args = parser.parse_args()
    nodes = [1, 2]
    print(f"Read bag from {args.bagname}, is replay bag :{args.replay}")
    poses, poses_fused, poses_vo, loops, detections = bag_read(args.bagname, nodes, args.replay)
    plot_fused(poses, poses_fused, poses_vo, loops, detections, nodes)
    plot_fused_err(poses, poses_fused, poses_vo, nodes, 1)
    plot_loops_error(poses, loops, nodes)
    plt.show()