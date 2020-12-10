#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from math import *
from scipy.interpolate import interp1d
from transformations import * 
import argparse
from numpy.linalg import norm

plt.rc('figure', figsize=(10,5))
#plt.rc('figure', figsize=(20,15))

def quat2eulers(w, x, y ,z):
    r = atan2(2 * (w * x + y * z),
                    1 - 2 * (x * x + y * y))
    p = asin(2 * (w * y - z * x))
    y = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return y, p, r

def RMSE(predictions, targets):
    return np.sqrt(np.mean((predictions-targets)**2))

def yaw_rotate_vec(yaw, vec):
    Re = rotation_matrix(yaw, [0, 0, 1])[0:3, 0:3]
    return np.transpose(np.dot(Re, np.transpose(vec)))

def read_pose_swarm_fused(bag, topic, _id, t0):
    pos = []
    ypr = []
    ts = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        for i in range(len(msg.ids)):
            _i = msg.ids[i]
            if _i == _id:
                ts.append(msg.header.stamp.to_sec() - t0)
                pos.append([msg.local_drone_position[i].x, msg.local_drone_position[i].y, msg.local_drone_position[i].z])
                ypr.append([msg.local_drone_yaw[i], 0, 0])
    ret = {
        "t": np.array(ts) ,
        "pos": np.array(pos),
        "ypr": np.array(ypr),
    }
    return ret
def read_pose_swarm_frame(bag, topic, _id, t0):
    pos = []
    ypr = []
    ts = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        for node in msg.node_frames:
            _i = node.id
            if _i == _id:
                ts.append(node.header.stamp.to_sec() - t0)
                pos.append([node.position.x, node.position.y, node.position.z])
                ypr.append([node.yaw, 0, 0])
    ret = {
        "t": np.array(ts),
        "pos": np.array(pos),
        "ypr": np.array(ypr),
    }
    return ret

def read_pose(bag, topic, t0):
    pos = []
    ypr = []
    ts = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t0 == 0:
            t0 = msg.header.stamp.to_sec()
    
        p = msg.pose.position
        q = msg.pose.orientation
        pos.append([p.x, p.y, p.z])
        y, p, r = quat2eulers(q.w, q.x, q.y, q.z)
        ypr.append([y, p, r])
        ts.append(msg.header.stamp.to_sec() - t0)
        
    ret = {
        "t": np.array(ts),
        "pos": np.array(pos),
       "pos_func": interp1d(ts, pos,axis=0,bounds_error=False,fill_value="extrapolate"),
        "ypr": np.array(ypr),
        "ypr_func": interp1d(ts, ypr,axis=0,bounds_error=False,fill_value="extrapolate")
    }
    
    print("Trajectory total length ", poses_length(ret))
    return ret, t0

def read_path(bag, topic, t0):
    path = None
    pos = []
    ypr = []
    ts = []
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        path = msg

    for msg in path.poses:
        p = msg.pose.position
        q = msg.pose.orientation
        pos.append([p.x, p.y, p.z])
        y, p, r = quat2eulers(q.w, q.x, q.y, q.z)
        ypr.append([y, p, r])
        ts.append(msg.header.stamp.to_sec() - t0)
        
    ret = {
        "t": np.array(ts),
        "pos": np.array(pos),
        "ypr": np.array(ypr)
    }

    return ret

def poses_length(poses):
    dp = np.diff(poses["pos"], axis=0)
    length = np.sum(np.linalg.norm(dp,axis=1))
    return length

def read_loops(bag, t0, topic="/swarm_loop/loop_connection"):
    loops = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # if msg.ts_a.to_sec() - t0 < -100 or msg.ts_b.to_sec() - t0 < -100:
        #     print(msg.ts_a.to_sec() - t0, msg.ts_b.to_sec() - t0)
        #     continue
        loop = {
            "ts_a": msg.ts_a.to_sec() - t0,
            "ts_b": msg.ts_b.to_sec() - t0,
            "id_a":msg.id_a,
            "id_b":msg.id_b,
            "dpos":np.array([msg.dpos.x, msg.dpos.y, msg.dpos.z]),
            "dyaw":msg.dyaw
        }
        loops.append(loop)
    return loops

def read_detections(bag, t0, topic="/swarm_drones/node_detected"):
    dets = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        det = {
            "ts": msg.header.stamp.to_sec() - t0,
            "id_a":msg.self_drone_id,
            "id_b":msg.remote_drone_id,
            "dpos":np.array([msg.dpos.x, msg.dpos.y, msg.dpos.z]),
            "inv_dep":msg.inv_dep
        }
        dets.append(det)
    return dets

def bag_read(bagname, nodes = [1, 2], is_pc=False, main_id=1):
    bag = rosbag.Bag(bagname)
    poses = {}
    poses_fused = {}
    poses_vo = {}
    poses_path = {}
    t0 = 0
    for i in nodes:
        poses[i], t0 =  read_pose(bag, f"/SwarmNode{i}/pose", t0)
        if is_pc:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused_pc", i, t0)
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path", t0)
        else:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused", i, t0)
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path", t0)

        poses_fused[i]["t"] = poses_fused[i]["t"]
        poses_vo[i] = read_pose_swarm_frame(bag, "/swarm_drones/swarm_frame_predict", i, t0)
    loops = read_loops(bag, t0, "/swarm_loop/loop_connection")
    detections = read_detections(bag, t0, "/swarm_drones/node_detected")

    bag.close()
    
    fused_offset = poses[main_id]["pos"][0] - poses_fused[main_id]["pos"][0]
    yaw_offset = (poses[main_id]["ypr"][0] - poses_fused[main_id]["ypr"][0])[0]
    Re = rotation_matrix(-yaw_offset, [0, 0, 1])[0:3, 0:3]
    for i in nodes:
        poses_fused[i]["pos"] = yaw_rotate_vec(-yaw_offset, poses_fused[i]["pos"]) + fused_offset
        poses_fused[i]["ypr"] = poses_fused[i]["ypr"] + np.array([yaw_offset, 0, 0])
        poses_fused[i]["pos_func"] = interp1d( poses_fused[i]["t"],  poses_fused[i]["pos"],axis=0,fill_value="extrapolate")
        poses_fused[i]["ypr_func"] = interp1d( poses_fused[i]["t"],  poses_fused[i]["ypr"],axis=0,fill_value="extrapolate")

        poses_path[i]["pos"] = yaw_rotate_vec(-yaw_offset, poses_path[i]["pos"]) + fused_offset
        poses_path[i]["pos_func"] = interp1d( poses_path[i]["t"],  poses_path[i]["pos"],axis=0,fill_value="extrapolate")

    for i in nodes:
        vo_offset = poses[i]["pos"][0] - poses_vo[i]["pos"][0]
        yaw_offset = (poses[i]["ypr"][0] - poses_vo[i]["ypr"][0])[0]
        Re = rotation_matrix(-yaw_offset, [0, 0, 1])[0:3, 0:3]
        poses_vo[i]["pos"] = np.transpose(np.dot(Re, np.transpose(poses_vo[i]["pos"]))) + vo_offset
        poses_vo[i]["ypr"] = poses_vo[i]["ypr"] + np.array([yaw_offset, 0, 0])
        poses_vo[i]["pos_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["pos"],axis=0,bounds_error=False,fill_value="extrapolate")
        poses_vo[i]["ypr_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["ypr"],axis=0,fill_value="extrapolate")
    
    return poses, poses_fused, poses_vo, poses_path, loops, detections
    

def plot_fused(poses, poses_fused, poses_vo, poses_path, loops, detections, nodes):
    fig = plt.figure("Ground Truth3d")
    fig.suptitle("Ground Truth3d")
    ax = fig.add_subplot(111, projection='3d')
    ax = fig.gca(projection='3d')
    
    for i in nodes:
        ax.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1],poses[i]["pos"][:,2], label=f"Vicon Traj{i}")
        #ax.scatter(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1],poses_path[i]["pos"][:,2], label=f"Fused Offline Traj{i}")
    
    ax.set_xlabel('$X$')
    ax.set_ylabel('$Y$')
    ax.set_ylabel('$Z$')
    
    #Plot Loops
    quivers = []
    quivers_det = []
    for loop in loops:
        posa_gt = poses[loop["id_a"]]["pos_func"](loop["ts_a"])
        posb_gt = poses[loop["id_b"]]["pos_func"](loop["ts_b"])
        quivers.append([posa_gt[0], posa_gt[1], posa_gt[2], posb_gt[0]-posa_gt[0], posb_gt[1]-posa_gt[1], posb_gt[2]-posa_gt[2]])
    
    for det in detections:
        posa_gt = poses[det["id_a"]]["pos_func"](det["ts"])
        yawa_gt = poses[det["id_a"]]["ypr_func"](det["ts"])[0]
        dpos = yaw_rotate_vec(yawa_gt, det["dpos"]/det["inv_dep"])
        quivers_det.append([posa_gt[0], posa_gt[1], posa_gt[2], dpos[0], dpos[1], dpos[2]])
    
    quivers = np.array(quivers)
    quivers_det = np.array(quivers_det)

    # c = np.arctan2(quivers[:,4], quivers[:,3])
    # c = (c.ravel() - c.min()) / c.ptp()
    # c = np.concatenate((c, np.repeat(c, 2)))
    # c = plt.cm.hsv(c)
    
    ax.quiver(quivers[:,0], quivers[:,1], quivers[:,2], quivers[:,3], quivers[:,4], quivers[:,5], 
        arrow_length_ratio=0.1, color="gray",linewidths=0.5)

    ax.quiver(quivers_det[:,0], quivers_det[:,1], quivers_det[:,2], quivers_det[:,3], quivers_det[:,4], quivers_det[:,5], 
        arrow_length_ratio=0.1, color="red",linewidths=1.0)

    plt.legend()

    #Plot Fused Vs GT 3D
    fig = plt.figure("Fused Vs GT 3D")
    fig.suptitle("Fused Vs GT 3D")
    for k in range(len(nodes)):
        i = nodes[k]
        ax = fig.add_subplot(1, len(nodes), k+1, projection='3d')
        ax.set_title(f"Traj {i}, length: {poses_length(poses[i]):3.3f}")
        ax.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1],poses[i]["pos"][:,2], label=f"Vicon Traj{i}")
        ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1],poses_fused[i]["pos"][:,2], label=f"Fused Traj{i}")
        ax.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1],poses_vo[i]["pos"][:,2], label=f"Aligned VO{i}")
        #ax.scatter(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1],poses_path[i]["pos"][:,2], label=f"Fused Offline Traj{i}", color="red")
        
        plt.legend()
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_ylabel('$Z$')
    
    fig = plt.figure("Fused Vs GT 2D")
    fig.suptitle("Fused Vs GT 2D")
    for k in range(len(nodes)):
        i = nodes[k]
        ax = fig.add_subplot(1, len(nodes), k+1)
        ax.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1], label=f"Vicon Traj{i}")
        ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Fused Traj{i}")
        #ax.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1], label=f"Aligned VO Traj{i}")
        ax.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")

        plt.grid()
        plt.legend()
    
def plot_fused_err(poses, poses_fused, poses_vo, poses_path, nodes, main_id=1):
    t_calib = 0
    fig = plt.figure("Fused Vs GT 1D")
    fig.suptitle("Fused Vs GT 1D")
    ax1, ax2, ax3 = fig.subplots(3, 1)

    for i in nodes:
        t_ = poses_fused[i]["t"]
        pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"])
        pos_fused = poses_fused[i]["pos"]
        _i = str(i) 

        ax1.plot(t_, pos_gt[:,0], label="$x_{gt}^" + _i + "$")
        ax1.plot(t_, pos_fused[:,0], label="$x_{fused}^" + _i + "$")
        ax1.plot(poses_path[i]["t"], poses_path[i]["pos"][:,0], '.', label=f"Fused Offline Traj{i}")

        ax2.plot(t_, pos_gt[:,1], label="$y_{gt}^" + _i + "$")
        ax2.plot(t_, pos_fused[:,1], label="$y_{fused}^" + _i + "$")
        ax2.plot(poses_path[i]["t"], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")

        ax3.plot(t_, pos_gt[:,2], label="$z_{gt}^" + _i + "$")
        ax3.plot(t_, pos_fused[:,2], label="$z_{fused}^" + _i + "$")
        ax3.plot(poses_path[i]["t"], poses_path[i]["pos"][:,2], '.', label=f"Fused Offline Traj{i}")

    ax1.legend()
    ax2.legend()
    ax3.legend()
    ax1.grid()
    ax2.grid()
    ax3.grid()

    #Plot Fused Vs GT absolute error
    fig = plt.figure("Fused Absolute Error")
    fig.suptitle("Fused Absolute Error")
    ax1, ax2, ax3 = fig.subplots(3, 1)
    for i in nodes:
        t_ = poses_fused[i]["t"]
        t0 =t_[0]
        pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"]+t_calib)
        pos_fused = poses_fused[i]["pos"]
        pos_vo = poses_vo[i]["pos"]
        pos_path = poses_path[i]["pos"]
        _i = str(i) 
        rmse_x = RMSE(pos_gt[:,0] , pos_fused[:,0])
        rmse_y = RMSE(pos_gt[:,1] , pos_fused[:,1])
        rmse_z = RMSE(pos_gt[:,2] , pos_fused[:,2])
        
        label = f"$errx_{i}$ RMSE{i}:{rmse_x:3.3f}"
        ax1.plot(t_, pos_gt[:,0]  - pos_fused[:,0], label=label)

        label = f"$erry_{i}$ RMSE{i}:{rmse_y:3.3f}"
        ax2.plot(t_, pos_gt[:,1]  - pos_fused[:,1], label=label)

        label = f"$erry_{i}$ RMSE{i}:{rmse_z:3.3f}"
        ax3.plot(t_,  pos_gt[:,1]  - pos_fused[:,1], label=label)

        pos_gt =  poses[i]["pos_func"](poses_vo[i]["t"]+t_calib)
        
        rmse_vo_x = RMSE(pos_vo[:,0] , pos_gt[:,0])
        rmse_vo_y = RMSE(pos_vo[:,1] , pos_gt[:,1])
        rmse_vo_z = RMSE(pos_vo[:,2] , pos_gt[:,2])
        
        label = f"$VO errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
        ax1.plot(poses_vo[i]["t"], pos_gt[:,0]  - pos_vo[:,0], label=label)

        label = f"$VO erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
        ax2.plot(poses_vo[i]["t"], pos_gt[:,1]  - pos_vo[:,1], label=label)
        
        label = f"$VO errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
        ax3.plot(poses_vo[i]["t"], pos_gt[:,2]  - pos_vo[:,2], label=label)


        pos_gt =  poses[i]["pos_func"](poses_path[i]["t"]+t_calib)
        
        rmse_path_x = RMSE(pos_path[:,0] , pos_gt[:,0])
        rmse_path_y = RMSE(pos_path[:,1] , pos_gt[:,1])
        rmse_path_z = RMSE(pos_path[:,2] , pos_gt[:,2])
        
        label = f"$Path errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
        ax1.plot(poses_path[i]["t"], pos_gt[:,0]  - pos_path[:,0], "+", label=label)

        label = f"$Path erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
        ax2.plot(poses_path[i]["t"], pos_gt[:,1]  - pos_path[:,1], "+",label=label)
        
        label = f"$Path errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
        ax3.plot(poses_path[i]["t"], pos_gt[:,2]  - pos_path[:,2], "+", label=label)
    
        print(f"RMSE Fused Online {i} is {rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}")
        print(f"RMSE Fused Offline Path {i} is {rmse_path_x:3.3f},{rmse_path_y:3.3f},{rmse_path_z:3.3f}")
        print(f"RMSE VO {i} is {rmse_vo_x:3.3f},{rmse_vo_y:3.3f},{rmse_vo_z:3.3f}")

    # ax1.legend()
    # ax2.legend()
    ax3.legend()
    ax1.grid()
    ax2.grid()
    ax3.grid()

    fig = plt.figure("Relative Pose")
    fig.suptitle("Relative Pose")
    
    ts = poses_fused[main_id]["t"]
    posa_gt =  poses[main_id]["pos_func"](ts)
    posa_fused = poses_fused[main_id]["pos_func"](ts)
    posa_vo =  poses[main_id]["pos_func"](ts)
    yawa_fused = poses_fused[main_id]["ypr_func"](ts)[:,0]
    yawa_gt = poses[main_id]["ypr_func"](ts)[:,0]
    ax1, ax2, ax3 = fig.subplots(3, 1)
    
    for i in nodes:
        if i!= main_id:
            posb_gt =  poses[i]["pos_func"](poses_fused[i]["t"])
            yaw_gt = poses[i]["ypr_func"](poses_fused[i]["t"])[:,0]
            
            posb_fused = poses_fused[i]["pos"]
            #posb_vo = poses_fused[i]["pos"]
            
            dp_gt = posb_gt - posa_gt
            dp_fused = posb_fused - posa_fused
            #dp_vo = posb_vo - posa_vo
            for i in range(len(yawa_fused)):
                yaw = yawa_fused[i]
                _dp_fused = np.transpose(dp_fused[i])
                Re = rotation_matrix(-yaw, [0, 0, 1])[0:3, 0:3]
                dp_fused[i] = np.transpose(np.dot(Re, _dp_fused))

            for i in range(len(yawa_fused)):
                yaw = yawa_gt[i]
                _dp_gt = np.transpose(dp_gt[i])
                Re = rotation_matrix(-yaw, [0, 0, 1])[0:3, 0:3]
                dp_gt[i] = np.transpose(np.dot(Re, _dp_gt))
                
            ax1.plot(ts, dp_gt[:,0], label="$X_{gt}^" + str(i) + "$")
            ax2.plot(ts, dp_gt[:,1], label="$Y_{gt}^" + str(i) + "$")
            ax3.plot(ts, dp_gt[:,2], label="$Z_{gt}^" + str(i) + "$")

            ax1.plot(ts, dp_fused[:,0], label="$X_{fused}^" + str(i) + "$")
            ax2.plot(ts, dp_fused[:,1], label="$Y_{fused}^" + str(i) + "$")
            ax3.plot(ts, dp_fused[:,2], label="$Z_{fused}^" + str(i) + "$")
            
    ax1.legend()
    ax2.legend()
    ax3.legend()
    ax1.grid()
    ax2.grid()
    ax3.grid()
        
    fig = plt.figure("Fused Relative Error")
    fig.suptitle("Fused Relative Error")
    ax1, ax2, ax3 = fig.subplots(3, 1)
    for i in nodes:
        if i!= main_id:
            posb_gt =  poses[i]["pos_func"](poses_fused[i]["t"])
            yaw_gt = poses[i]["ypr_func"](poses_fused[i]["t"])[:,0]
            
            posb_fused = poses_fused[i]["pos"]
            yaw_fused = poses_fused[i]["pos"]
            posb_vo = poses_fused[i]["pos"]
            
            dp_gt = posb_gt - posa_gt
            dp_fused = posb_fused - posa_fused
            dp_vo = posb_vo - posa_vo
                
            rmse_x = RMSE(dp_gt[:,0] , dp_fused[:,0])
            rmse_y = RMSE(dp_gt[:,1] , dp_fused[:,1])
            rmse_z = RMSE(dp_gt[:,2] , dp_fused[:,2])

            ax1.plot(ts, dp_gt[:,0] - dp_fused[:,0], label="$E_{xfused}^" + str(i) + f"$ RMSE:{rmse_x:3.3f}")
            ax2.plot(ts, dp_gt[:,1] - dp_fused[:,1], label="$E_{yfused}^" + str(i) + f"$ RMSE:{rmse_y:3.3f}")
            ax3.plot(ts, dp_gt[:,2] - dp_fused[:,2], label="$E_{zfused}^" + str(i) + f"$ RMSE:{rmse_z:3.3f}")
            print(f"RMSE {main_id}->{i} {rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}")
            
    ax1.legend()
    ax2.legend()
    ax3.legend()
    ax1.grid()
    ax2.grid()
    ax3.grid()
    plt.show()
    
    fig = plt.figure("Yaw")
    plt.title("yaw")
    ax1, ax2 = fig.subplots(2, 1)
    for i in nodes:
        t_ = poses_fused[i]["t"]
        yaw_gt =  poses[i]["ypr_func"](poses_fused[i]["t"])
        yaw_fused = poses_fused[i]["ypr"]
        
        ax1.plot(t_,  yaw_gt[:,0], label=f"$\psi gt{i}$")
        ax1.plot(t_,  yaw_fused[:,0], label=f"$\psi fused{i}$")

        ax2.plot(t_,  (yaw_fused[:,0] -  yaw_gt[:,0] + np.pi) % (2 * np.pi) - np.pi, label=f"$\psi_E{i}$")
        yaw_gt=  poses[i]["ypr_func"](poses_vo[i]["t"])
        ax2.plot(poses_vo[i]["t"],  (poses_vo[i]["ypr"][:,0] -  yaw_gt[:,0] + np.pi) % (2 * np.pi) - np.pi, label=f"$VO \psi_E{i}$")

    ax2.set_ylim(-0.5, 0.5)
    ax1.grid()
    ax2.grid()
    ax1.legend()
    ax2.legend() 

def plot_detection_error(poses, detections, nodes):
    _dets_data = []
    dpos_dets = []
    dpos_gts = []
    dpos_gt_norms= []
    dpos_det_norms= []
    dpos_errs = []
    inv_dep_errs = []
    dpos_errs_norm = []
    posa_gts = []
    ts_a = []
    dyaws = []
    yawa_gts = []
    print("Total detection", len(detections))
    for det in detections:
        if det["id_a"] == 2:
            continue
        posa_gt = poses[det["id_a"]]["pos_func"](det["ts"])
        posb_gt = poses[det["id_b"]]["pos_func"](det["ts"])
        yawa_gt = poses[det["id_a"]]["ypr_func"](det["ts"])[0]
        dpos_gt = yaw_rotate_vec(-yawa_gt, posb_gt - posa_gt)
        inv_dep_gt = 1/norm(dpos_gt)
        dpos_gt = dpos_gt * inv_dep_gt
        
        dpos_det = np.array(det["dpos"]) - np.array([0.02, 0, 0.05])
        inv_dep_det = det["inv_dep"]
        _dets_data.append({
            "dpos_det": dpos_det,
            "dpos_gt": dpos_gt,
            "dpos_err": dpos_gt - dpos_det,
            "inv_dep_err": inv_dep_gt - inv_dep_det
            })
        inv_dep_errs.append(inv_dep_gt - inv_dep_det)
        dpos_dets.append(dpos_det)
        dpos_gts.append(dpos_gt)
        dpos_errs.append(dpos_gt - dpos_det)    
        dpos_gt_norms.append(norm(dpos_gt))
        dpos_det_norms.append(norm(dpos_det))
        dpos_errs_norm.append(norm(dpos_gt - dpos_det))
        posa_gts.append(posa_gt)
        ts_a.append(det["ts"])
        yawa_gts.append(yawa_gt)
        
#         if np.linalg.norm(dpos_gt - dpos_det) > 1.0:
#             print("Error", np.linalg.norm(dpos_gt - dpos_loop) , loop)
        
    posa_gts = np.array(posa_gts)
    dpos_errs = np.array(dpos_errs)
    dpos_gts = np.array(dpos_gts)
    dpos_dets = np.array(dpos_dets)
    fig = plt.figure()

    plt.subplot(311)
    plt.plot(ts_a, dpos_errs_norm, '.', label="Err NORM")
    plt.plot(ts_a, np.abs(dpos_errs[:,0]), '+', label="ERR X")
    plt.plot(ts_a, np.abs(dpos_errs[:,1]), '+', label="ERR Y")
    plt.plot(ts_a, dpos_errs[:,2], '+', label="ERR Z")
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

    plt.figure("Hist")
    plt.subplot(141)
    plt.hist(inv_dep_errs, 5, density=True, facecolor='g', alpha=0.75)
    plt.subplot(142)
    plt.hist(dpos_errs[:,0], 5, density=True, facecolor='g', alpha=0.75)
    plt.subplot(143)
    plt.hist(dpos_errs[:,1], 5, density=True, facecolor='g', alpha=0.75)
    plt.subplot(144)
    plt.hist(dpos_errs[:,2], 5, density=True, facecolor='g', alpha=0.75)
    
    print(f"Mean {np.mean(dpos_errs, axis=0)}")


    print("Pos cov", np.cov(dpos_errs[:,0]), np.cov(dpos_errs[:,1]), np.cov(dpos_errs[:,2]) )
    
def plot_loops_error(poses, loops, nodes):
    _loops_data = []
    dpos_loops = []
    dpos_gts = []
    dpos_gt_norms= []
    dpos_loop_norms= []
    dpos_errs = []
    dpos_errs_norm = []
    posa_gts = []
    ts_a = []
    dyaws = []
    dyaw_gts = []
    yawa_gts = []
    yawb_gts = []
    dyaw_errs = []
    print("Total loops", len(loops))
    for loop in loops:
        posa_gt = poses[loop["id_a"]]["pos_func"](loop["ts_a"])
        posb_gt = poses[loop["id_b"]]["pos_func"](loop["ts_b"])
        yawa_gt = poses[loop["id_a"]]["ypr_func"](loop["ts_a"])[0]
        yawb_gt = poses[loop["id_b"]]["ypr_func"](loop["ts_b"])[0]
        dpos_gt = yaw_rotate_vec(-yawa_gt, posb_gt - posa_gt)
        dpos_loop = np.array(loop["dpos"])
        _loops_data.append({
            "dpos_loop": dpos_loop,
            "dpos_gt": dpos_gt,
            "dpos_err": dpos_gt - dpos_loop
            })
        dpos_loops.append(dpos_loop)
        dpos_gts.append(dpos_gt)
        dpos_errs.append(dpos_gt - dpos_loop)    
        dpos_gt_norms.append(norm(dpos_gt))
        dpos_loop_norms.append(norm(dpos_loop))
        dpos_errs_norm.append(norm(dpos_gt - dpos_loop))
        posa_gts.append(posa_gt)
        dyaws.append(loop["dyaw"])
        dyaw_gts.append(yawb_gt-yawa_gt)
        ts_a.append(loop["ts_a"])
        yawa_gts.append(yawa_gt)
        yawb_gts.append(yawb_gt)
        dyaw_errs.append(yawb_gt-yawa_gt-loop["dyaw"])
        
        if np.linalg.norm(dpos_gt - dpos_loop) > 1.0:
            print("Error", np.linalg.norm(dpos_gt - dpos_loop) , loop)
        
    posa_gts = np.array(posa_gts)
    dpos_errs = np.array(dpos_errs)
    fig = plt.figure()

    plt.subplot(311)
    plt.plot(ts_a, dpos_errs_norm, '.', label="Loop Error")
    plt.plot(ts_a, np.abs(dpos_errs[:,0]), '+', label="Loop Error X")
    plt.plot(ts_a, np.abs(dpos_errs[:,1]), '+', label="Loop Error Y")
    plt.plot(ts_a, dpos_errs[:,2], '+', label="Loop Error Z")
    plt.title("Error Pos Loop vs Vicon")
    plt.grid(which="both")
    plt.legend()

    plt.subplot(312)
    plt.plot(ts_a, dyaws, '.', label="DYaw Gt")
    plt.plot(ts_a, dyaw_gts, '+', label="DYaw Loop")
    plt.plot(ts_a, yawa_gts, "*", label="Vicon Yaw A")
    plt.plot(ts_a, np.abs(dyaw_errs), "x", label="DYaw Error")

    #plt.plot(ts_a, yawb_gts, "*", label="Vicon Yaw B")
    plt.title("Error Yaw Loop vs Vicon")
    plt.grid(which="both")
    plt.legend()


    plt.subplot(313)
    plt.plot(ts_a, posa_gts[:,0], '+', label="Vicon X")
    plt.plot(ts_a, posa_gts[:,1], '+', label="Vicon Y")
    plt.plot(ts_a, posa_gts[:,2], '+', label="Vicon Z")
    # plt.plot(poses[i]["t"], poses[i]["pos"][:,0], label="Vicon X")
    # plt.plot(poses[i]["t"], poses[i]["pos"][:,1], label="Vicon Y")
    # plt.plot(poses[i]["t"], poses[i]["pos"][:,2], label="Vicon Z")

    plt.grid(which="both")
    plt.legend()

    plt.figure()
    plt.subplot(141)
    plt.hist(dpos_errs_norm, 5, density=True, facecolor='g', alpha=0.75)
    plt.subplot(142)
    plt.hist(dpos_errs[:,0], 5, density=True, facecolor='g', alpha=0.75)
    plt.subplot(143)
    plt.hist(dpos_errs[:,1], 5, density=True, facecolor='g', alpha=0.75)
    plt.subplot(144)
    plt.hist(dpos_errs[:,2], 5, density=True, facecolor='g', alpha=0.75)


    print("Pos cov", np.cov(dpos_errs[:,0]), np.cov(dpos_errs[:,1]), np.cov(dpos_errs[:,2]) )
    print("Yaw cov", np.cov(dyaw_errs))


    # plt.figure()
    # plt.subplot(211)
    # plt.plot(dpos_gt_norms, dpos_errs_norm, 'o', label="GtDistance vs Error")
    # plt.grid(which="both")
    # plt.subplot(212)
    # plt.plot(dpos_loop_norms, dpos_errs_norm, 'o', label="LoopDistance vs Error")
    # plt.grid(which="both")
    # plt.legend()
    # plt.show()

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