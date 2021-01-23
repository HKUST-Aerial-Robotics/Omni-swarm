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

# def read_distance_swarm_frame(bag, topic, _id, to)
def read_distances_swarm_frame(bag, topic, t0, main_id):
    distances = {
    }
    ts = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        for node in msg.node_frames:
            if node.id == main_id:
                for i in range(len(node.dismap_ids)):
                    _id = node.dismap_ids[i]
                    _dis = node.dismap_dists[i]
                    if not (_id in distances):
                        distances[_id] = {
                            "t" : [],
                            "dis" : []
                        }
                    distances[_id]["t"].append(msg.header.stamp.to_sec() - t0)
                    distances[_id]["dis"].append(_dis)
    return distances

def read_distances_remote_nodes(bag, topic, t0, main_id):
    distances = {
    }
    ts = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        for i in range(len(msg.node_ids)):
            if msg.active[i]:
                _id = msg.node_ids[i]
                _dis = msg.node_dis[i]
                if not (_id in distances):
                    distances[_id] = {
                        "t" : [],
                        "dis" : []
                    }
                distances[_id]["t"].append(msg.header.stamp.to_sec() - t0)
                distances[_id]["dis"].append(_dis)
    return distances

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
        "pos_raw": np.array(pos),
        "pos_raw_func": interp1d(ts, pos,axis=0,bounds_error=False,fill_value="extrapolate"),
        "ypr_raw": np.array(ypr),
        "ypr_raw_func": interp1d(ts, ypr,axis=0,bounds_error=False,fill_value="extrapolate"),
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


def parse_path(path, t0):
    pos = []
    ypr = []
    ts = []
    
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

def read_path_all(bag, topic, t0):
    pathes = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        pathes.append(parse_path(msg, t0))
    return pathes

def read_path(bag, topic, t0):
    path = None
    for topic, msg, t in bag.read_messages(topics=[topic]):
        path = msg
    return parse_path(path, t0)

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
            "pos_a" : np.array([msg.local_pose_self.position.x, msg.local_pose_self.position.y, msg.local_pose_self.position.z]),
            "pos_b" : np.array([msg.local_pose_remote.position.x, msg.local_pose_remote.position.y, msg.local_pose_remote.position.z]),
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
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path_pc", t0)
        else:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused", i, t0)
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path", t0)

        poses_fused[i]["t"] = poses_fused[i]["t"]
        poses_vo[i] = read_pose_swarm_frame(bag, "/swarm_drones/swarm_frame_predict", i, t0)
    loops = read_loops(bag, t0, "/swarm_loop/loop_connection")
    detections = read_detections(bag, t0, "/swarm_drones/node_detected")
    distances = read_distances_remote_nodes(bag, "/uwb_node/remote_nodes", t0, main_id)
    bag.close()
    
    fused_offset = poses[main_id]["pos"][0] - poses_fused[main_id]["pos"][0]
    yaw_offset = (poses[main_id]["ypr"][0] - poses_fused[main_id]["ypr"][0])[0]

    print("Yaw Offset, ", yaw_offset*57.3, "Fused Offset", fused_offset)

    #Pvicon = DP Ppose
    #DP = PviconPpose^-1
    #DP = (PPose^-1 Pvicon)^-1
    #PVicon = DYaw * Pos
    #YawVicon = DYaw + Yaw
    for i in nodes:
        poses_fused[i]["pos"] = yaw_rotate_vec(yaw_offset, poses_fused[i]["pos"]) + fused_offset
        poses_fused[i]["ypr"] = poses_fused[i]["ypr"] + np.array([yaw_offset, 0, 0])
        poses_fused[i]["pos_func"] = interp1d( poses_fused[i]["t"],  poses_fused[i]["pos"],axis=0,fill_value="extrapolate")
        poses_fused[i]["ypr_func"] = interp1d( poses_fused[i]["t"],  poses_fused[i]["ypr"],axis=0,fill_value="extrapolate")

        poses_path[i]["ypr"] = poses_path[i]["ypr"] + np.array([yaw_offset, 0, 0])
        poses_path[i]["pos"] = yaw_rotate_vec(yaw_offset, poses_path[i]["pos"]) + fused_offset
        poses_path[i]["pos_func"] = interp1d( poses_path[i]["t"],  poses_path[i]["pos"],axis=0,fill_value="extrapolate")
        poses_path[i]["ypr_func"] = interp1d( poses_path[i]["t"],  poses_path[i]["ypr"],axis=0,fill_value="extrapolate")

    for i in nodes:
        vo_offset = poses[i]["pos"][0] - poses_vo[i]["pos_raw"][0]
        yaw_offset = (poses[i]["ypr"][0] - poses_vo[i]["ypr_raw"][0])[0]
        poses_vo[i]["pos"] = yaw_rotate_vec(yaw_offset, poses_vo[i]["pos_raw"]) + vo_offset
        poses_vo[i]["ypr"] = poses_vo[i]["ypr_raw"] + np.array([yaw_offset, 0, 0])
        poses_vo[i]["pos_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["pos"],axis=0,bounds_error=False,fill_value="extrapolate")
        poses_vo[i]["ypr_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["ypr"],axis=0,fill_value="extrapolate")
    
    return poses, poses_fused, poses_vo, poses_path, loops, detections, distances
    

def plot_fused(poses, poses_fused, poses_vo, poses_path, loops, detections, nodes, t_calib = {1:0, 2:0}):
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

    if len(quivers) > 0:   
        ax.quiver(quivers[:,0], quivers[:,1], quivers[:,2], quivers[:,3], quivers[:,4], quivers[:,5], 
            arrow_length_ratio=0.1, color="gray",linewidths=0.5)

    if len(quivers_det) > 0:   
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
        ax.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1],poses_path[i]["pos"][:,2], label=f"Fused Offline Traj{i}", color="red")
        
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
        # ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Fused Traj{i}")
        ax.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1], label=f"Aligned VO Traj{i}")
        # ax.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")
        ax.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], label=f"Fused Offline Traj{i}")

        plt.grid()
        plt.legend()

    for i in nodes:
        fig = plt.figure(f"Drone {i} fused Vs GT 1D")
        fig.suptitle(f"Drone {i} fused Vs GT 1D")
        ax1, ax2, ax3 = fig.subplots(3, 1)

        t_ = poses_fused[i]["t"]
        pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"] + t_calib[i])
        pos_fused = poses_fused[i]["pos"]
        _i = str(i) 

        ax1.plot(t_, pos_gt[:,0], label="$x_{gt}^" + _i + "$")
        ax1.plot(t_, pos_fused[:,0], label="$x_{fused}^" + _i + "$")
        # ax1.plot(poses_path[i]["t"], poses_path[i]["pos"][:,0], '.', label=f"Fused Offline Traj{i}")
        ax1.plot(poses_path[i]["t"], poses_path[i]["pos"][:,0], label=f"Fused Offline Traj{i}")
        ax1.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,0], label=f"Aligned VO Traj{i}")

        ax2.plot(t_, pos_gt[:,1], label="$y_{gt}^" + _i + "$")
        ax2.plot(t_, pos_fused[:,1], label="$y_{fused}^" + _i + "$")
        #ax2.plot(poses_path[i]["t"], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")
        #ax2.plot(poses_path[i]["t"], poses_path[i]["pos"][:,1], label=f"Fused Offline Traj{i}")
        ax2.plot(poses_path[i]["t"], poses_path[i]["pos"][:,1], label=f"Fused Offline Traj{i}")
        ax2.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,1], label=f"Aligned VO Traj{i}")

        ax3.plot(t_, pos_gt[:,2], label="$z_{gt}^" + _i + "$")
        ax3.plot(t_, pos_fused[:,2], label="$z_{fused}^" + _i + "$")
        #ax3.plot(poses_path[i]["t"], poses_path[i]["pos"][:,2], '.', label=f"Fused Offline Traj{i}")
        #ax3.plot(poses_path[i]["t"], poses_path[i]["pos"][:,2], label=f"Fused Offline Traj{i}")
        ax3.plot(poses_path[i]["t"], poses_path[i]["pos"][:,2], label=f"Fused Offline Traj{i}")
        ax3.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,2], label=f"Aligned VO Traj{i}")
        ax1.legend()
        ax2.legend()
        ax3.legend()
        ax1.grid()
        ax2.grid()
        ax3.grid()

def plot_fused_diff(poses, poses_fused, poses_vo, nodes = [1, 2], t_calib = {1:0, 2:0}):
    for i in nodes:
        fig = plt.figure(f"Drone diff {i} fused Vs GT 1D")
        fig.suptitle(f"Drone  diff {i} fused Vs GT 1D")
        ax1, ax2, ax3 = fig.subplots(3, 1)

        t_ = poses_fused[i]["t"]
        pos_fused = poses_fused[i]["pos"]
        pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"] + t_calib[i])
        pos_vo = poses_vo[i]["pos_func"](poses_fused[i]["t"])
        _i = str(i) 

        ax1.plot(t_[0:-1], np.diff(pos_gt[:,0]), label="$x_{gt}^" + _i + "$")
        ax1.plot(t_[0:-1], np.diff(pos_fused[:,0]), label="$x_{fused}^" + _i + "$")
        #ax1.plot(t_[0:-1], np.diff(pos_vo[:,0]), label=f"Aligned VO Traj{i}")

        ax2.plot(t_[0:-1], np.diff(pos_gt[:,1]), label="$y_{gt}^" + _i + "$")
        ax2.plot(t_[0:-1], np.diff(pos_fused[:,1]), label="$y_{fused}^" + _i + "$")
        #ax2.plot(t_[0:-1], np.diff(pos_vo[:,1]), label=f"Aligned VO Traj{i}")

        ax3.plot(t_[0:-1], np.diff(pos_gt[:,2]), label="$z_{gt}^" + _i + "$")
        ax3.plot(t_[0:-1], np.diff(pos_fused[:,2]), label="$z_{fused}^" + _i + "$")
        #ax3.plot(t_[0:-1], np.diff(pos_vo[:,2]), label=f"Aligned VO Traj{i}")

        ax1.legend()
        ax2.legend()
        ax3.legend()
        ax1.grid()
        ax2.grid()
        ax3.grid()

        rmse_min_pos = np.array([100, 100, 100])
        rmse_min_vel = np.array([100, 100, 100])
        rmse_best_dt_pos = 0
        rmse_best_dt_vel = 0
        pos_fused = poses_fused[i]["pos"]
        pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"])
        rmse_vel = np.array([
            RMSE(np.diff(pos_gt[:,0]),np.diff(pos_fused[:,0])),
            RMSE(np.diff(pos_gt[:,1]),np.diff(pos_fused[:,1])),
            RMSE(np.diff(pos_gt[:,2]),np.diff(pos_fused[:,2]))
            ])
        rmse_pos = np.array([RMSE(pos_gt[:,0],pos_fused[:,0]),RMSE(pos_gt[:,1],pos_fused[:,1]),RMSE(pos_gt[:,2],pos_fused[:,2])])
        print(f"RMSE for node @no calib {i} p {rmse_pos} v {rmse_vel}")

        for c in range(-100, 100, 1):
            dt = c / 100.0
            pos_fused = poses_fused[i]["pos"]
            pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"] + dt)
            rmse_vel = np.array([RMSE(np.diff(pos_gt[:,0]),np.diff(pos_fused[:,0])),RMSE(np.diff(pos_gt[:,1]),np.diff(pos_fused[:,1])),RMSE(np.diff(pos_gt[:,2]),np.diff(pos_fused[:,2]))])
            rmse_pos = np.array([RMSE(pos_gt[:,0],pos_fused[:,0]),RMSE(pos_gt[:,1],pos_fused[:,1]),RMSE(pos_gt[:,2],pos_fused[:,2])])
            if norm(rmse_pos) < norm(rmse_min_pos):
                rmse_min_pos = rmse_pos
                rmse_best_dt_pos = dt

            if norm(rmse_vel) < norm(rmse_min_vel):
                rmse_min_vel = rmse_vel
                rmse_best_dt_vel = dt
        
        print(f"Best RMSE for node {i} p {rmse_best_dt_pos}: {rmse_min_pos} v {rmse_best_dt_vel}: {rmse_min_vel}")

def plot_distance_err(poses, poses_fused, poses_vo, poses_path, distances, main_id, nodes,  t_calib = {1:0, 2:0}):

    for i in nodes:
        if i == main_id:
            continue
        fig = plt.figure(f"Distance {i}")
        fig.suptitle(f"Distance {i}")
        ax1, ax2, ax3 = fig.subplots(3, 1)
        t_ = np.array(distances[i]["t"])
        pos_gt = poses[i]["pos_func"](t_+t_calib[i])
        pos_fused = poses_fused[i]["pos_func"](t_)

        main_pos_gt = poses[main_id]["pos_func"](t_+t_calib[main_id])
        main_pos_fused = poses_fused[main_id]["pos_func"](t_)

        #pos_vo = poses_vo[i]["pos"]
        #pos_path = poses_path[i]["pos"](t_)
        
        dis_raw = distances[i]["dis"]
        dis_gt = norm(pos_gt  - main_pos_gt, axis=1)
        dis_fused = norm(pos_fused  - main_pos_fused, axis=1)
        #dis_vo = norm(pos_path  - main_pos_path, axis=0)

        ax1.plot(t_, dis_gt, label="Distance GT")
        ax1.plot(t_, dis_fused,label="Distance Fused")
        ax1.plot(t_, dis_raw, ".", label="Distance UWB")


        ax2.plot(t_, dis_gt-dis_raw, ".", label="Distance Error of UWB")
        ax2.plot(t_, dis_gt-dis_fused, label="Distance Error of Fused")

        ax3.plot(dis_gt, dis_raw,'+',label="GT VS UWB")

        print(f"Distce RMSE {RMSE(dis_raw, dis_gt)}")
        z = np.polyfit(dis_gt, dis_raw, 1)
        print(f"Fit {z[0]}, {z[1]}")
        ax1.legend()
        ax1.grid()
        
        ax2.legend()
        ax2.grid()
        
        ax3.legend()
        ax3.grid()
    


def plot_fused_err(poses, poses_fused, poses_vo, poses_path, nodes, main_id=1, t_calib = {1:0, 2:0}):
    #Plot Fused Vs GT absolute error
    for i in nodes:
        fig = plt.figure(f"Fused Absolute Error {i}")
        fig.suptitle(f"Fused Absolute Error {i}")
        ax1, ax2, ax3 = fig.subplots(3, 1)
        t_ = poses_fused[i]["t"]
        pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"]+t_calib[i])
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

        pos_gt =  poses[i]["pos_func"](poses_vo[i]["t"]+t_calib[i])
        
        rmse_vo_x = RMSE(pos_vo[:,0] , pos_gt[:,0])
        rmse_vo_y = RMSE(pos_vo[:,1] , pos_gt[:,1])
        rmse_vo_z = RMSE(pos_vo[:,2] , pos_gt[:,2])
        
        label = f"$VO errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
        ax1.plot(poses_vo[i]["t"], pos_gt[:,0]  - pos_vo[:,0], label=label)

        label = f"$VO erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
        ax2.plot(poses_vo[i]["t"], pos_gt[:,1]  - pos_vo[:,1], label=label)
        
        label = f"$VO errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
        ax3.plot(poses_vo[i]["t"], pos_gt[:,2]  - pos_vo[:,2], label=label)


        pos_gt =  poses[i]["pos_func"](poses_path[i]["t"]+t_calib[i])
        
        rmse_path_x = RMSE(pos_path[:,0] , pos_gt[:,0])
        rmse_path_y = RMSE(pos_path[:,1] , pos_gt[:,1])
        rmse_path_z = RMSE(pos_path[:,2] , pos_gt[:,2])
        
        # label = f"$Path errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
        # ax1.plot(poses_path[i]["t"], pos_gt[:,0]  - pos_path[:,0], label=label)

        # label = f"$Path erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
        # ax2.plot(poses_path[i]["t"], pos_gt[:,1]  - pos_path[:,1], label=label)
        
        # label = f"$Path errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
        # ax3.plot(poses_path[i]["t"], pos_gt[:,2]  - pos_path[:,2], label=label)
    
        print(f"RMSE Fused Online {i} is {rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}")
        print(f"RMSE Fused Offline Path {i} is {rmse_path_x:3.3f},{rmse_path_y:3.3f},{rmse_path_z:3.3f}")
        print(f"RMSE VO {i} is {rmse_vo_x:3.3f},{rmse_vo_y:3.3f},{rmse_vo_z:3.3f}")

        ax3.legend()
        ax1.grid()
        ax2.grid()
        ax3.grid()

    # poses_fused = poses_path
    ts = poses_fused[main_id]["t"]
    posa_gt =  poses[main_id]["pos_func"](ts)
    posa_fused = poses_fused[main_id]["pos_func"](ts)
    posa_vo =  poses[main_id]["pos_func"](ts)
    yawa_fused = poses_fused[main_id]["ypr_func"](ts)[:,0]
    yawa_gt = poses[main_id]["ypr_func"](ts)[:,0]
    
    fig = plt.figure("Relative Pose 2D")
    for i in nodes:
        if i!= main_id:
            posb_gt =  poses[i]["pos_func"](ts+t_calib[i])
            yaw_gt = poses[i]["ypr_func"](ts+t_calib[i])[:,0]
            
            posb_fused = poses_fused[i]["pos_func"](ts)
            
            dp_gt = posb_gt - posa_gt
            dp_fused = posb_fused - posa_fused
            for i in range(len(yawa_fused)):
                yaw = yawa_fused[i]
                dp_fused[i] = yaw_rotate_vec(-yaw, dp_fused[i])

            for i in range(len(yawa_fused)):
                yaw = yawa_gt[i]
                dp_gt[i] = yaw_rotate_vec(-yaw, dp_gt[i])
            
            plt.plot(dp_gt[:, 0], dp_gt[:, 1], label="Relative Pose GT")
            plt.plot(dp_fused[:, 0], dp_fused[:, 1], label="Relative Pose EST")
            plt.legend()
            plt.grid()
    
    fig = plt.figure("Relative Pose Polar")
    fig.suptitle("Relative Pose Polar")
    ax1, ax2 = fig.subplots(2, 1)
    for i in nodes:
        if i!= main_id:
            posb_gt =  poses[i]["pos_func"](ts+t_calib[i])
            yaw_gt = poses[i]["ypr_func"](ts+t_calib[i])[:,0]
            posb_fused = poses_fused[i]["pos_func"](ts)
            
            dp_gt = posb_gt - posa_gt
            dp_fused = posb_fused - posa_fused
            for i in range(len(yawa_fused)):
                yaw = yawa_fused[i]
                dp_fused[i] = yaw_rotate_vec(-yaw, dp_fused[i])

            for i in range(len(yawa_fused)):
                yaw = yawa_gt[i]
                dp_gt[i] = yaw_rotate_vec(-yaw, dp_gt[i])
            
            ax1.plot(ts, np.arctan2(dp_gt[:, 0], dp_gt[:, 1]), label="Relative Pose Angular GT")
            ax1.plot(ts, np.arctan2(dp_fused[:, 0], dp_fused[:, 1]), label="Relative Pose Angular Fused")

            ax2.plot(ts, norm(dp_gt, axis=1), label="Relative Pose Length GT")
            ax2.plot(ts, norm(dp_fused, axis=1), label="Relative Pose Length Fused")

            ax1.legend()
            ax1.grid()

            ax2.legend()
            ax2.grid()

    fig = plt.figure("Relative Pose")
    fig.suptitle("Relative Pose")
    ax1, ax2, ax3 = fig.subplots(3, 1)
    
    
    for i in nodes:
        if i!= main_id:
            posb_gt =  poses[i]["pos_func"](ts+t_calib[i])
            yaw_gt = poses[i]["ypr_func"](ts+t_calib[i])[:,0]
            
            posb_fused = poses_fused[i]["pos_func"](ts)
            #posb_vo = poses_fused[i]["pos"]
            
            dp_gt = posb_gt - posa_gt
            dp_fused = posb_fused - posa_fused
            #dp_vo = posb_vo - posa_vo
            for i in range(len(yawa_fused)):
                yaw = yawa_fused[i]
                dp_fused[i] = yaw_rotate_vec(-yaw, dp_fused[i])

            for i in range(len(yawa_fused)):
                yaw = yawa_gt[i]
                dp_gt[i] = yaw_rotate_vec(-yaw, dp_gt[i])
                
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
            posb_gt =  poses[i]["pos_func"](ts+t_calib[i])
            yaw_gt = poses[i]["ypr_func"](ts+t_calib[i])[:,0]
            
            posb_fused = poses_fused[i]["pos_func"](ts)
            
            dp_gt = posb_gt - posa_gt
            dp_fused = posb_fused - posa_fused

            rmse_x = RMSE(dp_gt[:,0] , dp_fused[:,0])
            rmse_y = RMSE(dp_gt[:,1] , dp_fused[:,1])
            rmse_z = RMSE(dp_gt[:,2] , dp_fused[:,2])

            ax1.plot(ts, dp_gt[:,0] - dp_fused[:,0], label="$E_{xfused}^" + str(i) + f"$ RMSE:{rmse_x:3.3f}")
            ax2.plot(ts, dp_gt[:,1] - dp_fused[:,1], label="$E_{yfused}^" + str(i) + f"$ RMSE:{rmse_y:3.3f}")
            ax3.plot(ts, dp_gt[:,2] - dp_fused[:,2], label="$E_{zfused}^" + str(i) + f"$ RMSE:{rmse_z:3.3f}")
            print(f"RMSE {main_id}->{i} {rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}")
            print(f"MEAN {main_id}->{i} {np.mean(dp_gt[:,0] - dp_fused[:,0]):3.3f},{np.mean(dp_gt[:,1] - dp_fused[:,1]):3.3f},{np.mean(dp_gt[:,2] - dp_fused[:,2]):3.3f}")
            
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

def plot_detections_error(poses, poses_vo, detections, nodes, main_id, t_calib):
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
    print("Total detection", len(detections))
    for det in detections:
        if det["id_a"] != main_id:
            continue
        yawa_gt = poses[det["id_a"]]["ypr_func"](det["ts"])[0]
        yawb_gt = poses[det["id_b"]]["ypr_func"](det["ts"])[0]

        posa_gt = poses[det["id_a"]]["pos_func"](det["ts"] + t_calib[det["id_b"]])
        posb_gt = poses[det["id_b"]]["pos_func"](det["ts"] + t_calib[det["id_b"]]) + yaw_rotate_vec(yawb_gt, np.array([-0.04, 0, 0.02]))

        posa_vo = poses_vo[det["id_a"]]["pos_raw_func"](det["ts"])
        yawa_vo = poses_vo[det["id_a"]]["ypr_raw_func"](det["ts"])[0]
        
        posa_gt = posa_gt + yaw_rotate_vec(yawa_gt, yaw_rotate_vec(-yawa_vo, det["pos_a"] - posa_vo))
        # posa_gt = posa_gt + yaw_rotate_vec(yawa_gt, np.array([-0.0616, -0.02, 0.11]))
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

    plt.figure("INV DEPS")
    plt.plot(ts_a, np.array(inv_deps), "+", label="INV DEP DET")
    plt.plot(ts_a, np.array(inv_deps_gt), "x", label="INV DEP GT")
    plt.legend()
    plt.grid()

    plt.figure("Self Pose Plot")
    plt.subplot(311)
    plt.title("VO X")
    plt.plot(poses_vo[1]["t"], poses_vo[1]["pos_raw"][:,0], label="VO1")
    plt.plot(poses_vo[2]["t"], poses_vo[2]["pos_raw"][:,0], label="VO2")
    plt.plot(ts_a, self_pos_a[:,0], 'o', label="SelfPose1")
    plt.plot(ts_a, self_pos_b[:,0], '+', label="SelfPose2")
    plt.legend()
    plt.grid()
    
    plt.subplot(312)
    plt.title("VO Y")
    plt.plot(poses_vo[1]["t"], poses_vo[1]["pos_raw"][:,1], label="VO1")
    plt.plot(poses_vo[2]["t"], poses_vo[2]["pos_raw"][:,1], label="VO2")
    plt.plot(ts_a, self_pos_a[:,1], 'o', label="SelfPose1")
    plt.plot(ts_a, self_pos_b[:,1], '+', label="SelfPose2")

    plt.legend()
    plt.grid()
    
    plt.subplot(313)
    plt.title("VO Z")
    plt.plot(poses_vo[1]["t"], poses_vo[1]["pos_raw"][:,2], label="VO1")
    plt.plot(poses_vo[2]["t"], poses_vo[2]["pos_raw"][:,2], label="VO2")
    plt.plot(ts_a, self_pos_a[:,2], 'o', label="SelfPose1")
    plt.plot(ts_a, self_pos_b[:,2], '+', label="SelfPose2")

    plt.figure("Yaw")
    plt.plot(ts_a, yawa_gts)


    # plt.legend()
    # plt.grid()
    plt.figure("Hist")
    plt.subplot(131)
    plt.hist(inv_dep_errs, 10, (-0.2, 0.2), density=True, facecolor='g', alpha=0.75)
    plt.subplot(132)
    plt.hist(dpos_errs[:,0], 50, (-0.1, 0.1), density=True, facecolor='g', alpha=0.75)
    # plt.subplot(143)
    # plt.hist(dpos_errs[:,1], 50, (-0.1, 0.1), density=True, facecolor='g', alpha=0.75)
    plt.subplot(133)
    plt.hist(dpos_errs[:,2], 50, (-0.1, 0.1), density=True, facecolor='g', alpha=0.75)
    

    filter_dpos = np.array(dpos_errs_norm) < 0.2
    print(f"Mean {np.mean(dpos_errs[filter_dpos], axis=0)}")

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
        # print(loop["id_a"], "->", loop["id_b"])
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
        if loop["ts_a"] > loop["ts_b"]:
            ts_a.append(loop["ts_a"])
        else:
            ts_a.append(loop["ts_b"])
        yawa_gts.append(yawa_gt)
        yawb_gts.append(yawb_gt)
        dyaw_errs.append(yawb_gt-yawa_gt-loop["dyaw"])
        
        # if np.linalg.norm(dpos_gt - dpos_loop) > 1.0:
        #     print("Error", np.linalg.norm(dpos_gt - dpos_loop) , loop)
        
    posa_gts = np.array(posa_gts)
    dpos_errs = np.array(dpos_errs)
    fig = plt.figure()

    plt.subplot(311)
    plt.plot(ts_a, dpos_errs_norm, 'x', label="Loop Error")
    plt.plot(ts_a, dpos_errs[:,0], '1', label="Loop Error X")
    plt.plot(ts_a, dpos_errs[:,1], '2', label="Loop Error Y")
    plt.plot(ts_a, dpos_errs[:,2], '3', label="Loop Error Z")
    plt.title("Error Pos Loop vs Vicon")
    plt.grid(which="both")
    plt.legend()

    plt.subplot(312)
    plt.plot(ts_a, dyaws, '.', label="DYaw Gt")
    plt.plot(ts_a, dyaw_gts, '+', label="DYaw Loop")
    plt.plot(ts_a, np.abs(dyaw_errs), "x", label="DYaw Error")
    
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


    print(f"Pos cov {np.cov(dpos_errs[:,0]):3.3f}, {np.cov(dpos_errs[:,1]):3.3f}, {np.cov(dpos_errs[:,2]):3.3f}")
    print(f"Yaw cov {np.cov(dyaw_errs)*57.3:3.3f}")


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