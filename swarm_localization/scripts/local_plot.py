#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
import numpy as np
from math import *
from scipy.interpolate import interp1d
from transformations import * 
import argparse
from numpy.linalg import norm
import scipy.stats as stats

plt.rc('figure', figsize=(10,5))
#plt.rc('figure', figsize=(20,15))

def quat2eulers(w, x, y ,z):
    r = atan2(2 * (w * x + y * z),
                    1 - 2 * (x * x + y * y))
    p = asin(2 * (w * y - z * x))
    y = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return y, p, r

def RMSE(predictions, targets):
    err_sq = (predictions-targets)**2
    err_sq = err_sq[~numpy.isnan(err_sq)]
    return np.sqrt(np.mean(err_sq))

def ATE_POS(predictions, targets):
    err = predictions-targets
    norm2 = err[:,0]*err[:,0]+err[:,1]*err[:,1]+err[:,2]*err[:,2]
    return np.sqrt(np.mean(norm2))

def yaw_rotate_vec(yaw, vec):
    Re = rotation_matrix(yaw, [0, 0, 1])[0:3, 0:3]
    return np.transpose(np.dot(Re, np.transpose(vec)))

def read_pose_swarm_fused(bag, topic, _id, t0):
    pos = []
    ypr = []
    ts = []
    quat = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        for i in range(len(msg.ids)):
            _i = msg.ids[i]
            if _i == _id:
                ts.append(msg.header.stamp.to_sec() - t0)
                pos.append([msg.local_drone_position[i].x, msg.local_drone_position[i].y, msg.local_drone_position[i].z])
                ypr.append([msg.local_drone_yaw[i], 0, 0])
                quat.append(quaternion_from_euler(0, 0, msg.local_drone_yaw[i]))

    ret = {
        "t": np.array(ts) ,
        "pos": np.array(pos),
        "ypr": np.array(ypr),
        "quat": np.array(quat)
    }
    return ret

def read_pose_swarm_frame(bag, topic, _id, t0):
    pos = []
    ypr = []
    ts = []
    quat = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if msg.header.stamp.to_sec() < t0:
                continue
        for node in msg.node_frames:
            _i = node.id
            if _i == _id and node.vo_available:
                ts.append(node.header.stamp.to_sec() - t0)
                pos.append([node.position.x, node.position.y, node.position.z])
                ypr.append([node.yaw, 0, 0])
                quat.append(quaternion_from_euler(0, 0, node.yaw))
    ret = {
        "t": np.array(ts),
        "pos_raw": np.array(pos),
        "pos": np.array(pos),
        "quat": np.array(quat),
        "pos_raw_func": interp1d(ts, pos,axis=0,bounds_error=False,fill_value="extrapolate"),
        "ypr_raw": np.array(ypr),
        "ypr_raw_func": interp1d(ts, ypr,axis=0,bounds_error=False,fill_value="extrapolate"),
    }
    return ret

# def read_distance_swarm_frame(bag, topic, _id, to)
def read_distances_swarm_frame(bag, topic, t0, main_id):
    distances = {
    }
    ts = []
    print(f"Read distances from topic {topic}")
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
    print(f"Read distances from topic {topic}")
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


def read_pose(bag, topic, t0):
    pos = []
    ypr = []
    ts = []
    quat = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t0 == 0:
            t0 = msg.header.stamp.to_sec()
        else:
            if msg.header.stamp.to_sec() < t0:
                continue
        p = msg.pose.position
        q = msg.pose.orientation
        pos.append([p.x, p.y, p.z])
        y, p, r = quat2eulers(q.w, q.x, q.y, q.z)
        quat.append([q.w, q.x, q.y, q.z])
        ypr.append([y, p, r])
        ts.append(msg.header.stamp.to_sec() - t0)
    ret = {
        "t": np.array(ts),
        "pos": np.array(pos),
        "pos_func": interp1d(ts, pos,axis=0,bounds_error=False,fill_value="extrapolate"),
        "ypr": np.array(ypr),
        "ypr_func": interp1d(ts, ypr,axis=0,bounds_error=False,fill_value="extrapolate"),
        "quat": np.array(quat)
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
    if path is not None:
        return parse_path(path, t0)
    return None

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
            "dyaw":msg.dyaw,
            "pnp_inlier_num": msg.pnp_inlier_num
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

def read_detections_raw(bag, t0, topic="/swarm_detection/swarm_detected_raw"):
    dets = []
    for topic, _msg, t in bag.read_messages(topics=[topic]):
        for msg in _msg.detected_nodes_xyz_yaw:
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

def output_pose_to_csv(filename, poses, skip = 1):
    with open(filename, 'w') as writer:
        for i in range(len(poses["t"])):
            if i % skip == 0:
                ts = poses["t"][i]
                t = poses["pos"][i]
                q = poses["quat"][i]
                writer.write(f"{ts} {t[0]} {t[1]} {t[2]} {q[1]} {q[2]} {q[3]} {q[0]}\n")


def bag2dataset(bagname, nodes = [1, 2], alg="fused", is_pc=False, main_id=1, trial = 0):
    bag = rosbag.Bag(bagname)
    poses = {}
    poses_fused = {}
    poses_vo = {}
    poses_path = {}
    t0 = 0
    plat = "pc"
    for i in nodes:
        poses[i], t0 = read_pose(bag, f"/SwarmNode{i}/pose", t0)
        output_pose_to_csv(f"data/{plat}/vio/{plat}_vio_drone{i}/stamped_groundtruth.txt", poses[i])
        output_pose_to_csv(f"data/{plat}/{alg}/{plat}_{alg}_drone{i}/stamped_groundtruth.txt", poses[i])
        if is_pc:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused_pc", i, t0)
            output_pose_to_csv(f"data/{plat}/{alg}/{plat}_{alg}_drone{i}/stamped_traj_estimate{trial}.txt", poses_fused[i])
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path_pc", t0)
        else:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused", i, t0)
            output_pose_to_csv(f"data/{plat}/{alg}/{plat}_{alg}_drone{i}/stamped_traj_estimate{trial}.txt", poses_fused[i])
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path", t0)

        poses_fused[i]["t"] = poses_fused[i]["t"]
        poses_vo[i] = read_pose_swarm_frame(bag, "/swarm_drones/swarm_frame_predict", i, t0)
        output_pose_to_csv(f"data/{plat}/vio/{plat}_vio_drone{i}/stamped_traj_estimate{trial}.txt", poses_vo[i], 10)

def bag_read(bagname, nodes = [1, 2], is_pc=False, main_id=1, groundtruth = True):
    bag = rosbag.Bag(bagname)
    poses = {}
    poses_fused = {}
    poses_vo = {}
    poses_path = {}
    t0 = 0
    plat = "pc"
    
    for topic, msg, t in bag.read_messages(topics=["/swarm_drones/swarm_frame"]):
        if len(msg.node_frames) >= len(nodes):
            t0 = msg.header.stamp.to_sec()
            # print(t0, msg)
            break
    
    for i in nodes:
        if groundtruth:
            poses[i], t0 = read_pose(bag, f"/SwarmNode{i}/pose", t0)
        if is_pc:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused_pc", i, t0)
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path_pc", t0)
        else:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused", i, t0)
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path", t0)

        poses_fused[i]["t"] = poses_fused[i]["t"]
        poses_vo[i] = read_pose_swarm_frame(bag, "/swarm_drones/swarm_frame", i, t0)

        if poses_path[i] is None:
            poses_path[i] = poses_fused[i]

    loops = read_loops(bag, t0, "/swarm_loop/loop_connection")
    # detections = read_detections(bag, t0, "/swarm_drones/node_detected")
    detections = read_detections_raw(bag, t0)
    distances = read_distances_swarm_frame(bag, "/swarm_drones/swarm_frame", t0, main_id)
    bag.close()
    if groundtruth:
        fused_offset = poses[main_id]["pos"][0] - poses_fused[main_id]["pos"][0]
        yaw_offset = (poses[main_id]["ypr"][0] - poses_fused[main_id]["ypr"][0])[0]
        print("Yaw Offset, ", yaw_offset*57.3, "Fused Offset", fused_offset)
    else:
        fused_offset = np.array([0, 0, 0])
        yaw_offset = 0

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
        if groundtruth:
            vo_offset = poses[i]["pos"][0] - poses_vo[i]["pos_raw"][0]
            yaw_offset = (poses[i]["ypr"][0] - poses_vo[i]["ypr_raw"][0])[0]
            print(f"VIO Offset for {i}: {vo_offset}")
            print(poses[i]["pos"][0], poses_vo[i]["pos_raw"][0])
        else:    
            vo_offset = np.array([0, 0, 0])
            yaw_offset = 0
        poses_vo[i]["pos"] = yaw_rotate_vec(yaw_offset, poses_vo[i]["pos_raw"]) + vo_offset
        poses_vo[i]["ypr"] = poses_vo[i]["ypr_raw"] + np.array([yaw_offset, 0, 0])
        poses_vo[i]["pos_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["pos"],axis=0,bounds_error=False,fill_value="extrapolate")
        poses_vo[i]["ypr_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["ypr"],axis=0,fill_value="extrapolate")
    if groundtruth:
        return poses, poses_fused, poses_vo, poses_path, loops, detections, distances
    else:
        return poses_fused, poses_vo, poses_path, loops, detections, distances

def plot_fused(poses, poses_fused, poses_vo, poses_path, loops, detections, nodes, t_calib = {1:0, 2:0}, groundtruth = True):
    fig = plt.figure("Traj2", figsize=(6, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax = fig.gca(projection='3d')
    
    for i in nodes:
        # ax.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1],poses[i]["pos"][:,2], label=f" Traj{i}")
        ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1],poses_fused[i]["pos"][:,2], label=f"Estimate {i}")
    
    ax.set_xlabel('$X$')
    ax.set_ylabel('$Y$')
    ax.set_zlabel('$Z$')
    
    #Plot Loops
    quivers = []
    quivers_det = []
    for loop in loops:
        posa_ = poses_path[loop["id_a"]]["pos_func"](loop["ts_a"])
        posb_ = poses_path[loop["id_b"]]["pos_func"](loop["ts_b"])
        if norm(loop["dpos"]) < 2.0 and norm(posb_-posa_) < 2.0:
            quivers.append([posa_[0], posa_[1], posa_[2], posb_[0]-posa_[0], posb_[1]-posa_[1], posb_[2]-posa_[2]])
            #quivers.append([posa_[0], posa_[1], posa_[2], loop["dpos"][0], loop["dpos"][1], loop["dpos"][2]])

    
    for det in detections:
        posa_ = poses_path[det["id_a"]]["pos_func"](det["ts"])
        yawa_ = poses_path[det["id_a"]]["ypr_func"](det["ts"])[0]
        dpos = yaw_rotate_vec(yawa_, det["dpos"]/det["inv_dep"])
        quivers_det.append([posa_[0], posa_[1], posa_[2], dpos[0], dpos[1], dpos[2]])
    
    quivers = np.array(quivers)
    quivers_det = np.array(quivers_det)

    # c = np.arctan2(quivers[:,4], quivers[:,3])
    # c = (c.ravel() - c.min()) / c.ptp()
    # c = np.concatenate((c, np.repeat(c, 2)))
    # c = plt.cm.hsv(c)

    step = 2
    if len(quivers) > 0:   
        ax.quiver(quivers[::step,0], quivers[::step,1], quivers[::step,2], quivers[::step,3], quivers[::step,4], quivers[::step,5], 
            arrow_length_ratio=0.1, color="green",linewidths=1.0)

    step_det = 10
    if len(quivers_det) > 0:   
        ax.quiver(quivers_det[::step_det,0], quivers_det[::step_det,1], quivers_det[::step_det,2], quivers_det[::step_det,3],
            quivers_det[::step_det,4], quivers_det[::step_det,5], 
            arrow_length_ratio=0.1, color="red",linewidths=1.0)

    plt.legend()
    plt.savefig("/home/xuhao/output/Traj2.png")

    #Plot Fused Vs GT 3D
    fig = plt.figure("FusedVsGT3D")
    # fig.suptitle("Fused Vs GT 3D")
    for k in range(len(nodes)):
        i = nodes[k]
        ax = fig.add_subplot(1, len(nodes), k+1, projection='3d')
        ax.set_title(f"Traj {i}, length: {poses_length(poses_fused[i]):3.3f}")
        if groundtruth:
            ax.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1],poses[i]["pos"][:,2], label=f"Ground Truth ${i}$")
        ax.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1],poses_vo[i]["pos"][:,2], label=f"Aligned VIO ${i}$")
        ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1],poses_fused[i]["pos"][:,2], label=f"Estimate ${i}$")
        
        plt.legend()
        ax.set_xlabel('$X$')
        ax.set_ylabel('$Y$')
        ax.set_zlabel('$Z$')
    plt.savefig("/home/xuhao/output/FusedVsGT3D.pdf")

    fig = plt.figure("Fused Multi 2d")
    plt.gca().set_aspect('equal')
    
    step_det = 10
    qview_width = 0.9
    if len(quivers) > 0:   
        for i in range(0,len(quivers),step_det):
            xs = [quivers[i,0],quivers[i,0]+quivers[i,3]]
            ys = [quivers[i,1], quivers[i,1]+quivers[i,4]]
            if i == 0:
                plt.plot(xs, ys, color="black", label="Map-based edges", linewidth=qview_width)
            else:
                plt.plot(xs, ys, color="black", linewidth=qview_width)
    step_det = 10
    if len(quivers_det) > 0: 
        for i in range(0,len(quivers_det),step_det):
            xs = [quivers_det[i,0],quivers_det[i,0]+quivers_det[i,3]]
            ys = [quivers_det[i,1], quivers_det[i,1]+quivers_det[i,4]]
            if i == 0:
                plt.plot(xs, ys, color="gray", label="Detection", linewidth=qview_width)
            else:
                plt.plot(xs, ys, color="gray", linewidth=qview_width)
    for i in nodes:
        plt.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], label=f"Estimation {i}")
        #plt.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Estimate {i}")
    for i in nodes:
        # plt.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1], label=f"VIO ${i}$", alpha=0.7)
        final_vio = norm(poses_vo[i]["pos"][-1,:])
        final_path = norm(poses_path[i]["pos"][-1,:])
        total_len = poses_length(poses_fused[i])
        plt.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1], label=f"Ground Truth {i}")
        
        print(f"Final drift {i} VIO {final_vio:3.2f}m {final_vio/total_len*100:3.1f}% Fused {final_path:3.2f}m {final_path/total_len*100:3.1f}%")
    
    plt.legend()
    plt.grid()
    plt.savefig("/home/xuhao/output/fused2d.pdf")

    fig = plt.figure("Fused Vs GT 2D")
    fig.suptitle("Fused Vs GT 2D")
    for k in range(len(nodes)):
        i = nodes[k]
        ax = fig.add_subplot(1, len(nodes), k+1)
        if groundtruth:
            ax.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1], label=f"Ground Truth {i}")
        ax.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1], label=f"VIO {i}")
        # ax.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")
        ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Estimation {i}")
        # ax.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Fused Oline{i}")
        plt.grid()
        plt.legend()

    plt.savefig("/home/xuhao/output/fusedvsgt2d.pdf")
    for i in nodes:
        fig = plt.figure(f"Drone {i} fused Vs GT 1D")
        #fig.suptitle(f"Drone {i} fused Vs GT 1D")
        ax1, ax2, ax3 = fig.subplots(3, 1)

        t_ = poses_fused[i]["t"]
        if groundtruth:
            pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"] + t_calib[i])
        pos_fused = poses_fused[i]["pos"]
        _i = str(i) 
        if groundtruth:
            ax1.plot(t_, pos_gt[:,0], label=f"Ground Truth ${i}$")
        # ax1.plot(poses_path[i]["t"], poses_path[i]["pos"][:,0], '.', label=f"Fused Offline Traj{i}")
        ax1.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,0], label=f"Aligned VO Traj{i}")
        ax1.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,0], label=f"Estimate {i}")
        ax1.tick_params( axis='x', which='both', bottom=False, top=False, labelbottom=False) 
        ax1.set_ylabel("x")

        if groundtruth:
            ax2.plot(t_, pos_gt[:,1], label=f"Ground Truth ${i}$")
        #ax2.plot(poses_path[i]["t"], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")
        ax2.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,1], label=f"Aligned VO Traj{i}")
        ax2.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,1], label=f"Estimate {i}")
        ax2.tick_params( axis='x', which='both', bottom=False, top=False, labelbottom=False) 
        ax2.set_ylabel("y")

        if groundtruth:
            ax3.plot(t_, pos_gt[:,2], label=f"Ground Truth ${i}$")
        #ax3.plot(poses_path[i]["t"], poses_path[i]["pos"][:,2], '.', label=f"Fused Offline Traj{i}")
        ax3.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,2], label=f"Aligned VIO ${i}$")
        ax3.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,2], label=f"Estimate {i}")
        ax3.set_ylabel("z")
        ax3.set_xlabel("t")

        # ax1.legend()
        # ax2.legend()
        ax3.legend()
        ax1.grid()
        ax2.grid()
        ax3.grid()
        plt.savefig(f"/home/xuhao/output/est_by_t{i}.png")

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

def plot_distance_err(poses, poses_fused, poses_vo, poses_path, distances, main_id, nodes):

    for i in nodes:
        if i == main_id:
            continue
        fig = plt.figure(f"Distance {i}")
        fig.suptitle(f"Distance {i}")
        ax1, ax2, ax3 = fig.subplots(3, 1)
        t_ = np.array(distances[i]["t"])
        pos_gt = poses[i]["pos_func"](t_)
        pos_fused = poses_fused[i]["pos_func"](t_)

        main_pos_gt = poses[main_id]["pos_func"](t_)
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
        # z = np.polyfit(dis_gt, dis_raw, 1)
        z = np.polyfit(dis_raw, dis_gt, 1)
        print(f"Fit {z[0]}, {z[1]}")
        ax1.legend()
        ax1.grid()
        
        ax2.legend()
        ax2.grid()
        
        ax3.legend()
        ax3.grid()
        plt.show()

        dis_calibed = 0.257+0.734*np.array(dis_raw)
        dis_calibed = np.array(dis_raw) - 0.257
        err_calibed_filter = np.fabs(dis_gt-dis_calibed) < 1.0
        err_calibed = (dis_gt-dis_calibed)[err_calibed_filter]

        plt.figure(f"Dist Hist {i}")
        plt.hist(err_calibed, 50, (-0.5, 0.5), density=True, facecolor='g', alpha=0.75)
        mu, std = stats.norm.fit(err_calibed)
        xmin, xmax = plt.xlim()
        x = np.linspace(xmin, xmax, 100)
        p = stats.norm.pdf(x, mu, std)
        plt.plot(x, p, 'k', linewidth=2)
        title = "mu = %.2f,  std = %.2f" % (mu, std)
        plt.title(title)
        plt.show()

    

def plot_relative_pose_err(poses, poses_fused, poses_vo, main_id, target_id, t_calib = {1:0, 2:0}, groundtruth = True):
    ts = poses_fused[main_id]["t"]
    posa_vo =  poses_vo[main_id]["pos_func"](ts)
    posa_fused = poses_fused[main_id]["pos_func"](ts)
    yawa_fused = poses_fused[main_id]["ypr_func"](ts)[:,0]
    yawa_vo = poses_vo[main_id]["ypr_func"](ts)[:,0]


    posb_vo =  poses_vo[target_id]["pos_func"](ts+t_calib[target_id])
    posb_fused = poses_fused[target_id]["pos_func"](ts)
    yawb_fused = poses_fused[target_id]["ypr_func"](ts)[:, 0]
    yawb_vo = poses_vo[target_id]["ypr_func"](ts)[:, 0]
    
    dp_fused = posb_fused - posa_fused
    dp_vo = posb_vo - posa_vo
    if groundtruth:
        posa_gt =  poses[main_id]["pos_func"](ts)
        yawa_gt = poses[main_id]["ypr_func"](ts)[:,0]
        posb_gt =  poses[target_id]["pos_func"](ts+t_calib[target_id])
        yawb_gt = poses[target_id]["ypr_func"](ts+t_calib[target_id])[:,0]
        dp_gt = posb_gt - posa_gt
    
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

    fig = plt.figure(f"Relative Pose 2D {main_id}->{target_id}")

    if groundtruth:
        plt.plot(dp_gt[:, 0], dp_gt[:, 1], label="Relative Pose GT")
    plt.plot(dp_fused[:, 0], dp_fused[:, 1], label="Relative Pose EST")
    plt.legend()
    plt.grid()

    fig = plt.figure("Relative Pose Polar")
    fig.suptitle("Relative Pose Polar")
    ax1, ax2 = fig.subplots(2, 1)

    if groundtruth:
        ax1.plot(ts, np.arctan2(dp_gt[:, 0], dp_gt[:, 1]), label="Relative Pose Angular GT")
    ax1.plot(ts, np.arctan2(dp_fused[:, 0], dp_fused[:, 1]), label="Relative Pose Angular Fused")

    if groundtruth:
        ax2.plot(ts, norm(dp_gt, axis=1), label="Relative Pose Length GT")
    ax2.plot(ts, norm(dp_fused, axis=1), label="Relative Pose Length Fused")

    ax1.legend()
    ax1.grid()
    ax2.legend()
    ax2.grid()

    fig = plt.figure("Relative Pose")
    fig.suptitle("Relative Pose")
    ax1, ax2, ax3 = fig.subplots(3, 1)

    if groundtruth:
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

    if groundtruth:
        rmse_yaw = RMSE(yawb_fused - yawa_fused, yawb_gt - yawa_gt)
        rmse_x = RMSE(dp_gt[:,0] , dp_fused[:,0])
        rmse_y = RMSE(dp_gt[:,1] , dp_fused[:,1])
        rmse_z = RMSE(dp_gt[:,2] , dp_fused[:,2])


        rmse_x_no_bias = RMSE(dp_gt[:,0] - np.mean(dp_gt[:,0] - dp_fused[:,0]), dp_fused[:,0])
        rmse_y_no_bias = RMSE(dp_gt[:,1] - np.mean(dp_gt[:,1] - dp_fused[:,1]), dp_fused[:,1])
        rmse_z_no_bias = RMSE(dp_gt[:,2] - np.mean(dp_gt[:,2] - dp_fused[:,2]), dp_fused[:,2])

        ax1.plot(ts, dp_gt[:,0] - dp_fused[:,0], label="$E_{xfused}^" + str(i) + f"$ RMSE:{rmse_x:3.3f}")
        ax2.plot(ts, dp_gt[:,1] - dp_fused[:,1], label="$E_{yfused}^" + str(i) + f"$ RMSE:{rmse_y:3.3f}")
        ax3.plot(ts, dp_gt[:,2] - dp_fused[:,2], label="$E_{zfused}^" + str(i) + f"$ RMSE:{rmse_z:3.3f}")
        print(f"RMSE {main_id}->{target_id} {rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f} yaw {rmse_yaw*180/pi:3.3} deg")
        print(f"BIAS {main_id}->{target_id} {np.mean(dp_gt[:,0] - dp_fused[:,0]):3.3f},{np.mean(dp_gt[:,1] - dp_fused[:,1]):3.3f},{np.mean(dp_gt[:,2] - dp_fused[:,2]):3.3f}")
        print(f"RMSE NO BIAS {main_id}->{target_id} {rmse_x_no_bias:3.3f},{rmse_y_no_bias:3.3f},{rmse_z_no_bias:3.3f}")

        rmse_yaw = RMSE(yawb_vo - yawa_vo, yawb_gt - yawa_gt)
        rmse_x = RMSE(dp_gt[:,0] , dp_vo[:,0])
        rmse_y = RMSE(dp_gt[:,1] , dp_vo[:,1])
        rmse_z = RMSE(dp_gt[:,2] , dp_vo[:,2])

        print(f"VO RMSE {main_id}->{target_id} {rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f} yaw {rmse_yaw*180/pi:3.3} deg")
        print(f"VO BIAS {main_id}->{target_id} {np.mean(dp_gt[:,0] - dp_vo[:,0]):3.3f},{np.mean(dp_gt[:,1] - dp_vo[:,1]):3.3f},{np.mean(dp_gt[:,2] - dp_vo[:,2]):3.3f}")
        
        ax1.legend()
        ax2.legend()
        ax3.legend()
        ax1.grid()
        ax2.grid()
        ax3.grid()
    plt.show()
    
    


def plot_fused_err(poses, poses_fused, poses_vo, poses_path, nodes, main_id=1, t_calib = {1:0, 2:0}):
    #Plot Fused Vs GT absolute error
    for i in nodes:
        fig = plt.figure(f"Fused Absolute Error {i}")
        fig.suptitle(f"Fused Absolute Error {i}")
        ax1, ax2, ax3 = fig.subplots(3, 1)
        t_ = poses_fused[i]["t"]
        pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"]+t_calib[i])
        pos_fused = poses_fused[i]["pos"]
        yaw_fused = poses_fused[i]["ypr"][:,0]
        pos_vo = poses_vo[i]["pos"]
        pos_path = poses_path[i]["pos"]
        yaw_gt = poses[i]["ypr_func"](poses_fused[i]["t"]+t_calib[i])[:,0]
        yaw_vo = poses_vo[i]["ypr"][:,0]
        _i = str(i) 


        rmse_x = RMSE(pos_gt[:,0] , pos_fused[:,0])
        rmse_y = RMSE(pos_gt[:,1] , pos_fused[:,1])
        rmse_z = RMSE(pos_gt[:,2] , pos_fused[:,2])

        ate_fused = ATE_POS(pos_fused, pos_gt)
        rmse_yaw_fused = RMSE(yaw_gt, yaw_fused)

        label = f"$errx_{i}$ RMSE{i}:{rmse_x:3.3f}"
        ax1.plot(t_, pos_gt[:,0]  - pos_fused[:,0], label=label)

        label = f"$erry_{i}$ RMSE{i}:{rmse_y:3.3f}"
        ax2.plot(t_, pos_gt[:,1]  - pos_fused[:,1], label=label)

        label = f"$erry_{i}$ RMSE{i}:{rmse_z:3.3f}"
        ax3.plot(t_,  pos_gt[:,2]  - pos_fused[:,2], label=label)

        pos_gt =  poses[i]["pos_func"](poses_vo[i]["t"]+t_calib[i])
        yaw_gt =  poses[i]["ypr_func"](poses_vo[i]["t"]+t_calib[i])[:,0]
        
        rmse_vo_x = RMSE(pos_vo[:,0] , pos_gt[:,0])
        rmse_vo_y = RMSE(pos_vo[:,1] , pos_gt[:,1])
        rmse_vo_z = RMSE(pos_vo[:,2] , pos_gt[:,2])
        
        label = f"$VO errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
        ax1.plot(poses_vo[i]["t"], pos_gt[:,0]  - pos_vo[:,0], label=label)

        label = f"$VO erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
        ax2.plot(poses_vo[i]["t"], pos_gt[:,1]  - pos_vo[:,1], label=label)
        
        label = f"$VO errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
        ax3.plot(poses_vo[i]["t"], pos_gt[:,2]  - pos_vo[:,2], label=label)
        ate_vo = ATE_POS(pos_vo, pos_gt)
        rmse_yaw_vo = RMSE(yaw_gt, yaw_vo)


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
        print(f"Drone {i} estimated by {main_id}")
        print(f"ATE Fused Pos{ate_fused:3.3f} Yaw {rmse_yaw_fused*180/pi:3.3f} deg RMSE Fused Online {rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}")
        print(f"RMSE Fused Offline Path {rmse_path_x:3.3f},{rmse_path_y:3.3f},{rmse_path_z:3.3f}")
        print(f"ATE VO {ate_vo:3.3f}  Yaw {rmse_yaw_vo*180/pi:3.3f} deg RMSE VO {rmse_vo_x:3.3f},{rmse_vo_y:3.3f},{rmse_vo_z:3.3f}")

        ax3.legend()
        ax1.grid()
        ax2.grid()
        ax3.grid()



    
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

def plot_detections_error(poses, poses_vo, detections, nodes, main_id, t_calib, enable_dpose):
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
        posb_gt = poses[det["id_b"]]["pos_func"](det["ts"] + t_calib[det["id_b"]])# + yaw_rotate_vec(yawb_gt, np.array([-0.04, 0, 0.02]))

        posa_vo = poses_vo[det["id_a"]]["pos_raw_func"](det["ts"])
        yawa_vo = poses_vo[det["id_a"]]["ypr_raw_func"](det["ts"])[0]
        
        if enable_dpose:
            posa_gt = posa_gt + yaw_rotate_vec(yawa_gt, yaw_rotate_vec(-yawa_vo, det["pos_a"] - posa_vo))

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


    plt.figure("INV DEPS")
    plt.title("INV DEPS")
    plt.plot(ts_a, np.array(inv_deps), "+", label="INV DEP DET")
    plt.plot(ts_a, np.array(inv_deps_gt), "x", label="INV DEP GT")
    plt.legend()
    plt.grid()

    plt.figure("INV DEPS ERR")
    plt.title("INV DEPS ERR")
    plt.plot(ts_a, np.array(inv_deps) - np.array(inv_deps_gt), "+", label="INV DEP DET")
    plt.legend()
    plt.grid()

    plt.figure("INV DEPS ERR HIST")
    plt.hist(np.array(inv_deps) - np.array(inv_deps_gt), 50, (-0.3, 0.3), density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(np.array(inv_deps) - np.array(inv_deps_gt))
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "INV DEPS ERR  Fit results: mu = %.2f,  std = %.2f" % (mu, std)
    print("INV DEPS ERR Variance", np.mean((np.array(inv_deps) - np.array(inv_deps_gt))**2))

    plt.title(title)


    plt.legend()
    plt.grid()

    plt.figure("DEPS")
    plt.title("DEPS")
    plt.plot(ts_a, 1/np.array(inv_deps), "+", label="DEP DET")
    plt.plot(ts_a, 1/np.array(inv_deps_gt), "x", label="DEP GT")
    plt.legend()
    plt.grid()


    plt.figure("DEPS ERR")
    plt.plot(ts_a, 1/np.array(inv_deps) - 1/np.array(inv_deps_gt), "+", label="DEP ERR")
    plt.legend()
    plt.grid()

    plt.figure("DEPS ERR HIST")
    mu, std = stats.norm.fit(1/np.array(inv_deps) - 1/np.array(inv_deps_gt))
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.hist(1/np.array(inv_deps) - 1/np.array(inv_deps_gt), 50, (-0.5, 0.5), density=True, facecolor='g', alpha=0.75)
    xmin, xmax = plt.xlim()
    plt.plot(x, p, 'k', linewidth=2)
    title = "DEPS ERR Fit results: mu = %.2f,  std = %.2f" % (mu, std)
    plt.title(title)
    plt.legend()
    plt.grid()
    print("Dep ERR MEAN", np.mean(1/np.array(inv_deps_gt) - 1/np.array(inv_deps)))
    print("DEPS ERR Variance", np.mean((1/np.array(inv_deps) - 1/np.array(inv_deps_gt))**2))

    
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



def plot_loops_error(poses, loops, nodes):
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
        pnp_inlier_nums.append(loop["pnp_inlier_num"])
        idas.append(loop["id_a"])
        idbs.append(loop["id_b"])

        # if np.linalg.norm(dpos_gt - dpos_loop) > 1.0:
        #     print("Error", np.linalg.norm(dpos_gt - dpos_loop) , loop)
    outlier_num = (np.array(dpos_errs_norm)>0.5).sum()
    total_loops = len(dpos_errs_norm)
    print(f"Outlier rate {outlier_num/total_loops*100:3.2f}% total loops {total_loops} outlier_num {outlier_num}")
    posa_gts = np.array(posa_gts)
    dpos_errs = np.array(dpos_errs)
    dyaw_errs = np.array(dyaw_errs)
    distances = np.array(distances)
    fig = plt.figure("Loop Error")
    plt.subplot(411)
    plt.plot(ts_a, dpos_errs_norm, 'x', label="Loop Error")
    plt.plot(ts_a, dpos_errs[:,0], '1', label="Loop Error X")
    plt.plot(ts_a, dpos_errs[:,1], '2', label="Loop Error Y")
    plt.plot(ts_a, dpos_errs[:,2], '3', label="Loop Error Z")
    plt.title("Error Pos Loop vs Vicon")
    plt.grid(which="both")
    plt.legend()

    plt.subplot(412)
    plt.plot(ts_a, dyaws, '.', label="DYaw Gt")
    plt.plot(ts_a, dyaw_gts, '+', label="DYaw Loop")
    plt.plot(ts_a, np.abs(dyaw_errs), "x", label="DYaw Error")
    
    plt.title("Error Yaw Loop vs Vicon")
    plt.grid(which="both")
    plt.legend()

    plt.subplot(413)
    plt.plot(ts_a, pnp_inlier_nums, "x", label="pnp_inlier_nums")
    plt.grid()


    plt.subplot(414)
    plt.plot(ts_a, posa_gts[:,0], '+', label="Vicon X")
    plt.plot(ts_a, posa_gts[:,1], '+', label="Vicon Y")
    plt.plot(ts_a, posa_gts[:,2], '+', label="Vicon Z")
    # plt.plot(poses[i]["t"], poses[i]["pos"][:,0], label="Vicon X")
    # plt.plot(poses[i]["t"], poses[i]["pos"][:,1], label="Vicon Y")
    # plt.plot(poses[i]["t"], poses[i]["pos"][:,2], label="Vicon Z")

    plt.grid(which="both")
    plt.legend()

    plt.figure("InliersVSErr")
    plt.title("InliersVSErr")
    plt.plot(pnp_inlier_nums, dpos_errs_norm, "x", label="")
    plt.grid(which="both")
    for i in range(len(pnp_inlier_nums)):
        plt.text(pnp_inlier_nums[i], dpos_errs_norm[i] + 0.2, f"{idas[i]}->{idbs[i]}", fontsize=12)
    # plt.figure()
    # plt.subplot(141)
    # plt.hist(dpos_errs_norm, 5, density=True, facecolor='g', alpha=0.75)

    plt.figure("DistanceVSErr")
    plt.title("DistancVSErr")
    plt.plot(dpos_loop_norms, dpos_errs_norm, "x", label="")
    plt.grid(which="both")
    # for i in range(len(pnp_inlier_nums)):
    #     plt.text(distances[i], dpos_errs_norm[i] + 0.2, f"{idas[i]}->{idbs[i]}", fontsize=12)

    plt.figure("Loop Hist")
    plt.subplot(131)
    plt.hist(dpos_errs[:,0], 50, density=True, facecolor='g', alpha=0.75)

    mu, std = stats.norm.fit(dpos_errs[:,0])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
    plt.title(title)

    plt.subplot(132)
    plt.hist(dpos_errs[:,1], 50, density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(dpos_errs[:,1])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "mu = %.2f,  std = %.2f" % (mu, std)
    plt.title(title)

    plt.subplot(133)
    plt.hist(dpos_errs[:,2], 50, density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(dpos_errs[:,2])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
    plt.title(title)

    print(f"Pos cov {np.cov(dpos_errs[:,0]):3.3f}, {np.cov(dpos_errs[:,1]):3.3f}, {np.cov(dpos_errs[:,2]):3.3f}")
    print(f"Yaw cov {np.cov(dyaw_errs)*57.3:3.3f}")

    _cov_distances = np.linspace(0.2,1.8,20)
    _cov_pos = []
    _cov_yaw = []
    for _mid in _cov_distances:
        thres_low = _mid - 0.2
        thres_high = _mid + 0.2
        mask = np.where((dpos_loop_norms > thres_low) & (dpos_loop_norms < thres_high), True, False)
        _dpos_err = dpos_errs[mask,:]
        _dyaw_err = dyaw_errs[mask]

        # mu, std = stats.norm.fit(_dpos_err[:,0])
        # xmin, xmax = plt.xlim()
        # x = np.linspace(xmin, xmax, 100)
        # p = stats.norm.pdf(x, mu, std)
        # plt.plot(x, p, 'k', linewidth=2)
        # title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
        # plt.title(title)

        # plt.subplot(132)
        # plt.hist(_dpos_err[:,1], 50, density=True, facecolor='g', alpha=0.75)
        # mu, std = stats.norm.fit(_dpos_err[:,1])
        # xmin, xmax = plt.xlim()
        # x = np.linspace(xmin, xmax, 100)
        # p = stats.norm.pdf(x, mu, std)
        # plt.plot(x, p, 'k', linewidth=2)
        # title = "mu = %.2f,  std = %.2f" % (mu, std)
        # plt.title(title)

        # plt.subplot(133)
        # plt.hist(_dpos_err[:,2], 50, density=True, facecolor='g', alpha=0.75)
        # mu, std = stats.norm.fit(_dpos_err[:,2])
        # xmin, xmax = plt.xlim()
        # x = np.linspace(xmin, xmax, 100)
        # p = stats.norm.pdf(x, mu, std)
        # plt.plot(x, p, 'k', linewidth=2)
        # title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
        # plt.title(title)
        cov = [np.cov(_dpos_err[:,0]), np.cov(_dpos_err[:,1]), np.cov(_dpos_err[:,2])]
        _cov_pos.append(sqrt(norm(cov)))
        _cov_yaw.append(sqrt(np.cov(_dyaw_err)))
        print(f"{_mid} Pos cov {(cov)}, std {sqrt(norm(cov)):3.3f} Yaw cov {sqrt(np.cov(_dyaw_err)):3.3f}")
    
    plt.figure()
    plt.subplot(211)
    plt.title("PosStd by Distance")
    plt.plot(_cov_distances, _cov_pos, "+")
    plt.grid()
    plt.subplot(212)
    plt.title("PosStd by Distance")
    plt.plot(_cov_distances, _cov_yaw, "+")
    plt.grid()
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