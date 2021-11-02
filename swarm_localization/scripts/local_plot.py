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
import copy

plt.rc('figure', figsize=(10,5))
#plt.rc('figure', figsize=(20,15))

def short_loop_id(id):
    return id //100000 + id%100000

def quat2eulers(w, x, y ,z):
    r = atan2(2 * (w * x + y * z),
                    1 - 2 * (x * x + y * y))
    p = asin(2 * (w * y - z * x))
    y = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return y, p, r

def RMSE(predictions, targets):
    if len(predictions) == 0:
        print("RMSE: no predictions")
        return 0
    err_sq = (predictions-targets)**2
    ret = np.sqrt(np.mean(err_sq))
    # print(predictions, targets, ret)
    ret = np.nan_to_num(ret, 0)

    return ret

def ATE_POS(predictions, targets):
    err = predictions-targets
    norm2 = err[:,0]*err[:,0]+err[:,1]*err[:,1]+err[:,2]*err[:,2]
    if np.isnan(norm2).any():
        print("ATE_POS has nan")

    return np.sqrt(np.mean(norm2))

def odometry_covariance_per_meter_with_rp(pos_vo, yaw_vo, pos_gt, yaw_gt, rp_length=1.0, gt_outlier_thres=0.1, show=False,step=1):
    i, j, c = 0, 0, 0
    sqr_err_pos_per_meter = np.zeros((3, 3))
    sqr_err_yaw_per_meter = 0
    ticks = []
    rp_errors = []
    dp_vos = []
    dp_gts = []

    if show:
        plt.figure()
        plt.title("rp_errors")
    while i < len(pos_vo) and j < len(pos_vo):
        len_ij = 0
        pos_last = pos_vo[i]
        j = i

        while j < len(pos_vo) - 1 and len_ij < rp_length:
            len_ij += np.linalg.norm(pos_vo[j] - pos_last)
            pos_last = pos_vo[j]
            j += 1
            # if i == 800:
                # print("len_ij", len_ij)

        #Now len ij is approximately rp_length, we compute error of ij
        pos_vo_i = pos_vo[i]
        pos_vo_j = pos_vo[j]
        yaw_vo_i = yaw_vo[i]
        yaw_vo_j = yaw_vo[j]

        dyaw_vo = wrap_pi(yaw_vo_j - yaw_vo_i)
        dpos_vo = yaw_rotate_vec(-yaw_vo_i, pos_vo_j - pos_vo_i)

        pos_gt_i = pos_gt[i]
        pos_gt_j = pos_gt[j]
        yaw_gt_i = yaw_gt[i]
        yaw_gt_j = yaw_gt[j]
        dyaw_gt = wrap_pi(yaw_gt_j - yaw_gt_i)
        dpos_gt = yaw_rotate_vec(-yaw_gt_i, pos_gt_j - pos_gt_i)
        dp_vos.append(dpos_vo)
        dp_gts.append(dpos_gt)
        ticks.append(i)
        
        err = np.transpose(np.array([(dpos_vo - dpos_gt)]))

        if len_ij > 0.01:
            sqr_err_pos = np.matmul(err, np.transpose(err))/len_ij
            sqr_err_yaw = ((dyaw_vo-dyaw_gt))*((dyaw_vo-dyaw_gt))/len_ij
            if np.linalg.norm(sqr_err_pos) < gt_outlier_thres*rp_length:
                sqr_err_pos_per_meter += sqr_err_pos
                sqr_err_yaw_per_meter += sqr_err_yaw
                c += 1
                rp_errors.append(np.linalg.norm(sqr_err_pos))
        i += step

    if show:
        dp_vos = np.array(dp_vos)
        dp_gts = np.array(dp_gts)
        # plt.subplot(311)
        plt.plot(ticks, dp_vos[:,0], label="VO X")
        plt.plot(ticks, dp_gts[:,0], label="GT X")
        plt.plot(ticks, dp_vos[:,0] - dp_gts[:,0], label="ERR X")
        plt.grid()
        plt.legend()
        # plt.subplot(312)
        # plt.plot(ticks, dp_vos[:,1], label="VO Y")
        # plt.plot(ticks, dp_gts[:,1], label="GT Y")
        # plt.grid()
        # plt.legend()
        # plt.subplot(313)
        # plt.plot(ticks, dp_vos[:,2], label="VO Z")
        # plt.plot(ticks, dp_gts[:,2], label="GT Z")
        # plt.grid()
        # plt.legend()
        # plt.grid()
        # print("RP Length", rp_length)
        # plt.plot(rp_errors)
        # plt.grid()
        plt.show()
    return sqr_err_pos_per_meter/c, sqr_err_yaw_per_meter/c

def odometry_covariance_per_meter(pos_vo, yaw_vo, pos_gt, yaw_gt, rp_lengths=[0.5, 1.0, 2.0], gt_outlier_thres=1.0, show=False,step=1):
    pos_covs = []
    sum_pos_cov = np.zeros((3, 3))
    sum_yaw_cov = 0
    for rp in rp_lengths:
        pos_cov, yaw_cov = odometry_covariance_per_meter_with_rp(pos_vo, yaw_vo, pos_gt, yaw_gt, rp_length=rp, gt_outlier_thres=gt_outlier_thres, show=show, step=step)
        sum_pos_cov += pos_cov
        sum_yaw_cov += yaw_cov
        pos_covs.append(np.linalg.norm(pos_cov))
    if show:
        plt.figure()
        plt.title("RP vs cov")
        plt.plot(rp_lengths, pos_covs)
        plt.grid()
        plt.show()
    return sum_pos_cov/len(rp_lengths), sum_yaw_cov/len(rp_lengths)

def yaw_rotate_vec(yaw, vec):
    Re = rotation_matrix(yaw, [0, 0, 1])[0:3, 0:3]
    return np.transpose(np.dot(Re, np.transpose(vec)))

def wrap_pi(data):
    return (data + np.pi) % (2 * np.pi) - np.pi

def read_pose_swarm_fused(bag, topic, _id, t0):
    pos = []
    ypr = []
    ts = []
    quat = []
    print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t.to_sec() > t0:
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
        if t.to_sec() > t0:
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
def read_distances_swarm_frame(bag, topic, t0):
    distances = {
    }
    ts = []
    print(f"Read distances from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t.to_sec() > t0:
            for node in msg.node_frames:
                _ida = node.id
                if not (_ida in distances):
                    distances[_ida] = {}
                for i in range(len(node.dismap_ids)):
                    _id = node.dismap_ids[i]
                    _dis = node.dismap_dists[i]
                    if not (_id in distances[_ida]):
                        distances[_ida][_id] = {
                            "t": [],
                            "dis": []
                        }
                    distances[_ida][_id]["t"].append(msg.header.stamp.to_sec() - t0)
                    distances[_ida][_id]["dis"].append(_dis)
    return distances

def read_distances_remote_nodes(bag, topic, t0, main_id):
    distances = {
    }
    ts = []
    print(f"Read distances from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t.to_sec() > t0:
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
        pos = msg.relative_pose.position
        q = msg.relative_pose.orientation
        y, p, r = quat2eulers(q.w, q.x, q.y, q.z)

        loop = {
            "ts_a": msg.ts_a.to_sec() - t0,
            "ts_b": msg.ts_b.to_sec() - t0,
            "id_a":msg.id_a,
            "id_b":msg.id_b,
            "dpos":np.array([pos.x, pos.y, pos.z]),
            "dyaw":y,
            "id":msg.id,
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
            "id": msg.id,
            "id_b":msg.remote_drone_id,
            "dpos":np.array([msg.dpos.x, msg.dpos.y, msg.dpos.z]),
            "pos_a" : np.array([msg.local_pose_self.position.x, msg.local_pose_self.position.y, msg.local_pose_self.position.z]),
            "extrinsic" : np.array([msg.camera_extrinsic.position.x, msg.camera_extrinsic.position.y, msg.camera_extrinsic.position.z]),
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
                "id": msg.id,
                "id_b":msg.remote_drone_id,
                "dpos":np.array([msg.dpos.x, msg.dpos.y, msg.dpos.z]),
                "pos_a" : np.array([msg.local_pose_self.position.x, msg.local_pose_self.position.y, msg.local_pose_self.position.z]),
                "pos_b" : np.array([msg.local_pose_remote.position.x, msg.local_pose_remote.position.y, msg.local_pose_remote.position.z]),
                "extrinsic" : np.array([msg.camera_extrinsic.position.x, msg.camera_extrinsic.position.y, msg.camera_extrinsic.position.z]),
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

def bag_read(bagname, nodes = [1, 2], is_pc=False, main_id=1, groundtruth = True, dt = 0):
    bag = rosbag.Bag(bagname)
    poses_gt = {}
    poses_fused = {}
    poses_vo = {}
    poses_path = {}
    t0 = 0
    plat = "pc"
    
    for topic, msg, t in bag.read_messages(topics=["/swarm_drones/swarm_frame"]):
        if msg.header.stamp.to_sec() < 1e9:
            continue

        if len(msg.node_frames) >= len(nodes):
            t0 = msg.header.stamp.to_sec() + dt
            # print("t0 is", t0, msg)
            break
    
    for i in nodes:
        if groundtruth:
            poses_gt[i], t0 = read_pose(bag, f"/SwarmNode{i}/pose", t0)
        if is_pc:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused_pc", i, t0)
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path_pc", t0)
        else:
            poses_fused[i] = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused", i, t0)
            #poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path", t0)

        poses_fused[i]["t"] = poses_fused[i]["t"]
        poses_vo[i] = read_pose_swarm_frame(bag, "/swarm_drones/swarm_frame", i, t0)

        if i not in poses_path or poses_path[i] is None:
            poses_path[i] = copy.copy(poses_fused[i])

    loops = read_loops(bag, t0, "/swarm_loop/loop_connection")
    # detections = read_detections(bag, t0, "/swarm_drones/node_detected")
    detections = read_detections_raw(bag, t0)
    # distances = read_distances_remote_nodes(bag, "/uwb_node/remote_nodes", t0, main_id)
    distances = read_distances_swarm_frame(bag, "/swarm_drones/swarm_frame", t0)
    bag.close()
    if groundtruth:
        offset_gt = - poses_gt[main_id]["pos"][0]
        yaw_offset_gt = -poses_gt[main_id]["ypr"][0, 0]
        print("Yaw Offset, ", yaw_offset_gt*57.3, "Fused Offset", offset_gt)
    else:
        offset_gt = np.array([0, 0, 0])
        yaw_offset_gt = 0

    fused_offset = - poses_fused[main_id]["pos"][0]
    fused_yaw_offset = -poses_fused[main_id]["ypr"][0, 0]
    #Pvicon = DP Ppose
    #DP = PviconPpose^-1
    #DP = (PPose^-1 Pvicon)^-1
    #PVicon = DYaw * Pos
    #YawVicon = DYaw + Yaw
    for i in nodes:
        poses_fused[i]["pos"] = yaw_rotate_vec(yaw_offset_gt, poses_fused[i]["pos"]) + fused_offset
        poses_fused[i]["ypr"] = poses_fused[i]["ypr"] + np.array([yaw_offset_gt, 0, 0])
        poses_fused[i]["pos_func"] = interp1d( poses_fused[i]["t"],  poses_fused[i]["pos"],axis=0,fill_value="extrapolate")
        poses_fused[i]["ypr_func"] = interp1d( poses_fused[i]["t"],  poses_fused[i]["ypr"],axis=0,fill_value="extrapolate")

        if i in poses_path:
            if len(poses_path[i]["pos"]) > 1:
                poses_path[i]["ypr"] = poses_path[i]["ypr"] + np.array([yaw_offset_gt, 0, 0])
                poses_path[i]["pos"] = yaw_rotate_vec(yaw_offset_gt, poses_path[i]["pos"]) + fused_offset
                poses_path[i]["pos_func"] = interp1d( poses_path[i]["t"],  poses_path[i]["pos"],axis=0,fill_value="extrapolate")
                poses_path[i]["ypr_func"] = interp1d( poses_path[i]["t"],  poses_path[i]["ypr"],axis=0,fill_value="extrapolate")

        if groundtruth:
            poses_gt[i]["pos"] = yaw_rotate_vec(fused_yaw_offset, poses_gt[i]["pos"]) + offset_gt
            poses_gt[i]["ypr"] = poses_gt[i]["ypr"] + np.array([fused_yaw_offset, 0, 0])
            poses_gt[i]["pos_func"] = interp1d( poses_gt[i]["t"],  poses_gt[i]["pos"],axis=0,fill_value="extrapolate")
            poses_gt[i]["ypr_func"] = interp1d( poses_gt[i]["t"],  poses_gt[i]["ypr"],axis=0,fill_value="extrapolate")

    for i in nodes:
        if groundtruth:
            vo_offset = poses_gt[i]["pos"][0] - poses_vo[i]["pos_raw"][0]
            yaw_offset = (poses_gt[i]["ypr"][0] - poses_vo[i]["ypr_raw"][0])[0]
            print(f"VIO Offset for {i}: {vo_offset}")
            print(poses_gt[i]["pos"][0], poses_vo[i]["pos_raw"][0])
        else:    
            vo_offset = np.array([0, 0, 0])
            yaw_offset = 0
        poses_vo[i]["pos"] = yaw_rotate_vec(yaw_offset, poses_vo[i]["pos_raw"]) + vo_offset
        poses_vo[i]["ypr"] = poses_vo[i]["ypr_raw"] + np.array([yaw_offset, 0, 0])
        poses_vo[i]["pos_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["pos"],axis=0,bounds_error=False,fill_value="extrapolate")
        poses_vo[i]["ypr_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["ypr"],axis=0,fill_value="extrapolate")
    if groundtruth:
        return poses_gt, poses_fused, poses_vo, poses_path, loops, detections, distances, t0
    else:
        return poses_fused, poses_vo, poses_path, loops, detections, distances, t0

def plot_fused(poses, poses_fused, poses_vo, poses_path, loops, detections, nodes, groundtruth = True, use_offline=False, output_path="/home/xuhao/output/", id_map = None):
    fig = plt.figure("Traj2", figsize=(6, 6))
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
        dpos = yaw_rotate_vec(yawa_, det["dpos"]/det["inv_dep"])
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
            arrow_length_ratio=0.1, color="black",linewidths=1.0, label="Map-based edges")

    step_det = 5
    if len(quivers_det) > 0:   
        ax.quiver(quivers_det[::step_det,0], quivers_det[::step_det,1], quivers_det[::step_det,2], quivers_det[::step_det,3],
            quivers_det[::step_det,4], quivers_det[::step_det,5], 
            arrow_length_ratio=0.1, color="gray",linewidths=1.0, label="Visual detection edges")

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

    fig = plt.figure("Fused Multi 2d", figsize=(6, 6))
    plt.gca().set_aspect('equal')
    
    step_det = 1
    qview_width = 0.9
    if len(quivers) > 0:   
        for i in range(0,len(quivers),step_det):
            xs = [quivers[i,0],quivers[i,0]+quivers[i,3]]
            ys = [quivers[i,1], quivers[i,1]+quivers[i,4]]
            if i == 0:
                plt.plot(xs, ys, color="black", label="Map-based edges", linewidth=qview_width)
            else:
                plt.plot(xs, ys, color="black", linewidth=qview_width)
    step_det = 1
    if len(quivers_det) > 0: 
        for i in range(0,len(quivers_det),step_det):
            xs = [quivers_det[i,0],quivers_det[i,0]+quivers_det[i,3]]
            ys = [quivers_det[i,1], quivers_det[i,1]+quivers_det[i,4]]
            if i == 0:
                plt.plot(xs, ys, color="gray", label="Visual detection edges", linewidth=qview_width)
            else:
                plt.plot(xs, ys, color="gray", linewidth=qview_width)
    for i in nodes:
        _id = id_map[i]

        if use_offline:
            plt.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], label=f"Estimation offline{_id}")
        plt.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Online {_id}")
    for i in nodes:
        _id = id_map[i]
        
        # plt.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1], label=f"VIO ${_id}$", alpha=0.7)
        final_vio = norm(poses_vo[i]["pos"][-1,:])
        if use_offline:
            final_path = norm(poses_path[i]["pos"][-1,:])
        else:
            final_path = norm(poses_fused[i]["pos"][-1,:])
            
        total_len = poses_length(poses_fused[i])
        if groundtruth:
            # plt.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1], label=f"Ground Truth {_id}")
            pass
        
        print(f"Final drift {i} VIO {final_vio:3.2f}m {final_vio/total_len*100:3.1f}% Fused {final_path:3.2f}m {final_path/total_len*100:3.1f}% total_len {total_len:.1f}m")
    
    plt.legend()
    plt.grid()
    plt.savefig(output_path+"fused2d.pdf")

    for k in range(len(nodes)):
        # fig.suptitle("Fused Vs GT 2D")
        i = nodes[k]
        _id = id_map[i]

        fig = plt.figure(f"Fused Vs GT 2D {i}", figsize=(6, 6))
        plt.gca().set_aspect('equal', adjustable="datalim", anchor="SE")

        if groundtruth:
            plt.plot(poses[i]["pos"][:,0], poses[i]["pos"][:,1], label=f"Ground Truth {_id}")
        plt.plot(poses_vo[i]["pos"][:,0], poses_vo[i]["pos"][:,1], label=f"VIO {_id}")
        # plt.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")
        if i in poses_path and use_offline:
            plt.plot(poses_path[i]["pos"][:,0], poses_path[i]["pos"][:,1], label=f"Estimation offline {_id}")
        plt.plot(poses_fused[i]["pos"][:,0], poses_fused[i]["pos"][:,1], label=f"Estimation {_id}")
        plt.grid()
        plt.ylabel('$Y$')
        plt.xlabel('$X$')
        plt.legend()

        plt.savefig(output_path+f"fusedvsgt2d_{i}.pdf")

    for i in nodes:
        _id = id_map[i]
        fig = plt.figure(f"Drone {i} fused Vs GT 1D")
        #fig.suptitle(f"Drone {i} fused Vs GT 1D")
        ax1, ax2, ax3, ax4 = fig.subplots(4, 1)

        t_ = poses_fused[i]["t"]
        if groundtruth:
            pos_gt =  poses[i]["pos_func"](poses_fused[i]["t"])
            yaw_gt =  poses[i]["ypr_func"](poses_fused[i]["t"])[:, 0]
        pos_fused = poses_fused[i]["pos"]
        _i = str(i) 
        if groundtruth:
            ax1.plot(t_, pos_gt[:,0], label=f"Ground Truth ${i}$")
        # ax1.plot(poses_path[i]["t"], poses_path[i]["pos"][:,0], '.', label=f"Fused Offline Traj{i}")
        ax1.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,0], label=f"Aligned VO Traj{_id}")
        ax1.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,0], label=f"Estimate {_id}")
        ax1.tick_params( axis='x', which='both', bottom=False, top=False, labelbottom=False) 
        ax1.set_ylabel("x")

        if groundtruth:
            ax2.plot(t_, pos_gt[:,1], label=f"Ground Truth ${i}$")
        #ax2.plot(poses_path[i]["t"], poses_path[i]["pos"][:,1], '.', label=f"Fused Offline Traj{i}")
        ax2.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,1], label=f"Aligned VO Traj{_id}")
        ax2.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,1], label=f"Estimate {_id}")
        ax2.tick_params( axis='x', which='both', bottom=False, top=False, labelbottom=False) 
        ax2.set_ylabel("y")

        if groundtruth:
            ax3.plot(t_, pos_gt[:,2], label=f"Ground Truth ${i}$")
        #ax3.plot(poses_path[i]["t"], poses_path[i]["pos"][:,2], '.', label=f"Fused Offline Traj{i}")
        ax3.plot(poses_vo[i]["t"], poses_vo[i]["pos"][:,2], label=f"Aligned VIO ${_id}$")
        ax3.plot(poses_fused[i]["t"], poses_fused[i]["pos"][:,2], label=f"Estimate {_id}")
        ax3.set_ylabel("z")
        ax3.set_xlabel("t")

        # ax1.legend()
        # ax2.legend()
        ax3.legend()
        ax1.grid()
        ax2.grid()
        ax3.grid()
        if groundtruth:
            ax4.plot(t_, yaw_gt, label=f"Ground Truth ${i}$")
        #ax3.plot(poses_path[i]["t"], poses_path[i]["pos"][:,2], '.', label=f"Fused Offline Traj{i}")
        ax4.plot(poses_vo[i]["t"], poses_vo[i]["ypr"][:,0], label=f"Aligned VIO ${_id}$")
        
        ax4.plot(poses_fused[i]["t"], poses_fused[i]["ypr"][:,0], label=f"Estimate {_id}")
        ax4.set_ylabel("Yaw")
        ax4.set_xlabel("t")
        ax4.legend()
        ax4.grid()
        plt.savefig(output_path+f"est_by_t{i}.png")

def plot_distance_err(poses, poses_fused, distances, main_id, nodes, calib = {}, is_show=False):
    for main_id in nodes:
        for i in nodes:
            if i == main_id:
                continue
            t_ = np.array(distances[i][main_id]["t"])
            pos_gt = poses[i]["pos_func"](t_)
            pos_fused = poses_fused[i]["pos_func"](t_)

            main_pos_gt = poses[main_id]["pos_func"](t_)
            main_pos_fused = poses_fused[main_id]["pos_func"](t_)

            #pos_vo = poses_vo[i]["pos"]
            #pos_path = poses_path[i]["pos"](t_)
            
            dis_raw = distances[i][main_id]["dis"]
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
        

def plot_relative_pose_err(poses, poses_fused, poses_vo, main_id, target_ids, dte=1000000, groundtruth = True, show=True):
    ts = poses_fused[main_id]["t"]
    ts = ts[ts<dte]
    posa_vo =  poses_vo[main_id]["pos_func"](ts)
    posa_fused = poses_fused[main_id]["pos_func"](ts)
    yawa_fused = poses_fused[main_id]["ypr_func"](ts)[:,0]
    yawa_vo = poses_vo[main_id]["ypr_func"](ts)[:,0]

    print("Relative Trajectory Statistics\nEST RMSE:\t\tPOS\t\tYAW\t|\tBIAS: POS\t\t\tYAW\t|VO\tRMSE:\tPOS\t\tYAW")
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
        
        if groundtruth:
            rmse_yaw = RMSE(wrap_pi(yawb_fused - yawa_fused - yawb_gt + yawa_gt), 0)
            rmse_x = RMSE(dp_gt[:,0] , dp_fused[:,0])
            rmse_y = RMSE(dp_gt[:,1] , dp_fused[:,1])
            rmse_z = RMSE(dp_gt[:,2] , dp_fused[:,2])


            rmse_x_no_bias = RMSE(dp_gt[:,0] - np.mean(dp_gt[:,0] - dp_fused[:,0]), dp_fused[:,0])
            rmse_y_no_bias = RMSE(dp_gt[:,1] - np.mean(dp_gt[:,1] - dp_fused[:,1]), dp_fused[:,1])
            rmse_z_no_bias = RMSE(dp_gt[:,2] - np.mean(dp_gt[:,2] - dp_fused[:,2]), dp_fused[:,2])
            #ERROR
            print(f"{main_id}->{target_id}\t{rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}\t{rmse_yaw*180/pi:3.2f}°", end="\t|")
            #BIAS
            print(f"{np.mean(dp_gt[:,0] - dp_fused[:,0]):3.3f},{np.mean(dp_gt[:,1] - dp_fused[:,1]):+3.3f},{np.mean(dp_gt[:,2] - dp_fused[:,2]):+3.3f}\t{np.mean(dyaw_gt - dyaw_fused)*180/3.14:+3.2f}°",end="\t")

            rmse_yaw = RMSE(wrap_pi(yawb_vo - yawa_vo - yawb_gt + yawa_gt), 0)
            rmse_x = RMSE(dp_gt[:,0] , dp_vo[:,0])
            rmse_y = RMSE(dp_gt[:,1] , dp_vo[:,1])
            rmse_z = RMSE(dp_gt[:,2] , dp_vo[:,2])

            print(f"|\t{rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}\t{rmse_yaw*180/pi:3.1f}°")
            # print(f"\t{np.mean(dp_gt[:,0] - dp_vo[:,0]):3.3f},{np.mean(dp_gt[:,1] - dp_vo[:,1]):3.3f},{np.mean(dp_gt[:,2] - dp_vo[:,2]):3.3f}")
            # print(f"RMSE NO BIAS {main_id}->{target_id} {rmse_x_no_bias:3.3f},{rmse_y_no_bias:3.3f},{rmse_z_no_bias:3.3f}")
    

        if show:
            fig = plt.figure(f"Relative Pose 2D {main_id}->{target_ids}")

            if groundtruth:
                plt.plot(dp_gt[:, 0], dp_gt[:, 1], label=f"Relative Pose GT {main_id}->{target_id}")
            plt.plot(dp_fused[:, 0], dp_fused[:, 1], label=f"Relative Pose EST {main_id}->{target_id}")
            plt.legend()
            if target_id == target_ids[0]:
                plt.grid()

            fig = plt.figure("Relative Pose Polar")
            fig.suptitle("Relative Pose Polar")
            ax1, ax2 = fig.subplots(2, 1)

            if groundtruth:
                ax1.plot(ts, np.arctan2(dp_gt[:, 0], dp_gt[:, 1]), label=f"Relative Pose Angular GT {main_id}->{target_id}")
            ax1.plot(ts, np.arctan2(dp_fused[:, 0], dp_fused[:, 1]), label=f"Relative Pose Angular Fused {main_id}->{target_id}")

            if groundtruth:
                ax2.plot(ts, norm(dp_gt, axis=1), label=f"Relative Pose Length GT {main_id}->{target_id}")
            ax2.plot(ts, norm(dp_fused, axis=1), label=f"Relative Pose Length Fused {main_id}->{target_id}")

            ax1.legend()
            ax1.grid()
            ax2.legend()
            ax2.grid()
            plt.tight_layout()

            fig = plt.figure("Relative Pose")
            fig.suptitle(f"Relative Pose {main_id}->{target_ids}")
            ax1, ax2, ax3, ax4 = fig.subplots(4, 1)

            if groundtruth:
                ax1.plot(ts, dp_gt[:,0], label="$X_{gt}^" + str(target_id) + "$")
                ax2.plot(ts, dp_gt[:,1], label="$Y_{gt}^" + str(target_id) + "$")
                ax3.plot(ts, dp_gt[:,2], label="$Z_{gt}^" + str(target_id) + "$")
                ax4.plot(ts, dyaw_gt, label="$Yaw_{gt}^" + str(target_id) + "$")

            ax1.plot(ts, dp_fused[:,0], label="$X_{fused}^" + str(target_id) + "$")
            ax2.plot(ts, dp_fused[:,1], label="$Y_{fused}^" + str(target_id) + "$")
            ax3.plot(ts, dp_fused[:,2], label="$Z_{fused}^" + str(target_id) + "$")
            ax4.plot(ts, dyaw_fused, label="$Yaw_{gt}^" + str(target_id) + "$")
            
            ax1.legend()
            ax2.legend()
            ax3.legend()
            ax4.legend()
            ax1.grid()
            ax2.grid()
            ax3.grid()
            ax4.grid()
            plt.tight_layout()
                
            fig = plt.figure("Fused Relative Error")
            fig.suptitle(f"Fused Relative Error {main_id}->{target_ids}")
            ax1, ax2, ax3 = fig.subplots(3, 1)

            ax1.plot(ts, dp_gt[:,0] - dp_fused[:,0], label="$E_{xfused}^" + str(target_id) + f"$ RMSE:{rmse_x:3.3f}")
            ax2.plot(ts, dp_gt[:,1] - dp_fused[:,1], label="$E_{yfused}^" + str(target_id) + f"$ RMSE:{rmse_y:3.3f}")
            ax3.plot(ts, dp_gt[:,2] - dp_fused[:,2], label="$E_{zfused}^" + str(target_id) + f"$ RMSE:{rmse_z:3.3f}")
            ax4.plot(ts, wrap_pi(dyaw_gt - dyaw_fused), label="$E_{yawfused}^" + str(target_id) + f"$ RMSE:{rmse_z:3.3f}")

            ax1.legend()
            ax2.legend()
            ax3.legend()
            ax1.grid()
            ax2.grid()
            ax3.grid()
            plt.tight_layout()

    if show:
        plt.show()


    


def plot_fused_err(poses, poses_fused, poses_vo, poses_path, nodes, main_id=1,dte=100000,show=True, rp_length=1.0):
    #Plot Fused Vs GT absolute error
    ate_vo_sum = 0
    rmse_vo_yaw_sum = 0

    ate_fused_sum = 0
    rmse_fused_yaw_sum = 0

    print("Absolute Trajectory Statistics\nEST:\tATE_P\tATE_Yaw\tATE\t\t\tCOV/m\t\tPOS\t\t\tYAW\t\t|\tVO:ATE_P\tYaw\t\t\tATE\t\tCOV/m\t\tPOS\t\tYAW\t")
    for i in nodes:
        t_ = poses_fused[i]["t"]
        mask = t_<dte
        t_ = t_[t_<dte]
        pos_gt =  poses[i]["pos_func"](t_)
        pos_fused = poses_fused[i]["pos"][mask]
        yaw_fused = poses_fused[i]["ypr"][mask][:,0]
        pos_vo = poses_vo[i]["pos"]
        if i in poses_path:
            pos_path = poses_path[i]["pos"]
        yaw_gt = poses[i]["ypr_func"](t_)[:,0]
        yaw_vo = poses_vo[i]["ypr"][:,0]
        _i = str(i) 

        fused_cov_per_meter, fused_yaw_cov_per_meter = odometry_covariance_per_meter(pos_fused, yaw_fused, pos_gt, yaw_gt)
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
        rmse_yaw_fused = RMSE(wrap_pi(yaw_gt-yaw_fused), 0)

        ate_fused_sum += ate_fused
        rmse_fused_yaw_sum += rmse_yaw_fused
        # if i in poses_path:
        #     pos_gt =  poses[i]["pos_func"](poses_path[i]["t"])
        #     rmse_path_x = RMSE(pos_path[:,0] , pos_gt[:,0])
        #     rmse_path_y = RMSE(pos_path[:,1] , pos_gt[:,1])
        #     rmse_path_z = RMSE(pos_path[:,2] , pos_gt[:,2])


        pos_gt_vo=  poses[i]["pos_func"](poses_vo[i]["t"])
        yaw_gt_vo =  poses[i]["ypr_func"](poses_vo[i]["t"])[:,0]
        rmse_vo_x = RMSE(pos_vo[:,0] , pos_gt_vo[:,0])
        rmse_vo_y = RMSE(pos_vo[:,1] , pos_gt_vo[:,1])
        rmse_vo_z = RMSE(pos_vo[:,2] , pos_gt_vo[:,2])

        vo_cov_per_meter, vo_yaw_cov_per_meter = odometry_covariance_per_meter(pos_vo, yaw_vo, pos_gt_vo, yaw_gt_vo, show=False,step=100)
        
        ate_vo = ATE_POS(pos_vo, pos_gt_vo)
        rmse_yaw_vo = RMSE(wrap_pi(yaw_vo-yaw_gt_vo), 0)

        ate_vo_sum += ate_vo
        rmse_vo_yaw_sum += rmse_yaw_vo

        if i == main_id:
            print(f"Ego{main_id}",end="\t")
        else:
            print(f"{i}by{main_id}",end="\t")
        print(f"{ate_fused:3.3f}\t{rmse_yaw_fused*180/pi:3.3f}°\t{rmse_x:3.3f},{rmse_y:3.3f},{rmse_z:3.3f}\t{fused_cov_x:.1e},{fused_cov_y:.1e},{fused_cov_z:.1e}\t{fused_yaw_cov_per_meter:.1e}rad\t|\t",end="")
        # if i in poses_path:
        #     print(f"RMSE Fused Offline Path {rmse_path_x:3.3f},{rmse_path_y:3.3f},{rmse_path_z:3.3f}")
        print(f"{ate_vo:3.3f}\t\t{rmse_yaw_vo*180/pi:3.3f}°\t{rmse_vo_x:3.3f},{rmse_vo_y:3.3f},{rmse_vo_z:3.3f}\t{vo_cov_per_meter[0][0]:.1e},{vo_cov_per_meter[1][1]:.1e},{vo_cov_per_meter[2][2]:.1e}\t{vo_yaw_cov_per_meter:.1e}rad")

        # print("VO COV POS\n", vo_cov_per_meter, 'yaw', vo_yaw_cov_per_meter)

        if show:
            fig = plt.figure(f"Fused Absolute Error {i}")
            fig.suptitle(f"Fused Absolute Error {i}")
            ax1, ax2, ax3, ax4 = fig.subplots(4, 1)
            label = f"$errx_{i}$ RMSE{i}:{rmse_x:3.3f}"
            ax1.plot(t_, pos_gt[:,0]  - pos_fused[:,0], label=label)

            label = f"$erry_{i}$ RMSE{i}:{rmse_y:3.3f}"
            ax2.plot(t_, pos_gt[:,1]  - pos_fused[:,1], label=label)

            label = f"$erry_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax3.plot(t_,  pos_gt[:,2]  - pos_fused[:,2], label=label)

            label = f"$yaw_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax4.plot(t_, wrap_pi(yaw_gt-yaw_fused), label=label)

            label = f"$VO errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
            ax1.plot(poses_vo[i]["t"], pos_gt_vo[:,0]  - pos_vo[:,0], label=label)

            label = f"$VO erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
            ax2.plot(poses_vo[i]["t"], pos_gt_vo[:,1]  - pos_vo[:,1], label=label)
            
            label = f"$VO errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
            ax3.plot(poses_vo[i]["t"], pos_gt_vo[:,2]  - pos_vo[:,2], label=label)

            label = f"$VO yaw_{i}$ RMSE{i}:{rmse_z:3.3f}"
            ax4.plot(poses_vo[i]["t"], wrap_pi(yaw_gt_vo-yaw_vo), label=label)

            # label = f"$Path errx_{i}$ RMSE{i}:{rmse_vo_x:3.3f}"
            # ax1.plot(poses_path[i]["t"], pos_gt[:,0]  - pos_path[:,0], label=label)

            # label = f"$Path erry_{i}$ RMSE{i}:{rmse_vo_y:3.3f}"
            # ax2.plot(poses_path[i]["t"], pos_gt[:,1]  - pos_path[:,1], label=label)
            
            # label = f"$Path errz_{i}$ RMSE{i}:{rmse_vo_z:3.3f}"
            # ax3.plot(poses_path[i]["t"], pos_gt[:,2]  - pos_path[:,2], label=label)
            ax3.legend()
            ax1.grid()
            ax2.grid()
            ax3.grid()
            for i in nodes:
                fig = plt.figure(f"Yaw {i}")
                plt.title(f"Yaw {i}")
                # ax1, ax2 = fig.subplots(1, 1)
                ax1 = fig.subplots(1, 1)
                t_ = poses_fused[i]["t"]
                yaw_gt =  poses[i]["ypr_func"](poses_fused[i]["t"])
                yaw_fused = poses_fused[i]["ypr"]
                yaw_vo = poses_vo[i]["ypr_func"](poses_fused[i]["t"])
                
                ax1.plot(t_,  yaw_gt[:,0]*57.3, label=f"$\psi gt{i}$")
                ax1.plot(t_,  yaw_fused[:,0]*57.3, label=f"$\psi fused{i}$")
                ax1.plot(t_,  yaw_vo[:,0]*57.3, label=f"$\psi fused{i}$")


                ax1.grid()
                ax1.legend()
                # ax2.plot(t_,  (yaw_fused[:,0] -  yaw_gt[:,0] + np.pi) % (2 * np.pi) - np.pi, label=f"$\psi_E{i}$")
                # yaw_gt =  poses[i]["ypr_func"](poses_vo[i]["t"])
                # ax2.plot(poses_vo[i]["t"],  (poses_vo[i]["ypr"][:,0] -  yaw_gt[:,0] + np.pi) % (2 * np.pi) - np.pi, label=f"$VO \psi_E{i}$")
                # ax2.set_ylim(-0.5, 0.5)
                # ax2.grid()
                # ax2.legend() 

        
        num = len(nodes)
    print(f"Avg\t{ate_fused_sum/num:3.3f}\t{rmse_yaw_fused*180/pi/num:3.3f}°\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t|\t{ate_vo_sum/num:3.3f}\t\t{rmse_yaw_vo/num*180/pi:3.3f}°")

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



def plot_loops_error(poses, loops, outlier_thres=1.0, inlier_file=""):
    good_loop_id = set()
    if inlier_file != "":
        with open(inlier_file, "r") as f:
            lines = f.readlines()
            for line in lines:
                good_loop_id.add(int(line))

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
    count_inter_loop = 0

    loops_error = {}
    loop_ids = []
    for loop in loops:
        if inlier_file!= "" and loop["id"] not in good_loop_id:
            continue
        # print(loop["id_a"], "->", loop["id_b"])
        if loop["id_a"] != loop["id_b"]:
            count_inter_loop += 1
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
        dyaw_errs.append(wrap_pi(yawb_gt-yawa_gt-loop["dyaw"]))
        pnp_inlier_nums.append(loop["pnp_inlier_num"])
        idas.append(loop["id_a"])
        idbs.append(loop["id_b"])
        loop_ids.append(loop["id"])

        if loop["id"] in loops_error:
            print("Duplicate loop", loop["id"])
        else:
            loops_error[loop["id"]] = {
                "pos": norm(dpos_gt - dpos_loop), 
                "yaw": wrap_pi(yawb_gt-yawa_gt-loop["dyaw"]), 
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
    fig = plt.figure("Loop Error")
    plt.subplot(411)
    plt.tight_layout()
    plt.plot(ts_a, dpos_errs_norm, 'x', label="Loop Error")
    plt.plot(ts_a, dpos_errs[:,0], '1', label="Loop Error X")
    plt.plot(ts_a, dpos_errs[:,1], '2', label="Loop Error Y")
    plt.plot(ts_a, dpos_errs[:,2], '3', label="Loop Error Z")
    plt.title(f"Error Pos Loop vs Vicon. ErrNorm max {np.max(dpos_errs_norm):.2f}m")
    plt.ylim(-np.min(dpos_errs_norm)*1.2, np.max(dpos_errs_norm)*1.2)
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
        if dpos_errs_norm[i]>0.2:
            plt.text(pnp_inlier_nums[i], dpos_errs_norm[i], f"{short_loop_id(loop_ids[i])}|{idas[i]}->{idbs[i]}", fontsize=12)
    # plt.figure()
    # plt.subplot(141)
    # plt.hist(dpos_errs_norm, 5, density=True, facecolor='g', alpha=0.75)

    plt.figure("DistanceVSErr")
    plt.title("DistancVSErr")
    plt.plot(dpos_loop_norms, dpos_errs_norm, "x", label="")
    plt.grid(which="both")
    for i in range(len(pnp_inlier_nums)):
        if dpos_errs_norm[i]>0.2:
            plt.text(dpos_loop_norms[i], dpos_errs_norm[i], f"{short_loop_id(loop_ids[i])}", fontsize=12)


    mask = np.array(dpos_errs_norm)<outlier_thres
    dpos_errs=dpos_errs[mask]
    dyaw_errs = dyaw_errs[mask]

    plt.figure("Loop Hist")
    plt.subplot(131)
    plt.hist(dpos_errs[:,0], 50, density=True, facecolor='g', alpha=0.75)

    mu, std = stats.norm.fit(dpos_errs[:,0])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "mu = %.2f,  std = %.2f cov = %.2f" % (mu, std, std*std)
    plt.title(title)

    plt.subplot(132)
    plt.hist(dpos_errs[:,1], 50, density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(dpos_errs[:,1])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "mu = %.2f,  std = %.2f cov = %.2f" % (mu, std, std*std)
    plt.title(title)

    plt.subplot(133)
    plt.hist(dpos_errs[:,2], 50, density=True, facecolor='g', alpha=0.75)
    mu, std = stats.norm.fit(dpos_errs[:,2])
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = stats.norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "mu = %.2f,  std = %.2f cov = %.2f" % (mu, std, std*std)
    plt.title(title)

    print(f"Pos cov {np.cov(dpos_errs[:,0]):.1e}, {np.cov(dpos_errs[:,1]):.1e}, {np.cov(dpos_errs[:,2]):.1e}")
    print(f"Yaw cov {np.cov(dyaw_errs):.1e}")

    # plt.figure()
    # plt.subplot(211)
    # plt.plot(dpos_gt_norms, dpos_errs_norm, 'o', label="GtDistance vs Error")
    # plt.grid(which="both")
    # plt.subplot(212)
    # plt.plot(dpos_loop_norms, dpos_errs_norm, 'o', label="LoopDistance vs Error")
    # plt.grid(which="both")
    # plt.legend()
    # plt.show()
    return loops_error

def debugging_pcm(pcm_folder, loops_error, pcm_threshold):
    pcm_errors = {}
    pcm_errors_sum = {}
    pcm_out_thres_count = {}
    good_loop_id = set()
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
    with open(pcm_folder+"/pcm_good.txt", "r") as f:
        lines = f.readlines()
        for line in lines:
            good_loop_id.add(int(line))
    print(f"PCM loops {len(pcm_errors)}")
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
            print(f"Loop {loop_id} not found")
            continue
        loop_id_array.append(loop_id)
        pcm_errors_sum_array.append(pcm_errors_sum[loop_id])
        pcm_out_thres_count_array.append(pcm_out_thres_count[loop_id])
        loop_error_T.append(loops_error[loop_id]["pos"])
        loop_error_yaw.append(loops_error[loop_id]["yaw"])
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
            plt.text(pcm_out_thres_count_array[i], loop_error_T[i], f"{(loop_id_array[i])},{loop_dt[i]:.1f}s", fontsize=12)
            print(f"{(loop_id_array[i])} {loop_dt[i]:.1f} error T {loop_error_T[i]}")
        if loop_id_array[i] not in good_loop_id:
            plt.text(pcm_out_thres_count_array[i], loop_error_T[i], "x", fontsize=12, color="red")

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