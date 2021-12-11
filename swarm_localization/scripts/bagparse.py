#!/usr/bin/env python3
import rosbag
import numpy as np
from math import *
from scipy.interpolate import interp1d
import copy
from utils import *
import time

def read_pose_swarm_fused(bag, topic, t0, te):
    pos = {}
    ypr = {}
    ts = {}
    quat = {}
    ret_poses = {}
    # print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if te > t.to_sec() > t0:
            for i in range(len(msg.ids)):
                _id = msg.ids[i]
                if _id not in pos:
                    pos[_id] = []
                    ypr[_id] = []
                    ts[_id] = []
                    quat[_id] = []
                _ts = ts[_id]
                _pos = pos[_id]
                _ypr = ypr[_id]
                _quat = quat[_id]

                q = msg.local_drone_rotation[i]
                y, p, r = quat2eulers(q.w, q.x, q.y, q.z)
                _ts.append(msg.header.stamp.to_sec() - t0)
                _pos.append([msg.local_drone_position[i].x, msg.local_drone_position[i].y, msg.local_drone_position[i].z])
                _ypr.append([y, p, r])
                _quat.append([q.w, q.x, q.y, q.z])

    for _id in pos:
        _ts = ts[_id]
        _pos = pos[_id]
        _ypr = ypr[_id]
        _quat = quat[_id]
        ret = {
            "t": np.array(_ts),
            "pos": np.array(_pos),
            "quat": np.array(_quat),
            "ypr": np.array(_ypr),
        }
        ret_poses[_id] = ret
    return ret_poses

def read_pose_swarm_frame(bag, topic, t0, te):
    pos = {}
    ypr = {}
    ts = {}
    quat = {}
    ret_poses = {}
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t.to_sec() > t0:
            if te < msg.header.stamp.to_sec() or   msg.header.stamp.to_sec() > te:
                    continue
            for node in msg.node_frames:
                _id = node.id
                if _id not in pos and node.vo_available:
                    pos[_id] = []
                    ypr[_id] = []
                    ts[_id] = []
                    quat[_id] = []

                if node.vo_available:
                    _ts = ts[_id]
                    _pos = pos[_id]
                    _ypr = ypr[_id]
                    _quat = quat[_id]

                    _ts.append(node.header.stamp.to_sec() - t0)
                    y, p, r = quat2eulers(node.quat.w, node.quat.x, node.quat.y, node.quat.z)
                    _pos.append([node.position.x, node.position.y, node.position.z])
                    _ypr.append([y, p, r])
                    _quat.append([node.quat.w, node.quat.x, node.quat.y, node.quat.z])
    for _id in pos:
        _ts = ts[_id]
        _pos = pos[_id]
        _ypr = ypr[_id]
        _quat = quat[_id]
        ret = {
            "t": np.array(_ts),
            "pos": np.array(_pos),
            "quat": np.array(_quat),
            "ypr": np.array(_ypr),
        }
        ret_poses[_id] = ret
    return ret_poses

# def read_distance_swarm_frame(bag, topic, _id, to)
def read_distances_swarm_frame(bag, topic, t0):
    distances = {
    }
    ts = []
    # print(f"Read distances from topic {topic}")
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

def read_pose(bag, topic, t0, te, P_vicon_in_imu=None):
    pos = []
    ypr = []
    ts = []
    quat = []
    # print(f"Read poses from topic {topic}")
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if t0 == 0:
            t0 = msg.header.stamp.to_sec()
        else:
            if msg.header.stamp.to_sec() < t0 or msg.header.stamp.to_sec()>te:
                continue
        p = msg.pose.position
        q = msg.pose.orientation
        pos.append([p.x, p.y, p.z])
        y, p, r = quat2eulers(q.w, q.x, q.y, q.z)
        quat.append([q.w, q.x, q.y, q.z])
        ypr.append([y, p, r])
        ts.append(msg.header.stamp.to_sec() - t0)
    
    # T_viconc_now = quat*viconc_in_imu + T_imu_vicon
            # T_imu_vicon = T_viconc_now - quat*viconc_in_imu

    pos = np.array(pos)
    ypr = np.array(ypr)
    if P_vicon_in_imu is not None: #CVT to IMU
        for j in range(len(pos)):
            pos[j] = pos[j] - yaw_rotate_vec(ypr[j,0], P_vicon_in_imu) #Roll pitch is ignored since they are small.

    ret = {
        "t": np.array(ts),
        "pos": pos,
        "pos_func": interp1d(ts, pos,axis=0,bounds_error=False,fill_value="extrapolate"),
        "ypr": ypr,
        "ypr_func": interp1d(ts, ypr,axis=0,bounds_error=False,fill_value="extrapolate"),
        "quat": np.array(quat)
    }
    
    return ret, t0


def parse_path(path, t0, te):
    pos = []
    ypr = []
    ts = []
    quat = []
    
    for msg in path.poses:
        if te > msg.header.stamp.to_sec() > t0:
            p = msg.pose.position
            q = msg.pose.orientation
            pos.append([p.x, p.y, p.z])
            y, p, r = quat2eulers(q.w, q.x, q.y, q.z)
            ypr.append([y, p, r])
            ts.append(msg.header.stamp.to_sec() - t0)
            quat.append([q.w, q.x, q.y, q.z])  
    ret = {
        "t": np.array(ts),
        "pos": np.array(pos),
        "ypr": np.array(ypr),
        "quat": np.array(quat)
    }

    return ret

def read_path(bag, topic, t0, te):
    path = None
    for topic, msg, t in bag.read_messages(topics=[topic]):
        path = msg
    if path is not None:
        return parse_path(path, t0, te)
    return None

def poses_length(poses):
    dp = np.diff(poses["pos"], axis=0)
    length = np.sum(np.linalg.norm(dp,axis=1))
    return length

def parse_loopedge(msg, t0):
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
    return loop
    
def read_loops(bag, t0, topic="/swarm_loop/loop_connection"):
    loops = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        loops.append(parse_loopedge(msg, t0))
    return loops 

def read_goodloops(bag, t0, topic="/swarm_drones/goodloops"):
    all_loops = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        loops = []
        for _msg in msg.loops:
            loops.append(parse_loopedge(_msg, t0))
        all_loops.append(loops)
    return all_loops 


def read_detections_6d(bag, t0, topic="/swarm_drones/node_detected_6d"):
    dets = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        pos = msg.relative_pose.pose.position
        q = msg.relative_pose.pose.orientation
        y, p, r = quat2eulers(q.w, q.x, q.y, q.z)
        det = {
            "ts": msg.header.stamp.to_sec() - t0,
            "ts_a": msg.header.stamp.to_sec() - t0,
            "ts_b": msg.header.stamp.to_sec() - t0,
            "id_a":msg.self_drone_id,
            "id": msg.id,
            "id_b":msg.remote_drone_id,
            "dpos":np.array([pos.x, pos.y, pos.z]),
            "dyaw":y,
            "pnp_inlier_num": 0
        }
        dets.append(det)
    return dets

def read_loop_inliers(bag, topic="/swarm_drones/loop_inliers"):
    inliers_set = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        _set = set(msg.data)
        inliers_set.append(_set)
    return inliers_set

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

def bag_read(bagname, nodes = [1, 2], is_pc=False, main_id=1, groundtruth = True, t_s = 0, t_e=10000, P_vicon_in_imu={}, verbose=False):
    bag = rosbag.Bag(bagname)
    poses_gt = {}
    poses_fused = {}
    poses_vo = {}
    poses_path = {}
    t0 = 0
    for topic, msg, t in bag.read_messages(topics=["/swarm_drones/swarm_frame"]):
        if msg.header.stamp.to_sec() < 1e9:
            continue

        if len(msg.node_frames) >= len(nodes):
            t0 = msg.header.stamp.to_sec() + t_s
            te = msg.header.stamp.to_sec() + t_s + t_e 
            # print("t0 is", t0, msg)
            break

    poses_vo = read_pose_swarm_frame(bag, "/swarm_drones/swarm_frame", t0, te)
    # print("Finish parse vo")

    if is_pc:
        poses_fused = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused_pc", t0, te)
    else:
        poses_fused = read_pose_swarm_fused(bag, "/swarm_drones/swarm_drone_fused", t0, te)
    # print("Finish parse fused")
    sum_len = 0
    for i in poses_fused:
        if groundtruth:
            if i in P_vicon_in_imu:
                poses_gt[i], t0 = read_pose(bag, f"/SwarmNode{i}/pose", t0, te, P_vicon_in_imu[i])
            else:
                poses_gt[i], t0 = read_pose(bag, f"/SwarmNode{i}/pose", t0, te)
            sum_len += poses_length(poses_gt[i])

        if is_pc:
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path_pc", t0, te)
        else:    
            poses_path[i] = read_path(bag, f"/swarm_drones/est_drone_{i}_path", t0, te)

        if i not in poses_path or poses_path[i] is None:
            poses_path[i] = copy.copy(poses_fused[i])

    if groundtruth:
        print(f"Init. at {poses_fused[main_id]['t'][0]}. Avg. len. {sum_len/len(poses_gt):.1f}m")
    else:
        print(f"Init. at {poses_fused[main_id]['t'][0]}m")

    if groundtruth:
        offset_gt = - poses_gt[main_id]["pos"][0]
        yaw_offset_gt = -poses_gt[main_id]["ypr"][0, 0]
    else:
        offset_gt = np.array([0, 0, 0])
        yaw_offset_gt = 0

    fused_offset = np.array([0, 0, 0])
    fused_yaw_offset = 0

    # Pvicon = DP Ppose
    # DP = PviconPpose^-1
    # DP = (PPose^-1 Pvicon)^-1
    # PVicon = DYaw * Pos
    # YawVicon = DYaw + Yaw
    for i in nodes:
        poses_fused[i]["pos"] = yaw_rotate_vec(fused_yaw_offset, poses_fused[i]["pos"]) + yaw_rotate_vec(fused_yaw_offset, fused_offset)
        poses_fused[i]["ypr"] = poses_fused[i]["ypr"] + np.array([fused_yaw_offset, 0, 0])
        poses_fused[i]["ypr"][:, 0] = np.unwrap(poses_fused[i]["ypr"][:, 0])
        poses_fused[i]["pos_func"] = interp1d( poses_fused[i]["t"],  poses_fused[i]["pos"],axis=0,fill_value="extrapolate")
        poses_fused[i]["ypr_func"] = interp1d( poses_fused[i]["t"],  poses_fused[i]["ypr"],axis=0,fill_value="extrapolate")

        if i in poses_path:
            if len(poses_path[i]["pos"]) > 1:
                poses_path[i]["ypr"] = poses_path[i]["ypr"] + np.array([fused_yaw_offset, 0, 0])
                poses_path[i]["ypr"][:, 0] = np.unwrap(poses_path[i]["ypr"][:, 0])
                poses_path[i]["pos"] = yaw_rotate_vec(fused_yaw_offset, poses_path[i]["pos"])  + yaw_rotate_vec(fused_yaw_offset, fused_offset)
                poses_path[i]["pos_func"] = interp1d( poses_path[i]["t"],  poses_path[i]["pos"],axis=0,fill_value="extrapolate")
                poses_path[i]["ypr_func"] = interp1d( poses_path[i]["t"],  poses_path[i]["ypr"],axis=0,fill_value="extrapolate")

        if groundtruth:
            #Align by initial
            poses_gt[i]["pos"] = yaw_rotate_vec(yaw_offset_gt, poses_gt[i]["pos"]) + yaw_rotate_vec(yaw_offset_gt, offset_gt)
            poses_gt[i]["ypr"] = poses_gt[i]["ypr"] + np.array([yaw_offset_gt, 0, 0])
            poses_gt[i]["ypr"][:, 0] = np.unwrap(poses_gt[i]["ypr"][:, 0])
            poses_gt[i]["pos_func"] = interp1d( poses_gt[i]["t"],  poses_gt[i]["pos"],axis=0,fill_value="extrapolate")
            poses_gt[i]["ypr_func"] = interp1d( poses_gt[i]["t"],  poses_gt[i]["ypr"],axis=0,fill_value="extrapolate")

    for i in nodes:
        if groundtruth:
            vo_offset = poses_gt[i]["pos"][0] - poses_vo[i]["pos"][0]
            yaw_offset = (poses_gt[i]["ypr"][0] - poses_vo[i]["ypr"][0])[0]
        else:    
            vo_offset = np.array([0, 0, 0])
            yaw_offset = 0
        poses_vo[i]["pos"] = yaw_rotate_vec(yaw_offset, poses_vo[i]["pos"]) + yaw_rotate_vec(yaw_offset, vo_offset)
        poses_vo[i]["ypr"] = poses_vo[i]["ypr"] + np.array([yaw_offset, 0, 0])
        poses_vo[i]["ypr"][:, 0] = np.unwrap(poses_vo[i]["ypr"][:, 0])
        poses_vo[i]["pos_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["pos"],axis=0,bounds_error=False,fill_value="extrapolate")
        poses_vo[i]["ypr_func"] = interp1d( poses_vo[i]["t"],  poses_vo[i]["ypr"],axis=0,fill_value="extrapolate")
    if groundtruth:
        return poses_gt, poses_fused, poses_vo, poses_path, t0, bag
    else:
        return poses_fused, poses_vo, poses_path, t0, bag