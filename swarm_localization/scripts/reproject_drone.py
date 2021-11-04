import matplotlib.pyplot as plt
import sys
import os
import cv2
import numpy as np
sys.path.append("../scripts")

from local_plot import *
#plt.tight_layout(pad=1.0, w_pad=0.2, h_pad=1.0)
plt.tight_layout(pad=0.4, w_pad=0.5, h_pad=1.0)
plt.rc("figure", figsize=(15,10))

def parse_params(text_file, t0):
    with open(text_file, "r") as f:
        lines = f.readlines()
        _bbox = lines[0].split(" ")
        t = float(_bbox[0])
        bx = float(_bbox[1])
        by = float(_bbox[2])
        w = float(_bbox[3])
        h = float(_bbox[4])
        _cam_ex = lines[1].split(" ")
        x, y, z = float(_cam_ex[0]), float(_cam_ex[1]), float(_cam_ex[2])
        qw, qx, qy, qz = float(_cam_ex[3]), float(_cam_ex[4]), float(_cam_ex[5]), float(_cam_ex[6])
        _cam_in = lines[2].split(" ")
        cx, cy, fx, fy = float(_cam_in[0]), float(_cam_in[1]), float(_cam_in[2]), float(_cam_in[3])
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        return {
            "t": t - t0,
            "bbox": [bx, by, w, h],
            "cam_T": np.array([x, y, z]),
            "cam_Q": np.array([qw, qx, qy, qz]),
            "Kcam": K
        }
    
def parse_images(folder, t0 = 0, step=1):
    img_files = [f for f in os.listdir(folder) if f.endswith('.jpg')]
    img_files.sort()
    img_files = img_files[::step]
    imgs = [cv2.imread(folder + "/" + img) for img in img_files]
    params = [parse_params(folder + "/" + img.split(".")[0] + ".txt", t0) for img in img_files]
    return imgs, params

def quaternion_rotate(q, v):
    mat = quaternion_matrix(q)[0:3,0:3]
    v = np.array([v])
    v = v.transpose()
    v = mat @ v
    v = v.transpose()[0]
    return v

def quaternion_rotate_pts(q, pts):
    mat = quaternion_matrix(q)[0:3,0:3]
    pts = mat @ (pts.transpose())
    return pts.transpose()

def _reproject_landmarks(pt3ds, T1, Q1, T2, Q2, Tcam, Qcam, Kcam, P_vicon_in_GC1, P_vicon_in_GC2, correct_extrinsic_z):
    #Cvt pt3d in vicon body frame
    pt3ds = pt3ds - P_vicon_in_GC2
    #Cvt pt3d in vicon frame
    pt3ds_v = quaternion_rotate_pts(Q2, pt3ds) + T2
    #Convert camera position to  body vicon frame
    Tcam_vb = np.array([0, 0, Tcam[2] + correct_extrinsic_z]) - P_vicon_in_GC1
    #Convert camera position to  body frame
    Tcam_v =quaternion_rotate(Q1, Tcam_vb) + T1
    Qcam_v = quaternion_multiply(Q1, Qcam)
    
    pt3ds_cam = quaternion_rotate_pts(quaternion_inverse(Qcam_v) ,pt3ds_v - Tcam_v)
    if np.min(pt3ds_cam[:,2]) < 0:
        return False, []
    pts_uv = (Kcam@pt3ds_cam.transpose()).transpose()
    pts2d = []
    for i in range(len(pts_uv)):
        ptuv = pts_uv[i]/pts_uv[i,2]
        pts2d.append([ptuv[0], ptuv[1]])
    return True, np.array(pts2d)
    
def reproject_landmarks(landmarks, img, param, poses, main_id, target_id, P_vicon_in_GC1, P_vicon_in_GC2, Qcam_calib, correct_extrinsic_z):
    t = param["t"]
    T1 = poses[main_id]["pos_func"](t)
    ypr1 = poses[main_id]["ypr_func"](t)
    Q1 = quaternion_from_euler(ypr1[2], ypr1[1],ypr1[0])
    T2 = poses[target_id]["pos_func"](t)
    ypr2 = poses[target_id]["ypr_func"](t)
    Q2 = quaternion_from_euler(ypr2[2], ypr2[1],ypr2[0])
    Qcam = quaternion_multiply(Qcam_calib, param["cam_Q"])
    succ, pts2d = _reproject_landmarks(landmarks, T1, Q1, T2, Q2, param["cam_T"], Qcam, param["Kcam"], P_vicon_in_GC1, P_vicon_in_GC2, correct_extrinsic_z )
    if  not succ or np.max(pts2d[:,0]) > img.shape[1] or np.min(pts2d[:,0]) < 0 or np.max(pts2d[:,1]) > img.shape[0]  or np.min(pts2d[:,1])  <0:
        return False, pts2d, None, None
    if param["bbox"][0] > 0:
        #crop use BBOX from detector
        w = param["bbox"][2]
        h = param["bbox"][3]
        x = int(param["bbox"][0]-w*0.1)
        y = max(int(param["bbox"][1]  - h/3), 0)
        w = int(w*1.2)
        h =  int(h*1.66)
    else:
        h =int( (np.max(pts2d[:,1]) - np.min(pts2d[:,1])) *1.2)
        y = max(int(np.min(pts2d[:,1]) - 0.1* (np.max(pts2d[:,1]) -  np.min(pts2d[:,1]))), 0)

        w =int( (np.max(pts2d[:,0]) -  np.min(pts2d[:,0]))*1.5)
        x = max(int(np.min(pts2d[:,0]) - 0.25* (np.max(pts2d[:,0]) -  np.min(pts2d[:,0]))), 0)
    img = img[y:y+h,x:x+w]   
    ret = []
    
    _show = img.copy()
    for i in range(len(pts2d)):
        pt = pts2d[i]
        _x = int(pt[0] - x)
        _y = int(pt[1] - y)
        ret.append([_x, _y])
        _show =  cv2.circle(_show, (_x, _y), 1, (0, 255, 0), -1)
            
    return succ, ret, img, _show

def process_data(imgs, params, poses, output_folder, main_id, target_id, Pvicon_gc, Qcam_calib,  correct_extrinsic_z, \
        prefix="img_", _show=False, interactive=False, vc=10):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    if _show:
        img = None
        c = 0
        plt.rc("figure", figsize=(50,10))
        f, axarr = plt.subplots(1, vc) 
    count = 0
    for index in range(0, len(imgs), 1):
        succ, pts, img, show = reproject_landmarks(landmarks_GC/1000.0, imgs[index].copy(), params[index], poses, main_id, target_id, \
            Pvicon_gc[main_id]/1000.0, Pvicon_gc[target_id]/1000.0, Qcam_calib, correct_extrinsic_z/1000.0)
        if succ:
            if interactive:
                if img.shape[0] == 0 or  img.shape[1] == 0:
                    continue
                show = cv2.resize(show, [0, 0], fx=4, fy=4)
                cv2.imshow("A: Accept, S: Skip", show)
                key = cv2.waitKey(-1)
                if key == ord("a"):
                    #print("Accept")
                    pass
                elif key == ord("q"):
                    #print("Terminate.")
                    cv2.destroyAllWindows()
                    return
                else:
                    #print("Escape")
                    continue
                cv2.imwrite(f"{output_folder}/{prefix}_{count:06d}.jpg", img)
                with open(f"{output_folder}/{prefix}_{count:06d}.txt", "w") as f:
                    for i in range(len(pts)):
                        f.write(f"{pts[i][0]} {pts[i][1]}\n")

            elif _show:
                if not succ or img.shape[0] == 0 or  img.shape[1] == 0 :
                    continue
                show = cv2.cvtColor(show, cv2.COLOR_BGR2RGB)
                axarr[c].imshow(show)
                c += 1
                if c >= vc:
                    plt.show()
                    c = 0
                    f, axarr = plt.subplots(1, vc) 
            count += 1
    print(f"Total {count} images")
    cv2.destroyAllWindows()

    
landmarks_GC = np.array([
    [102.15,121.79,52.2],
    [-102.15,121.79,52.2],
    [102.15,-121.79,52.2],
    [-102.15,-121.79,52.2],
    [0, 0, 115],
    [0, 0, -55],
    [98.92,117.94,-71],
    [-98.92,117.94,-71],
    [98.92,-117.94,-71],
    [-98.92,-117.94,-71],
    [-60,0, 177]
])
