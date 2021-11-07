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

DISP_SCALE = 4
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

def rotate_pts_2d(a, pts):
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    _pts = pts.transpose()
    return (R@_pts).transpose()

def _reproject_landmarks(pt3ds, T1, Q1, T2, Q2, Tcam, Qcam, Kcam, P_vicon_in_GC1, P_vicon_in_GC2, correct_extrinsic_z):
    #Cvt pt3d in vicon body frame
    pt3ds = pt3ds - P_vicon_in_GC2
    #Cvt pt3d in vicon frame
    pt3ds_v = quaternion_rotate_pts(Q2, pt3ds) + T2
    #Convert camera position to  body vicon frame
    Tcam_vb = Tcam - P_vicon_in_GC1
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

fx = 0
fy = 0
cam_roll = 0
cam_pitch = 0
cam_yaw = 0
def reproject_landmarks(landmarks, img, param, poses, main_id, target_id, P_vicon_in_GC1, P_vicon_in_GC2, Qcam_calib, correct_extrinsic_z, min_width=64, show_crop=True):
    t = param["t"]
    T1 = poses[main_id]["pos_func"](t)
    ypr1 = poses[main_id]["ypr_func"](t)
    Q1 = quaternion_from_euler(ypr1[2], ypr1[1],ypr1[0])
    T2 = poses[target_id]["pos_func"](t)
    ypr2 = poses[target_id]["ypr_func"](t)
    Q2 = quaternion_from_euler(ypr2[2], ypr2[1],ypr2[0])
    Qcam = quaternion_multiply(Qcam_calib, param["cam_Q"])
    Tcam_vb = np.array([0, 0, param["cam_T"][2] + correct_extrinsic_z])
    succ, pts2d = _reproject_landmarks(landmarks, T1, Q1, T2, Q2, Tcam_vb, Qcam, param["Kcam"], P_vicon_in_GC1, P_vicon_in_GC2, correct_extrinsic_z )
    if  not succ or np.max(pts2d[:,0]) > img.shape[1] or np.min(pts2d[:,0]) < 0 or np.max(pts2d[:,1]) > img.shape[0]  or np.min(pts2d[:,1])  <0:
        return False, pts2d, None, None
    if param["bbox"][0] > 0:
        #crop use BBOX from detector
        w = param["bbox"][2]
        h = param["bbox"][3]
        x = int(param["bbox"][0]-w*0.2)
        y = max(int(param["bbox"][1]  - h/4), 0)
        w = int(w*1.4)
        h =  int(h*1.5)
        if w < min_width:
            return False, pts2d, None, None
    else:
        h =int( (np.max(pts2d[:,1]) - np.min(pts2d[:,1])) *1.4)
        y = max(int(np.min(pts2d[:,1]) - 0.2* (np.max(pts2d[:,1]) -  np.min(pts2d[:,1]))), 0)

        w = int( (np.max(pts2d[:,0]) -  np.min(pts2d[:,0]))*1.8)
        x = max(int(np.min(pts2d[:,0]) - 0.4* (np.max(pts2d[:,0]) -  np.min(pts2d[:,0]))), 0)
        
        if w < min_width:
            return False, pts2d, None, None

    img = img[y:y+h,x:x+w]   
    
    mean = np.mean(pts2d, axis=0)
    pts2d = rotate_pts_2d(mouse_rotate, pts2d-mean)
    pts2d[:,0] = pts2d[:,0]*mouse_scale + fx + mean[0]
    pts2d[:,1] = pts2d[:,1]*mouse_scale + fy + mean[1]

    if show_crop:
        ret = []
        
        _show = img.copy()
        for i in range(len(pts2d)):
            pt = pts2d[i]
            _x = int(pt[0] - x)
            _y = int(pt[1] - y)
            ret.append([_x, _y])
            _show =  cv2.circle(_show, (_x, _y), 1, (0, 255, 0), -1)
    else:
        ret = []
        _show = img.copy()
        for i in range(len(pts2d)):
            pt = pts2d[i]
            _x = int(pt[0])
            _y = int(pt[1])
            ret.append([_x - x, _y - y])
            _show =  cv2.circle(_show, (_x, _y), 1, (0, 255, 0), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            dir = quaternion_rotate(Qcam, [0, 0, 1])
            cv2.putText(_show,  f'Drone1 Pose {T1[0]:.2f} {T1[1]:.2f} {T1[2]:.2f} YRP {ypr1[0]*57.3:.1f} {ypr1[1]*57.3:.1f} {ypr1[2]*57.3:.1f} deg',
                (10, 20), font, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(_show,  f'Drone2 Pose {T2[0]:.2f} {T2[1]:.2f} {T2[2]:.2f} YRP {ypr2[0]*57.3:.1f} {ypr2[1]*57.3:.1f} {ypr2[2]*57.3:.1f} deg',
                (10, 50), font, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(_show,  f'Cam Pose {Tcam_vb[0]:.2f} {Tcam_vb[1]:.2f} {Tcam_vb[2]:.2f} Dir {dir[0]:.2f} {dir[1]:.2f} {dir[2]:.2f}',
                (10, 80), font, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
    return succ, ret, img, _show

def process_data(imgs, params, poses, output_folder, main_id, target_id, landmarks_GC, Pvicon_gc, Qcam_calib,  correct_extrinsic_z, \
        prefix="img_", mode=0, vc=10, step=3):
    
    if mode == 0:
        interactive = False
        no_filter = False
        _show=True

    if mode == 1:
        interactive = True
        no_filter = False
        _show=False

    if mode == 2:
        interactive = False
        no_filter = True
        _show=False

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    if _show:
        img = None
        c = 0
        plt.rc("figure", figsize=(50,10))
        f, axarr = plt.subplots(1, vc) 
    count = 0
    print(len(imgs))
    for index in range(0, len(imgs), step):
        decide = False
        global mouse_scale
        while not decide:
            succ, pts, img, show = reproject_landmarks(landmarks_GC/1000.0, imgs[index].copy(), params[index], poses, main_id, target_id, \
                Pvicon_gc[main_id]/1000.0, Pvicon_gc[target_id]/1000.0, Qcam_calib, correct_extrinsic_z/1000.0)
            if  not succ or img.shape[0] == 0 or  img.shape[1] == 0 :
                    break
            accept = no_filter
            if interactive or no_filter: 
                show = cv2.resize(show, [0, 0], fx=DISP_SCALE, fy=DISP_SCALE)
                if not no_filter:
                    cv2.putText(show,  f'{count}/{index}/{len(imgs)}',
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.imshow("A: Accept, S: Skip", show)
                    cv2.setMouseCallback("A: Accept, S: Skip", mouse_handler)
                    key = cv2.waitKey(10)
                    if key == ord("a"):
                        decide = True
                        accept = True
                    elif key == ord("q"):
                        cv2.destroyAllWindows()
                        return
                    elif key == ord("s"):
                        decide = True
                        break
                else:
                    cv2.imshow("Landmarks", show)
                    key = cv2.waitKey(1)
                    if key == ord("q"):
                        print("Terminate.")
                        cv2.destroyAllWindows()
                        return
                if accept:
                    cv2.imwrite(f"{output_folder}/{prefix}_{index:06d}_{count}.jpg", img)
                    with open(f"{output_folder}/{prefix}_{index:06d}_{count}.txt", "w") as f:
                        for i in range(len(pts)):
                            f.write(f"{pts[i][0]} {pts[i][1]}\n")
                    count += 1

            elif _show:
                decide = True
                if not succ or img.shape[0] == 0 or  img.shape[1] == 0 :
                    continue
                show = cv2.cvtColor(show, cv2.COLOR_BGR2RGB)
                axarr[c].imshow(show)
                c += 1
                if c >= vc:
                    plt.show()
                    c = 0
                    f, axarr = plt.subplots(1, vc) 
    print(f"Total {count} images")
    cv2.destroyAllWindows()

mouse_down_x = 0
mouse_down_y = 0
mouse_mode = -1
mouse_scale = 1.0
mouse_rotate = 0
def mouse_handler(event, x, y, flags, data):
    global mouse_down_x, mouse_down_y
    global fx, fy, mouse_mode, mouse_scale, mouse_rotate
    if event == cv2.EVENT_LBUTTONDOWN:
        if flags == cv2.EVENT_FLAG_CTRLKEY + cv2.EVENT_FLAG_LBUTTON: #Scale pts
            mouse_mode = 1
        elif flags == cv2.EVENT_FLAG_SHIFTKEY + cv2.EVENT_FLAG_LBUTTON: #Scale pts
            mouse_mode = 2
        else:
            mouse_mode = 0
        mouse_down_x = x 
        mouse_down_y = y

    if event == cv2.EVENT_MOUSEMOVE:
        if mouse_mode == 0:
            fx += (x-mouse_down_x)/DISP_SCALE
            fy += (y-mouse_down_y)/DISP_SCALE
            mouse_down_x = x 
            mouse_down_y = y
        if mouse_mode == 1:
            mouse_scale *= (1-(y-mouse_down_y)/1000.0)
            mouse_down_x = x 
            mouse_down_y = y
        elif mouse_mode == 2:
            mouse_rotate -= (x-mouse_down_x)/300.0
            mouse_down_x = x 
            mouse_down_y = y
            
    if event == cv2.EVENT_LBUTTONUP:
        mouse_mode = -1
