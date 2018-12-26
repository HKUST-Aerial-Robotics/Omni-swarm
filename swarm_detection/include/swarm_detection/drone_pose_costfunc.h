#pragma once
#include <swarm_detection/swarm_detect_types.h>


template <typename T>
inline void InverseTrans(const T pose[7], T pose_output[7]) {
    pose_output[0] = - pose[0];
    pose_output[1] = pose[1];
    pose_output[2] = pose[2];
    pose_output[3] = pose[3];
    pose_output[4] = - pose[4];
    pose_output[5] = - pose[5];
    pose_output[6] = - pose[6];
}

template <typename T>
inline void AppleTrans2Point(const T pose[7],const T point[3], T p[3]) {
    QuaternionRotatePoint(pose, point, p);
    p[0] += pose[4];
    p[1] += pose[5];
    p[2] += pose[6];
}

template <typename T>
inline void MultiplyTrans(const T posea[7],const T poseb[7], T ret[7]) {
    QuaternionProduct(posea, poseb, ret);
    T pointb[3] ;
    T respoint[3];
    
    pointb[0] = poseb[4];
    pointb[1] = poseb[5];
    pointb[2] = poseb[6];

    QuaternionRotatePoint(posea, pointb, respoint);
    ret[4] = respoint[0] + posea[0];
    ret[5] = respoint[1] + posea[1];
    ret[6] = respoint[2] + posea[2];
}

template <typename T>
inline void TransFromVecQuat(const Vector3d vec, const Quaterniond quat,T trans[7]) {
    trans[0] = T(quat.w());
    trans[1] = T(quat.x());
    trans[2] = T(quat.y());
    trans[3] = T(quat.z());
    trans[4] = T(vec.x());
    trans[5] = T(vec.y());
    trans[6] = T(vec.z());
}


struct DronePoseReprojectionError {
    std::vector<corner_array> point_by_cam;
    camera_array cam_array;

    DronePoseReprojectionError(std::vector<corner_array> _point_by_cam, camera_array & _ca) : 
        point_by_cam(_point_by_cam), cam_array(_ca)
    {

    }

    
    template <typename T>
    bool operator()(const T* const pose,
                    T* residuals) const {
        //Pose 0,1,2,3 quat
        //Pose 4,5,6 position
        int res_count = 0;
        for (int i = 0; i < point_by_cam.size(); i ++) {
            const Camera * cam_def = cam_array[i];
            for (MarkerCornerObervsed mco : point_by_cam[i]) {
                //Calculate a residual
                // residuals[res_count] = 
                //proj(Tcam^-1 * Tik * Pmpni) -Zn
                //Tik is pose of target drone, i.e pose
                T cam_pose[7];
                T _tmp[7];
                T _tmp2[7];

                TransFromVecQuat(cam_def->pos(), cam_def->att(), cam_pose);

                T Pmpni[3];
                Vector3d pmpni = mco.rel_corner_pos();
                Pmpni[0] = T(pmpni.x());
                Pmpni[1] = T(pmpni.y());
                Pmpni[2] = T(pmpni.z());

                T point_to_proj[3];
                
                InverseTrans(cam_pose, _tmp);
                MultiplyTrans(_tmp, pose, _tmp2);

                AppleTrans2Point(_tmp2, Pmpni, point_to_proj);

                //Point should on camera
                T predict_point[2];
                cam_def->project_to_camera(point_to_proj, predict_point);

                residuals[res_count] = predict_point[0] - mco.observed_point.x();
                res_count ++;
                residuals[res_count] = predict_point[1] - mco.observed_point.y();
                res_count ++;
            }
        }
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(std::vector<corner_array> _point_by_cam, camera_array _ca) {
        return (new ceres::AutoDiffCostFunction<
                DronePoseReprojectionError, ceres::DYNAMIC, 7>(
                    new DronePoseReprojectionError( _point_by_cam, _ca)));
    }

};

