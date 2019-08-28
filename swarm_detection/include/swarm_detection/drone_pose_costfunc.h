#pragma once
#include <swarm_detection/swarm_detect_types.h>

template <typename T>
inline void InverseTrans(const T pose[7], T pose_output[7]) {
    pose_output[3] = -pose[3];
    pose_output[4] = pose[4];
    pose_output[5] = pose[5];
    pose_output[6] = pose[6];
    pose_output[0] = - pose[0];
    pose_output[1] = -pose[1];
    pose_output[2] = -pose[2];
}

template <typename T>
inline void ApplyTrans2Point(const T pose[7],const T point[3], T p[3]) {
    QuaternionRotatePoint(pose + 3, point, p);
    p[0] += pose[0];
    p[1] += pose[1];
    p[2] += pose[2];
}

template <typename T>
inline void MultiplyTrans(const T posea[7],const T poseb[7], T ret[7]) {
    QuaternionProduct(posea + 3, poseb + 3, ret + 3);
    T pointb[3] ;
    T respoint[3];

    pointb[0] = poseb[0];
    pointb[1] = poseb[1];
    pointb[2] = poseb[2];

    QuaternionRotatePoint(posea + 3, pointb, respoint);
    ret[0] = respoint[0] + posea[0];
    ret[1] = respoint[1] + posea[1];
    ret[2] = respoint[2] + posea[2];
}

template <typename T>
inline void TransFromVecQuat(const Vector3d vec, const Quaterniond quat,T trans[7]) {
    trans[3] = T(quat.w());
    trans[4] = T(quat.x());
    trans[5] = T(quat.y());
    trans[6] = T(quat.z());
    trans[0] = T(vec.x());
    trans[1] = T(vec.y());
    trans[2] = T(vec.z());
}


struct DronePoseReprojectionError {
    std::vector<corner_array> point_by_cam;
    camera_array cam_array;

    DronePoseReprojectionError(std::vector<corner_array> _point_by_cam, camera_array & _ca) :
            point_by_cam(std::move(_point_by_cam)), cam_array(_ca)
    {

    }

    template <typename T>
    Eigen::Matrix<T, 3, 1> Point3dtoProj(const Camera * cam_def, const MarkerCornerObservsed & mco, const T * pose) const {
        T Pmpni[3];
        Vector3d pmpni = mco.rel_corner_pos();
        Pmpni[0] = T(pmpni.x());
        Pmpni[1] = T(pmpni.y());
        Pmpni[2] = T(pmpni.z());

        T point_to_proj[3];

        ApplyTrans2Point(pose, Pmpni, point_to_proj);

        //Point should on camera
        return Eigen::Matrix<T, 3, 1>(point_to_proj[0], point_to_proj[1], point_to_proj[2]);
    }

    template <typename T>
    Eigen::Matrix<T, 2, 1> PredictPoint(const Camera * cam_def, const MarkerCornerObservsed & mco, const T * pose) const {
        Eigen::Matrix<T, 3, 1>  P = Point3dtoProj(cam_def, mco, pose);
        Eigen::Matrix<T, 2, 1> predict_point;

        cam_def->project_to_camera(P, predict_point);

        return predict_point;
    }

    template<typename T>
    Eigen::Matrix<T, 2, 1>
    PredictPointUndist(const Camera *cam_def, const MarkerCornerObservsed &mco, const T *pose) const {
        Eigen::Matrix<T, 3, 1> P = Point3dtoProj(cam_def, mco, pose);
        Eigen::Matrix<T, 2, 1> predict_point;

        cam_def->project_to_camera_undist(P, predict_point);

        return predict_point;
    }


    template <typename T>
    bool operator()(T const* const* parameters,
                    T* residuals) const {
        const T * pose = parameters[0];
        //Pose 0,1,2,3 quat
        //Pose 4,5,6 position
        int res_count = 0;
        for (int i = 0; i < point_by_cam.size(); i ++) {
            const Camera * cam_def = cam_array[i];
            for (auto mco : point_by_cam[i]) {
                Eigen::Matrix<T, 2, 1> predict_point = PredictPointUndist(cam_def, mco, pose);
                residuals[res_count] = predict_point(0) - mco.p_undist.x();
                res_count ++;
                residuals[res_count] = predict_point(1) - mco.p_undist.y();
                res_count ++;
            }
        }
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::DynamicAutoDiffCostFunction<DronePoseReprojectionError, 7>* 
        Create(std::vector<corner_array> _point_by_cam, camera_array _ca) {
        return (new ceres::DynamicAutoDiffCostFunction<
                DronePoseReprojectionError, 7>(
                    new DronePoseReprojectionError( _point_by_cam, _ca)));
    }

};

