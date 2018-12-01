#include <swarm_detection/drone_pose_estimator.h>
#include <assert.h> 

DronePoseEstimator::DronePoseEstimator(camera_array _ca):
    cam_defs(_ca){
    
}

DronePoseEstimator::estimation_drone_pose(std::vector<corner_array> point_by_cam) {
    assert(point_by_cam.size() == cam_defs.size());
}