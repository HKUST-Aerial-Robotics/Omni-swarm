#include <swarm_detection/drone_pose_estimator.h>
#include <assert.h> 
#include <swarm_detection/drone_pose_costfunc.h>

inline float rand_FloatRange(float a, float b)
{
    return ((b - a) * ((float)rand() / RAND_MAX)) + a;
}


DronePoseEstimator::DronePoseEstimator(SwarmDroneDefs  & _drone_defs, camera_array & _ca):
    drone_defs(_drone_defs), ca(_ca){
    
}

Pose DronePoseEstimator::estimate_drone_pose(std::vector<corner_array> & point_by_cam){
    assert(point_by_cam.size() == ca.size());
    Pose pose;
    for (int i = 0; i < 10; i++) {
        pose.pos.x() = rand_FloatRange(0.2, 5);
        pose.pos.y() = rand_FloatRange(-2, 2);
        pose.pos.z() = rand_FloatRange(3, 3);

        pose.quat = Quaterniond(AngleAxisd(
            rand_FloatRange(-3.14, 3.14),
            Vector3d(rand_FloatRange(-1, 1), rand_FloatRange(-1, 1), rand_FloatRange(-1, 1))
        ));

        this->estimate_drone_pose(point_by_cam, pose);
    }

}


Pose DronePoseEstimator::estimate_drone_pose(std::vector<corner_array> & point_by_cam, Pose initial_pose) {
    Problem ba_problem;
    ceres::CostFunction* cost_func = DronePoseReprojectionError::Create(point_by_cam, ca);
    double x[7];
    initial_pose.to_vector(x);
    ba_problem.AddResidualBlock(cost_func,NULL, x);
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &ba_problem, &summary);
    std::cout << summary.FullReport() << "\n";

}


// SwarmPosesEstimator::SwarmPosesEstimator(std::string swarm_maker_config) {

// }
