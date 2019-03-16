#include <swarm_detection/drone_pose_estimator.h>
#include <assert.h> 
#include <swarm_detection/drone_pose_costfunc.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace ros;

inline float rand_FloatRange(float a, float b)
{
    return ((b - a) * ((float)rand() / RAND_MAX)) + a;
}


DronePoseEstimator::DronePoseEstimator(SwarmDroneDefs  & _r_drone_defs, camera_array & _sca):
    remote_drone_defs(_r_drone_defs), self_ca(_sca){
    
}

double DronePoseEstimator::estimate_drone_pose(std::vector<corner_array> & point_by_cam){
    assert(point_by_cam.size() == self_ca.size());
    Pose pose;
    pose.position.x() = rand_FloatRange(-0.3, 0.3);
    pose.position.y() = rand_FloatRange(-0.3, 0.3);
    pose.position.z() = rand_FloatRange(0.1, 0.5);

    pose.attitude = Quaterniond(AngleAxisd(
        rand_FloatRange(-3.14, 3.14),
        Vector3d(rand_FloatRange(-1, 1), rand_FloatRange(-1, 1), rand_FloatRange(-1, 1))
    ));

    double cost = this->estimate_drone_pose(point_by_cam, pose);
    return cost;
}

void DronePoseEstimator::draw(double x[], std::vector<corner_array> & point_by_cam) {
    auto err = new DronePoseReprojectionError( point_by_cam, self_ca);
    cv::Mat &new_to_draw1 = mat_to_draw_1;
    cv::Mat &new_to_draw2 = mat_to_draw_2;
    const Camera * cam_def = self_ca[0];
    for (auto mco : point_by_cam[0]) {
//         std::cout << "mco pos" << mco.rel_corner_pos() << std::endl;
        auto point3d = err->Point3dtoProj<double>(cam_def, mco, x);
        auto predict_point = err->PredictPoint<double>(cam_def, mco, x);
        cv::Point p;
        p.x = predict_point(0)*2;
        p.y = predict_point(1)*2;
        printf("p3d %f %f %f ,pp %d %d\n", point3d(0), point3d(1), point3d(2), p.x, p.y);
        cv::circle(new_to_draw1, p, 10, cv::Scalar(0, 0,255), 2);

        p.x = mco.observed_point.x()*2;
        p.y = mco.observed_point.y()*2;
        printf("op %d %d\n", p.x, p.y);

        cv::circle(new_to_draw1, p, 15, cv::Scalar(255,0,0), 2);
    }

    cam_def = self_ca[1];

    for (auto mco : point_by_cam[1]) {
//         std::cout << "mco pos" << mco.rel_corner_pos() << std::endl;
        auto point3d = err->Point3dtoProj<double>(cam_def, mco, x);
        auto predict_point = err->PredictPoint<double>(cam_def, mco, x);
        cv::Point p;
        p.x = predict_point(0)*2;
        p.y = predict_point(1)*2;
//         printf("p3d %f %f %f ,pp %d %d\n",point3d(0), point3d(1), point3d(2), p.x, p.y);
        cv::circle(new_to_draw2, p, 10, cv::Scalar(0, 0,255), 2);

        p.x = mco.observed_point.x()*2;
        p.y = mco.observed_point.y()*2;
        printf("op %d %d\n", p.x, p.y);

        cv::circle(new_to_draw2, p, 15, cv::Scalar(255,0,0), 2);

    }

    cv::Mat showMat;
    hconcat(new_to_draw1, new_to_draw2, showMat);
    cv::resize(showMat, showMat, cv::Size(1280, 400));
    cv::imshow("predict", showMat);
//    cv::imshow("predict1", new_to_draw1);
//    cv::imshow("predict2", new_to_draw2);

    cv::waitKey(30);

}

double DronePoseEstimator::estimate_drone_pose(std::vector<corner_array> & point_by_cam, Pose initial_pose) {
    Problem ba_problem;
    ceres::DynamicAutoDiffCostFunction<DronePoseReprojectionError, 7>* 
        cost_func = DronePoseReprojectionError::Create(point_by_cam, self_ca);
    int num_res = 0;
    for (auto ca : point_by_cam) {
        num_res += ca.size()*2;
    }

    printf("Number of residual is %d", num_res);
    cost_func ->SetNumResiduals(num_res);
    cost_func->AddParameterBlock(7);

    double x[7] = {0};
    initial_pose.to_vector(x);
    printf("Init Quat %3.2f %3.2f %3.2f %3.2f pos %3.2f %3.2f %3.2f\n\n",
        x[0],x[1],x[2],x[3],x[4],x[5],x[6]);



    ba_problem.AddResidualBlock(cost_func,NULL, x);
    for (int i = 0; i < 4 ; i ++)
    {
        ba_problem.SetParameterLowerBound(x, i, -0.1);
        ba_problem.SetParameterUpperBound(x, i, 0.1);
    }

    for (int i = 4; i < 7 ; i ++)
    {
        ba_problem.SetParameterLowerBound(x, i, -2);
        ba_problem.SetParameterUpperBound(x, i, 2);
    }

    ba_problem.SetParameterLowerBound(x, 6, 0);
    ba_problem.SetParameterUpperBound(x, 6, 2);

    Pose pose;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &ba_problem, &summary);
    std::cout << summary.BriefReport() << "  " << summary.total_time_in_seconds * 1000 << "ms\n";

    printf("Quat %3.2f %3.2f %3.2f %3.2f pos %3.3f %3.3f %3.3f\n\n",
        x[0],x[1],x[2],x[3],x[4],x[5],x[6]);

    if (enable_drawing) {
        draw(x, point_by_cam);
    }
    return summary.final_cost;
}


// SwarmPosesEstimator::SwarmPosesEstimator(std::string swarm_maker_config) {

// }


