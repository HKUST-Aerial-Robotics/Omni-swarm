#include <swarm_detection/drone_pose_estimator.h>
#include <assert.h> 
#include <swarm_detection/drone_pose_costfunc.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
using namespace ros;

inline float rand_FloatRange(float a, float b)
{
    return ((b - a) * ((float)rand() / RAND_MAX)) + a;
}


DronePoseEstimator::DronePoseEstimator(SwarmDroneDefs  & _r_drone_defs, camera_array & _sca):
    remote_drone_defs(_r_drone_defs), self_ca(_sca){
    
}

Pose DronePoseEstimator::estimate_drone_pose(std::vector<corner_array> &point_by_cam) {
    assert(point_by_cam.size() == self_ca.size());
    Pose pose;
    /*
    pose.position.x() = rand_FloatRange(-0.3, 0.3);
    pose.position.y() = rand_FloatRange(-0.3, 0.3);
    pose.position.z() = rand_FloatRange(0.1, 0.5);

    pose.attitude = Quaterniond(AngleAxisd(
        rand_FloatRange(-3.14, 3.14),
        Vector3d(rand_FloatRange(-1, 1), rand_FloatRange(-1, 1), rand_FloatRange(-1, 1))
    ));
     */
    //Use pnp to solve initial first


    for (int i = 0; i < point_by_cam.size(); i++) {
        if (point_by_cam[i].size() > 0) {
            ROS_INFO("Using camera %d pnp as init\n", i);
            std::vector<cv::Point2f> pts2d;
            std::vector<cv::Point3f> pts3d;

            corner_array ca = point_by_cam[i];

            for (MarkerCornerObservsed cor: ca) {
                pts2d.emplace_back(cv::Point2f(cor.p_undist.x(), cor.p_undist.y()));

                Eigen::Vector3d _p3d = cor.rel_corner_pos();
                pts3d.emplace_back(cv::Point3f(_p3d.x(), _p3d.y(), _p3d.z()));

//                printf("Add point %f %f 3d %f %f %f\n",
//                        cor.p_undist.x(),cor.p_undist.y(),_p3d.x(), _p3d.y(), _p3d.z());
            }

            cv::Mat r, rvec, t, D;
            rvec = (cv::Mat_<double>(3, 1) << 0, 0, 0);
            t = (cv::Mat_<double>(3, 1) << 1, 0, 0);
//            t<< 0, 0, 0;
            cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
            bool pnp_succ;

            ros::Time t1 = ros::Time::now();
            pnp_succ = cv::solvePnP(pts3d, pts2d, K, D, rvec, t);
            double dt = (ros::Time::now() - t1).toSec();

            if (!pnp_succ) {
                ROS_INFO("PNP Failed, try other");
            } else {
//                printf("PNP Sucess");
                Eigen::MatrixXd R_pnp, T_pnp;
//                std::cout << "r" << rvec << std::endl;
                cv::Rodrigues(rvec, r);

//                std::cout << "r "<< r <<" t "<<t << std::endl;

                cv::cv2eigen(r, R_pnp);
                cv::cv2eigen(t, T_pnp);


                //R_pnp and T_pnp is camera pose in target

                Eigen::Matrix3d R;
                Eigen::Vector3d P;

                R = R_pnp;
                P = T_pnp;


                pose.set_pos(P);
                pose.set_att(Eigen::Quaterniond(R));

                Eigen::Isometry3d drone_trans = pose.to_isometry();

                drone_trans = self_ca[i]->pose.to_isometry() * drone_trans;

                pose = Pose(drone_trans);
//                std::cout << "TPNP " << T_pnp << std::endl;

//                std::cout <<"Res !R\n" << R << "\nT\n" << P << std::endl <<"Pse::";
                ROS_INFO("PNP Pose %3.2fms:  ", dt * 1000);
                pose.print();

                break;
            }
        }
    }

    int useful_camera_count = 0;
    for (int i = 0; i < point_by_cam.size(); i++) {
        if (point_by_cam[i].size() > 0) {
            useful_camera_count++;
        }
    }
    if (useful_camera_count > 1 && use_ba) {
        pose = this->estimate_drone_pose(point_by_cam, pose);
    } else {
        if (enable_drawing) {
            double x[7] = {0};
            pose.to_vector(x);
            draw(x, point_by_cam);
        }
        ROS_INFO("Not use ba or drone Seen by single camera, no need for BA");
    }
    return pose;
}

void DronePoseEstimator::draw(double x[], std::vector<corner_array> &point_by_cam, std::string name) {
    auto err = new DronePoseReprojectionError( point_by_cam, self_ca);
    cv::Mat &new_to_draw1 = mat_to_draw_1;
    cv::Mat &new_to_draw2 = mat_to_draw_2;
    const Camera * cam_def = self_ca[0];
    // printf("Draw : %s\n", name.c_str());
    double res[8] = {0};

    for (auto mco : point_by_cam[0]) {
//         std::cout << "mco pos" << mco.rel_corner_pos() << std::endl;
        auto point3d = err->Point3dtoProj<double>(cam_def, mco, x);
        auto predict_point = err->PredictPoint<double>(cam_def, mco, x);
        cv::Point p;
        p.x = predict_point(0)*2;
        p.y = predict_point(1)*2;
//        printf("p3d %f %f %f ,pp %d %d\n", point3d(0), point3d(1), point3d(2), p.x, p.y);
        if (name == "BA") {
            cv::circle(new_to_draw1, p, 3, cv::Scalar(0, 0, 255), -1);
        } else {
            cv::circle(new_to_draw1, p, 10, cv::Scalar(0, 255, 255), 2);
        }

//        printf("Predict op %3.1f %3.1f obser %3.1f %3.1f\n", predict_point(0)*2, predict_point(1)*2, mco.observed_point.x()*2, mco.observed_point.y()*2);

        p.x = mco.observed_point.x()*2;
        p.y = mco.observed_point.y()*2;

        cv::circle(new_to_draw1, p, 15, cv::Scalar(255,0,0), 2);
    }

    if (point_by_cam.size() > 1) {

        cam_def = self_ca[1];

        for (auto mco : point_by_cam[1]) {
            //         std::cout << "mco pos" << mco.rel_corner_pos() << std::endl;
            auto point3d = err->Point3dtoProj<double>(cam_def, mco, x);
            auto predict_point = err->PredictPoint<double>(cam_def, mco, x);
            cv::Point p;
            p.x = predict_point(0) * 2;
            p.y = predict_point(1) * 2;

            if (name == "BA") {
                cv::circle(new_to_draw2, p, 3, cv::Scalar(0, 0, 255), -1);
            } else {
                cv::circle(new_to_draw2, p, 10, cv::Scalar(0, 255, 255), 2);
            }

            p.x = mco.observed_point.x() * 2;
            p.y = mco.observed_point.y() * 2;
            //        printf("op %d %d\n", p.x, p.y);

            cv::circle(new_to_draw2, p, 15, cv::Scalar(255, 0, 0), 2);

        }
        cv::Mat showMat;
        hconcat(new_to_draw1, new_to_draw2, showMat);
        cv::resize(showMat, showMat, cv::Size(1280, 400));
        cv::imshow("predict", showMat);
    } else {
        cv::imshow("predict", new_to_draw1);
    }

    cv::waitKey(10);

}

Pose DronePoseEstimator::estimate_drone_pose(std::vector<corner_array> &point_by_cam, Pose initial_pose) {
    Problem ba_problem;
    ceres::DynamicAutoDiffCostFunction<DronePoseReprojectionError, 7>* 
        cost_func = DronePoseReprojectionError::Create(point_by_cam, self_ca);
    int num_res = 0;
    for (auto ca : point_by_cam) {
        num_res += ca.size()*2;
    }

    cost_func ->SetNumResiduals(num_res);
    cost_func->AddParameterBlock(7);

    double x[7] = {0};
    initial_pose.to_vector(x);

//    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
//    ba_problem.AddParameterBlock(x, 7, local_parameterization);

    ba_problem.AddResidualBlock(cost_func, NULL, x);

    Pose pose;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &ba_problem, &summary);
    std::cout << summary.BriefReport() << "  " << summary.total_time_in_seconds * 1000 << "ms\n";

    pose = Pose(x);
    printf("Final pose");
    pose.print();
    if (enable_drawing) {
        draw(x, point_by_cam);
    }
    return pose;
}


// SwarmPosesEstimator::SwarmPosesEstimator(std::string swarm_maker_config) {

// }


