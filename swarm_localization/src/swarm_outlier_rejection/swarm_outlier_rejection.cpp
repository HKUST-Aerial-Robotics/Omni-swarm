#include <swarm_localization/swarm_outlier_rejection.hpp>

double computeSquaredMahalanobisDistance(Matrix<double, 6, 1> logmap, Matrix<double, 6, 1> cov_vec) {
    Matrix<double, 6, 1>  inf_vec = cov_vec.cwiseInverse();
    auto inf_mat = cov_vec.asDiagonal();
    Eigen::Matrix<double, 1, 1> ret = logmap.transpose() * inf_mat * logmap;
    return std::sqrt(ret(0, 0));
}

std::vector<Swarm::LoopEdge> SwarmLocalOutlierRejection::OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops) {
    std::map<FrameIdType, int> bad_pair_count;
    for (size_t i = 0; i < available_loops.size(); i++) {
        auto & edge1 = available_loops[i];
        //Now only process inter-edges
        if (!edge1.is_inter_loop()) {
            continue;
        }

        auto p_edge1 = edge1.relative_pose;
        auto _cov_vec = edge1.get_cov_vec();

        for (size_t j = 0; j < i; j ++) {
            //Now only process inter-edges
            auto & edge2 = available_loops[j];
            _cov_vec += edge2.get_cov_vec();

            int same_robot_pair = edge2.same_robot_pair(edge1);
            if (same_robot_pair > 0) {
                //Now we can compute the consistency error.
                Swarm::Pose odom_a, odom_b, p_edge2;
                if (same_robot_pair == 1) {
                    p_edge2 = edge2.relative_pose;
                    //ODOM is tsa->tsb
                    odom_a = ego_motion_trajs.at(edge1.id_a).get_relative_pose_by_ts(edge1.ts_a, edge2.ts_a);
                    odom_b = ego_motion_trajs.at(edge1.id_b).get_relative_pose_by_ts(edge1.ts_b, edge2.ts_b);

                    _cov_vec += ego_motion_trajs.at(edge1.id_a).covariance_between_ts(edge1.ts_a, edge2.ts_a, 0.0001, 0.0001);
                    _cov_vec += ego_motion_trajs.at(edge1.id_b).covariance_between_ts(edge1.ts_b, edge2.ts_b, 0.0001, 0.0001);

                }  else if (same_robot_pair == 2) {
                    p_edge2 = edge2.relative_pose.inverse();
                    odom_a = ego_motion_trajs.at(edge1.id_a).get_relative_pose_by_ts(edge1.ts_a, edge2.ts_b);
                    odom_b = ego_motion_trajs.at(edge1.id_b).get_relative_pose_by_ts(edge1.ts_b, edge2.ts_a);

                    _cov_vec += ego_motion_trajs.at(edge1.id_a).covariance_between_ts(edge1.ts_a, edge2.ts_b, 0.0001, 0.0001);
                    _cov_vec += ego_motion_trajs.at(edge1.id_b).covariance_between_ts(edge1.ts_b, edge2.ts_a, 0.0001, 0.0001);
                }
                Swarm::Pose err_pose = odom_a*p_edge2*odom_b.inverse()*p_edge1.inverse();
                auto logmap = err_pose.log_map();
                // printf("\n");
                // ROS_INFO("[SWARM_LOCAL](OutlierRejection) Edge1 %ld@%d->%ld@%d %s", 
                //     edge1.ts_a, edge1.id_a,
                //     edge1.ts_b, edge1.id_b,
                //     edge1.relative_pose.tostr().c_str()
                // );
                // ROS_INFO("[SWARM_LOCAL](OutlierRejection) Edge2 %ld@%d->%ld@%d %s", 
                //     edge2.ts_a, edge2.id_a,
                //     edge2.ts_b, edge2.id_b,
                //     edge2.relative_pose.tostr().c_str()
                // );
                // ROS_INFO("[SWARM_LOCAL](OutlierRejection) odom_a %s", odom_a.tostr().c_str());
                // ROS_INFO("[SWARM_LOCAL](OutlierRejection) odom_b %s", odom_b.tostr().c_str());

                double smd = computeSquaredMahalanobisDistance(logmap, _cov_vec);
                printf("[SWARM_LOCAL](OutlierRejection) EdgePair %ld->%ld squaredMahalanobisDistance %f Same Direction %d\n", edge1.id, edge2.id, smd, same_robot_pair == 1);
                printf("[SWARM_LOCAL](OutlierRejection) err_pose %s logmap", err_pose.tostr().c_str());
                std::cout << logmap.transpose() << std::endl;

            }
        }
    }

    ROS_INFO("Outlier rejection not yet implement. return original.");
    return available_loops;
}