#include <swarm_localization/swarm_outlier_rejection.hpp>
#include <fstream>
#include <stdio.h>
#include "third_party/fast_max-clique_finder/src/graphIO.h"
#include "third_party/fast_max-clique_finder/src/findClique.h"
#include "swarm_localization/swarm_localization_factors.hpp"

#define PCM_DEBUG_OUTPUT

std::fstream pcm_good;
std::fstream pcm_good_detection;
std::fstream pcm_errors;
FILE * f_logs;


std::vector<Swarm::DroneDetection> SwarmLocalOutlierRejection::OutlierRejectionDetections(const std::vector<Swarm::DroneDetection> & detections) {
    std::vector<Swarm::DroneDetection> good_dets;


    std::map<int, std::map<int, std::vector<Swarm::DroneDetection>>> inter_edges;
    bool has_new_detection = false;
    for (auto & edge: detections) {
        if (edge.is_inter_loop()) {
            if (all_detections_set.find(edge.id) == all_detections_set.end()) {
                all_detections_set.insert(edge.id);
                inter_edges[edge.id_a][edge.id_b].emplace_back(edge);
                inter_edges[edge.id_b][edge.id_a].emplace_back(edge);
                has_new_detection = true;
            }
        }
    }


    if (param.debug_write_pcm_good && has_new_detection) {
        pcm_good_detection.open("/root/output/pcm_good_det.txt", std::ios::out);
    }

    for (auto it_a: inter_edges) {
        for (auto it_b: it_a.second) {
            if (it_a.first > it_b.first) {
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) Detection Inter-LCM drone%d<->drone%d", it_a.first, it_b.first);
                OutlierRejectionDetectionsPCM(it_b.second, it_a.first, it_b.first);
                for (auto det : it_b.second) {
                    auto _good_detections_set = good_detections_set[it_a.first][it_b.first];
                    if (_good_detections_set.find(det.id) != _good_detections_set.end()) {
                        good_dets.emplace_back(det);
                    }
                }
            }
        }
    }

    if (param.debug_write_pcm_good && has_new_detection) {
        pcm_good_detection.close();
    }

    return good_dets;
}

void SwarmLocalOutlierRejection::OutlierRejectionDetectionsPCM(
        const std::vector<Swarm::DroneDetection > & new_detections, int ida, int idb) {
    TicToc tic1;
    auto & pcm_graph = det_pcm_graph[ida][idb];
    auto & _good_detections_set = good_detections_set[ida][idb];
    auto & _all_detection_set = all_detections_set_by_pair[ida][idb];
    auto & _all_detections = all_detections[ida][idb];

    for (size_t i = 0; i < new_detections.size(); i++) {

        auto det1 = new_detections[i];
        while (pcm_graph.size() < _all_detections.size() + 1) {
            pcm_graph.emplace_back(std::vector<int>(0));
        }

        //Now only process inter-edges
        for (size_t j = 0; j < _all_detections.size(); j++) {

            //Now only process inter-edges
            auto det2 = _all_detections[j];
            det1.enable_depth = true;
            det2.enable_depth = true;
            det1.enable_dpose = true;
            det2.enable_dpose = true;
            det1.dpose_self_a = det1.extrinsic;
            det1.dpose_self_b = det1.GC;
            det2.dpose_self_a = det2.extrinsic;
            det2.dpose_self_b = det2.GC;

            int same_robot_pair = det2.same_robot_pair(det1);
            if (same_robot_pair > 0) {
                Problem problem;
                //Now we can compute the consistency error.
                std::pair<Swarm::Pose, Matrix6d> odom_a, odom_b;
                Swarm::Pose p_edge2;
                Matrix6d _covariance;
                
                double * pose_a1 = new double[4];
                double * pose_a2 = new double[4];
                double * pose_b1 = new double[4];
                double * pose_b2 = new double[4];
                
                auto cost_det1 = DroneDetection4dFactor::Create(det1);
                auto cost_det2 = DroneDetection4dFactor::Create(det2);

                Eigen::Map<Vector4d> _pose_a1(pose_a1);
                Eigen::Map<Vector4d> _pose_a2(pose_a2);
                Eigen::Map<Vector4d> _pose_b1(pose_b1);
                Eigen::Map<Vector4d> _pose_b2(pose_b2);
                _pose_a1.setZero(); 
                _pose_a2.setZero();
                _pose_b1.setZero();
                _pose_b2.setZero();

                _pose_b1.block<3, 1>(0, 0) = det1.p / det1.inv_dep; // Initial b1 by detection and zero yaw.

                if (same_robot_pair == 1) {
                    //ODOM is tsa->tsb
                    odom_a = ego_motion_trajs.at(det1.id_a).get_relative_pose_by_ts(det1.ts_a, det2.ts_a, true);
                    odom_b = ego_motion_trajs.at(det1.id_b).get_relative_pose_by_ts(det1.ts_b, det2.ts_b, true);

                    _covariance = odom_a.second + odom_b.second;

                    if (odom_a.first.pos().norm() > 0.01) {
                        _pose_a2.block<3, 1>(0, 0) = odom_a.first.pos();
                        _pose_a2(3) = odom_a.first.yaw();
                    } else {
                        pose_a2 = pose_a1;
                    }

                    if (odom_b.first.pos().norm() > 0.01) {
                        _pose_b2.block<3, 1>(0, 0) =  _pose_a2.block<3, 1>(0, 0) + det2.p / det2.inv_dep; // Initial b2 pos by detection and zero yaw.
                    } else {
                        pose_b2 = pose_b1;
                    }
                    
                    problem.AddResidualBlock(cost_det1, nullptr, pose_a1, pose_b1);
                    problem.AddResidualBlock(cost_det2, nullptr, pose_a2, pose_b2);


                }  else if (same_robot_pair == 2) {
                    odom_a = ego_motion_trajs.at(det1.id_a).get_relative_pose_by_ts(det1.ts_a, det2.ts_b, true);
                    odom_b = ego_motion_trajs.at(det1.id_b).get_relative_pose_by_ts(det1.ts_b, det2.ts_a, true);
                    _covariance = odom_a.second + odom_b.second;

                    if (odom_a.first.pos().norm() > 0.01) {
                        _pose_a2.block<3, 1>(0, 0) = odom_a.first.pos();
                        _pose_a2(3) = odom_a.first.yaw();
                    } else {
                        pose_a2 = pose_a1;
                    }

                    if (odom_b.first.pos().norm() > 0.01) {
                        _pose_b2.block<3, 1>(0, 0) =  _pose_a2.block<3, 1>(0, 0) - det2.p / det2.inv_dep; // Initial b2 pos by detection and zero yaw.
                    } else {
                        pose_b2 = pose_b1;
                    }

                    problem.AddResidualBlock(cost_det1, nullptr, pose_a1, pose_b1);
                    problem.AddResidualBlock(cost_det2, nullptr, pose_b2, pose_a2);
                }

                //Now we need solve a small problem to get the SquaredMahalanobisDistance
                
                if (pose_a1 != pose_a2) {
                    auto cf_odoma = RelativePoseFactor4d::CreateCov6d(odom_a.first, odom_a.second);
                    problem.AddResidualBlock(cf_odoma, nullptr, pose_a1, pose_a2);
                }

                if (pose_b1 != pose_b2) {
                    auto cf_odomb = RelativePoseFactor4d::CreateCov6d(odom_b.first, odom_b.second);
                    problem.AddResidualBlock(cf_odomb, nullptr, pose_b1, pose_b2);
                }

                problem.SetParameterBlockConstant(pose_a1);

                //TODO: Speed up this, should be at least 10 times faster.
                ceres::Solver::Options options;
                options.max_num_iterations = 100;
                // options.linear_solver_type = ceres::DENSE_QR;
                // options.minimizer_type = ceres::LINE_SEARCH;
                options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
                options.max_solver_time_in_seconds = 0.1;
                // std::cout << "_pose_a2 " << _pose_a2.transpose() << "_pose_b1 " << _pose_b1.transpose() << "_pose_b2 " << _pose_b2.transpose() << std::endl;

                Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);

                double SqrtMahalanobisDistance = summary.final_cost*2;
                //This can be treated as MahlanobisDistance.
                // ROS_INFO("[SWARM_LOCAL](Debug) Det 1 %d->%d dir [%+3.1f,%+3.1f,%+3.1f] enable_depth %d",
                //     det1.id_a, det1.id_b, det1.p.x(), det1.p.y(), det1.p.z(), det1.enable_depth);
            
                // ROS_INFO("[SWARM_LOCAL](Debug) Det 2 %d->%d dir [%+3.1f,%+3.1f,%+3.1f] enable_depth %d",
                //     det2.id_a, det2.id_b, det2.p.x(), det2.p.y(), det2.p.z(), det2.enable_depth);


                // std::cout << summary.FullReport() << std::endl;

                
                // std::cout << "_pose_a2 " << _pose_a2.transpose() << "_pose_b1 " << _pose_b1.transpose() << "_pose_b2 " << _pose_b2.transpose() << std::endl;

                if (SqrtMahalanobisDistance < param.pcm_thres_det) { //Temporary use same with detection.
                    //Add edge i to j
                    pcm_graph[_all_detections.size()].push_back(j);
                    pcm_graph[j].push_back(_all_detections.size());
                }
                
            }
        }

        _all_detections.push_back(det1);
        _all_detection_set.insert(det1.id);
    }

    double compute_pcm_erros = tic1.toc();

    FMC::CGraphIO pcm_graph_fmc;
    pcm_graph_fmc.m_vi_Vertices.push_back(0);

	for(size_t i=0;i < pcm_graph.size(); i++) {
		pcm_graph_fmc.m_vi_Edges.insert(pcm_graph_fmc.m_vi_Edges.end(),pcm_graph[i].begin(),pcm_graph[i].end());
		pcm_graph_fmc.m_vi_Vertices.push_back(pcm_graph_fmc.m_vi_Edges.size());
	}

    pcm_graph_fmc.CalculateVertexDegrees();
    std::vector<int> max_clique_data;
    TicToc tic;
    auto max_clique_size = FMC::maxCliqueHeu(pcm_graph_fmc, max_clique_data);
    ROS_INFO("[SWARM_LOCAL](OutlierRejection Detection) compute_pcm_errors %.1fms maxCliqueHeu takes %.1fms new_added_dets %ld all_dets %ld good %ld", 
        compute_pcm_erros, tic.toc(), new_detections.size(), _all_detections.size(), max_clique_data.size());

    _good_detections_set.clear();
    std::vector<Swarm::DroneDetection> good_detections;
    for (auto i : max_clique_data) {
        _good_detections_set.insert(_all_detections[i].id);
        if (param.debug_write_pcm_good) {
            pcm_good_detection << _all_detections[i].id << std::endl;
        }
    }
}


std::vector<Swarm::LoopEdge> SwarmLocalOutlierRejection::OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops) {
    if (param.debug_write_pcm_good) {
        pcm_good.open("/root/output/pcm_good.txt", std::ios::out);
    }

    if (param.debug_write_pcm_errors) {
        pcm_errors.open("/root/output/pcm_errors.txt", std::ios::out);
        f_logs = fopen("/root/output/pcm_logs.txt", "w");
    }

    std::map<int, std::map<int, std::vector<Swarm::LoopEdge>>> inter_loops;
    std::map<int, std::vector<Swarm::LoopEdge >> intra_loops;
    std::vector<Swarm::LoopEdge> good_loops;
    for (auto & edge: available_loops) {
        if (edge.is_inter_loop()) {
            inter_loops[edge.id_a][edge.id_b].emplace_back(edge);
            inter_loops[edge.id_b][edge.id_a].emplace_back(edge);
        } else {
            intra_loops[edge.id_a].emplace_back(edge);
        }
    }

    for (auto it : intra_loops) {
        ROS_INFO("[SWARM_LOCAL](OutlierRejection) Intra-LCM drone %d", it.first);
        auto good_intra_loops = OutlierRejectionLoopEdgesPCM(it.second);
        good_loops.insert( good_loops.end(), good_intra_loops.begin(), good_intra_loops.end() );

    }


    for (auto it_a: inter_loops) {
        for (auto it_b: it_a.second) {
            if (it_a.first > it_b.first) {
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) Inter-LCM drone%d<->drone%d", it_a.first, it_b.first);
                auto good_inter_loops = OutlierRejectionLoopEdgesPCM(it_b.second);
                good_loops.insert( good_loops.end(), good_inter_loops.begin(), good_inter_loops.end() );
            }
        }
    }

    if (param.debug_write_pcm_good) {
        pcm_good.close();
    }

    if (param.debug_write_pcm_errors) {
        pcm_errors.close();
        fclose(f_logs);
    }


    return good_loops;
}

bool SwarmLocalOutlierRejection::check_outlier_by_odometry_consistency(const Swarm::LoopEdge & loop) {
    return false;
}

std::vector<Swarm::LoopEdge> SwarmLocalOutlierRejection::OutlierRejectionLoopEdgesPCM(const std::vector<Swarm::LoopEdge > & available_loops) {
    std::map<FrameIdType, int> bad_pair_count;

    if (!param.enable_pcm) {
        return available_loops;
    }


    std::vector<std::vector<int>> pcm_graph(available_loops.size());
    TicToc tic1;

    for (size_t i = 0; i < available_loops.size(); i++) {
        auto & edge1 = available_loops[i];
        //Now only process inter-edges

        auto p_edge1 = edge1.relative_pose;
        Matrix6d _cov_mat_1 = edge1.get_covariance();

        for (size_t j = 0; j < i; j ++) {
            //Now only process inter-edges
            auto & edge2 = available_loops[j];
            Matrix6d _covariance = _cov_mat_1 + edge2.get_covariance();

            int same_robot_pair = edge2.same_robot_pair(edge1);
            if (same_robot_pair > 0) {
                //Now we can compute the consistency error.
                std::pair<Swarm::Pose, Matrix6d> odom_a, odom_b;
                Swarm::Pose p_edge2;

                if (same_robot_pair == 1) {
                    p_edge2 = edge2.relative_pose;
                    //ODOM is tsa->tsb
                    odom_a = ego_motion_trajs.at(edge1.id_a).get_relative_pose_by_ts(edge1.ts_a, edge2.ts_a, true);
                    odom_b = ego_motion_trajs.at(edge1.id_b).get_relative_pose_by_ts(edge1.ts_b, edge2.ts_b, true);

                    _covariance += odom_a.second + odom_b.second;

                }  else if (same_robot_pair == 2) {
                    p_edge2 = edge2.relative_pose.inverse();
                    odom_a = ego_motion_trajs.at(edge1.id_a).get_relative_pose_by_ts(edge1.ts_a, edge2.ts_b, true);
                    odom_b = ego_motion_trajs.at(edge1.id_b).get_relative_pose_by_ts(edge1.ts_b, edge2.ts_a, true);

                    _covariance += odom_a.second + odom_b.second;
                }

                Swarm::Pose err_pose = odom_a.first*p_edge2*odom_b.first.inverse()*p_edge1.inverse();
                auto logmap = err_pose.log_map();
                double smd = Swarm::computeSquaredMahalanobisDistance(logmap, _covariance);

                if (smd < param.pcm_thres) {
                    //Add edge i to j
                    pcm_graph[i].push_back(j);
                    pcm_graph[j].push_back(i);
                }

                if (param.debug_write_pcm_errors) {
                    fprintf(f_logs, "\n");
                    fprintf(f_logs, "EdgePair %ld->%ld\n", edge1.id, edge2.id);
                    fprintf(f_logs, "Edge1 %ld@%d->%ld@%d DOF %d Pose %s\n", 
                        edge1.ts_a, edge1.id_a,
                        edge1.ts_b, edge1.id_b,
                        edge1.res_count,
                        edge1.relative_pose.tostr().c_str()
                    );
                    fprintf(f_logs, "Edge2 %ld@%d->%ld@%d DOF %d Pose %s\n", 
                        edge2.ts_a, edge2.id_a,
                        edge2.ts_b, edge2.id_b,
                        edge1.res_count,
                        edge2.relative_pose.tostr().c_str()
                    );
                    fprintf(f_logs, "odom_a %s cov YPR [%+3.1e] T [%+3.1e]\n", odom_a.first.tostr().c_str(), odom_a.second(0), odom_a.second(3));
                    fprintf(f_logs, "odom_b %s cov YPR [%+3.1e] T [%+3.1e]\n", odom_b.first.tostr().c_str(), odom_b.second(0), odom_b.second(3));
                    fprintf(f_logs, "err_pose %s logmap [%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e]\n", err_pose.tostr().c_str(), 
                        logmap(0), logmap(1), logmap(2), logmap(3), logmap(4), logmap(5));
                    fprintf(f_logs, "squaredMahalanobisDistance %f Same Direction %d _cov  [%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e]\n", smd, same_robot_pair == 1,
                        _covariance(0, 0),
                        _covariance(1, 1),
                        _covariance(2, 2),
                        _covariance(3, 3),
                        _covariance(4, 4),
                        _covariance(5, 5));
                        pcm_errors << edge1.id << " " << edge2.id << " "  << smd << " " << std::endl;
                }

            }
        }
    }

    double compute_pcm_erros = tic1.toc();

    FMC::CGraphIO pcm_graph_fmc;
    pcm_graph_fmc.m_vi_Vertices.push_back(0);

	for(size_t i=0;i < pcm_graph.size(); i++) {
		pcm_graph_fmc.m_vi_Edges.insert(pcm_graph_fmc.m_vi_Edges.end(),pcm_graph[i].begin(),pcm_graph[i].end());
		pcm_graph_fmc.m_vi_Vertices.push_back(pcm_graph_fmc.m_vi_Edges.size());
	}

    pcm_graph_fmc.CalculateVertexDegrees();
    std::vector<int> max_clique_data;
    TicToc tic;
    auto max_clique_size = FMC::maxCliqueHeu(pcm_graph_fmc, max_clique_data);
    ROS_INFO("[SWARM_LOCAL](OutlierRejection) compute_pcm_errors %.1fms maxCliqueHeu takes %.1fms inter_loop %ld good %ld", 
        compute_pcm_erros, tic.toc(), available_loops.size(), max_clique_data.size());

    std::vector<Swarm::LoopEdge> good_loops;
    for (auto i : max_clique_data) {
        good_loops.emplace_back(available_loops[i]);
        if (param.debug_write_pcm_good) {
            pcm_good << available_loops[i].id << std::endl;
        }
    }

    return good_loops;
}