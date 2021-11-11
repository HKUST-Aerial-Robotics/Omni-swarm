#include <swarm_localization/swarm_outlier_rejection.hpp>
#include <fstream>
#include <stdio.h>
#include "third_party/fast_max-clique_finder/src/graphIO.h"
#include "third_party/fast_max-clique_finder/src/findClique.h"
#include "swarm_localization/swarm_localization_factors.hpp"

#define PCM_DEBUG_OUTPUT

std::fstream pcm_errors;
FILE * f_logs;

SwarmLocalOutlierRejection::SwarmLocalOutlierRejection(const SwarmLocalOutlierRejectionParams &_param, std::map<int, Swarm::DroneTrajectory> &_ego_motion_trajs):
        param(_param), ego_motion_trajs(_ego_motion_trajs) {
    if (param.debug_write_pcm_errors) {
        f_logs = fopen("/root/output/pcm_logs.txt", "w");
        pcm_errors.open("/root/output/pcm_errors.txt", std::ios::out);
        pcm_errors.close();
        fclose(f_logs);
    }
}


std::vector<Swarm::LoopEdge> SwarmLocalOutlierRejection::OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops) {
    if (param.debug_write_pcm_errors) {
        pcm_errors.open("/root/output/pcm_errors.txt", std::ios::app);
        f_logs = fopen("/root/output/pcm_logs.txt", "a");
    }

    std::map<int, std::map<int, std::vector<Swarm::LoopEdge>>> new_loops;
    std::vector<Swarm::LoopEdge> good_loops;
    for (auto & edge: available_loops) {
        if (all_loops_set.find(edge.id) == all_loops_set.end()) {
            new_loops[edge.id_a][edge.id_b].emplace_back(edge);
            if (edge.id_a!=edge.id_b){
                new_loops[edge.id_b][edge.id_a].emplace_back(edge);
            }
        }
    }

    for (auto it_a: new_loops) {
        for (auto it_b: it_a.second) {
            if (it_a.first >= it_b.first) {
                OutlierRejectionLoopEdgesPCM(it_b.second, it_a.first, it_b.first);
            }
        }
    }

    for (auto & loop : available_loops) {
        auto _good_loops_set = good_loops_set[loop.id_a][loop.id_b];
        if (_good_loops_set.find(loop.id) != _good_loops_set.end()) {
            good_loops.emplace_back(loop);
        }
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

void SwarmLocalOutlierRejection::OutlierRejectionLoopEdgesPCM(const std::vector<Swarm::LoopEdge > & new_loops, int id_a, int id_b) {
    std::map<FrameIdType, int> bad_pair_count;

    auto & pcm_graph = loop_pcm_graph[id_a][id_b];
    auto & _all_loop_set = all_loops_set_by_pair[id_a][id_b];
    auto & _all_loops = all_loops[id_a][id_b];

    TicToc tic1;

    for (size_t i = 0; i < new_loops.size(); i++) {
        auto & edge1 = new_loops[i];
        //Now only process inter-edges
        while (pcm_graph.size() < _all_loops.size() + 1) {
            pcm_graph.emplace_back(std::vector<int>(0));
        }

        auto p_edge1 = edge1.relative_pose;
        Matrix6d _cov_mat_1 = edge1.get_covariance();

        for (size_t j = 0; j < _all_loops.size(); j++) {
            auto & edge2 = _all_loops[j];
            auto _cov_mat_2 = edge2.get_covariance();;
            Matrix6d _covariance = _cov_mat_1 + _cov_mat_2;

            int same_robot_pair = edge2.same_robot_pair(edge1);
            if (same_robot_pair > 0) {
                //Now we can compute the consistency error.
                std::pair<Swarm::Pose, Matrix6d> odom_a, odom_b;
                Swarm::Pose p_edge2;
                double traj_a = 0, traj_b = 0;

                if (same_robot_pair == 1) {
                    p_edge2 = edge2.relative_pose;
                    //ODOM is tsa->tsb
                    odom_a = ego_motion_trajs.at(edge1.id_a).get_relative_pose_by_ts(edge1.ts_a, edge2.ts_a, true);
                    odom_b = ego_motion_trajs.at(edge1.id_b).get_relative_pose_by_ts(edge1.ts_b, edge2.ts_b, true);
                    if (param.debug_write_debug) {
                        traj_a = ego_motion_trajs.at(edge1.id_a).trajectory_length_by_ts(edge1.ts_a, edge2.ts_a);
                        traj_b = ego_motion_trajs.at(edge1.id_b).trajectory_length_by_ts(edge1.ts_b, edge2.ts_b);
                    }

                    _covariance += odom_a.second + odom_b.second;

                }  else if (same_robot_pair == 2) {
                    p_edge2 = edge2.relative_pose.inverse();
                    odom_a = ego_motion_trajs.at(edge1.id_a).get_relative_pose_by_ts(edge1.ts_a, edge2.ts_b, true);
                    odom_b = ego_motion_trajs.at(edge1.id_b).get_relative_pose_by_ts(edge1.ts_b, edge2.ts_a, true);
                    
                    if (param.debug_write_debug) {
                        traj_a = ego_motion_trajs.at(edge1.id_a).trajectory_length_by_ts(edge1.ts_a, edge2.ts_b);
                        traj_b = ego_motion_trajs.at(edge1.id_b).trajectory_length_by_ts(edge1.ts_b, edge2.ts_a);
                    }

                    _covariance += odom_a.second + odom_b.second;
                }

                Swarm::Pose err_pose = odom_a.first*p_edge2*odom_b.first.inverse()*p_edge1.inverse();
                auto logmap = err_pose.log_map();
                double smd = Swarm::computeSquaredMahalanobisDistance(logmap, _covariance);

                if (smd < param.pcm_thres) {
                    //Add edge i to j
                    pcm_graph[_all_loops.size()].push_back(j);
                    pcm_graph[j].push_back(_all_loops.size());
                }

                if (param.debug_write_debug) {
                    fprintf(f_logs, "\n");
                    fprintf(f_logs, "EdgePair %ld->%ld\n", edge1.id, edge2.id);
                    fprintf(f_logs, "Edge1 %ld@%d->%ld@%d DOF %d Pose %s cov_1 [%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e]\n", 
                        edge1.id_a, TSShort(edge1.ts_a), edge1.id_b, TSShort(edge1.ts_b), edge1.res_count, edge1.relative_pose.tostr().c_str(),
                        _cov_mat_1(0, 0), _cov_mat_1(1, 1), _cov_mat_1(2, 2), _cov_mat_1(3, 3), _cov_mat_1(4, 4), _cov_mat_1(5, 5));
                    fprintf(f_logs, "Edge2 %ld@%d->%ld@%d DOF %d Pose %s cov_2 [%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e]\n", 
                        edge2.id_a, TSShort(edge2.ts_a), edge2.id_b, TSShort(edge2.ts_b), edge2.res_count, edge2.relative_pose.tostr().c_str(),
                        _cov_mat_2(0, 0), _cov_mat_2(1, 1), _cov_mat_2(2, 2), _cov_mat_2(3, 3), _cov_mat_2(4, 4), _cov_mat_2(5, 5));
                        
                    auto cov = odom_a.second;
                    fprintf(f_logs, "odom_a %s traj len %.2f cov (T, Q) [%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e]\n", odom_a.first.tostr().c_str(), 
                        traj_a, cov(0, 0), cov(1, 1), cov(2, 2), cov(3, 3), cov(4, 4), cov(5, 5));
                    cov = odom_b.second;
                    fprintf(f_logs, "odom_b %s traj len %.2f cov (T, Q) [%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e]\n", odom_b.first.tostr().c_str(), 
                        traj_b, cov(0, 0), cov(1, 1), cov(2, 2), cov(3, 3), cov(4, 4), cov(5, 5));
                    fprintf(f_logs, "err_pose %s logmap [%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e]\n", err_pose.tostr().c_str(), 
                        logmap(0), logmap(1), logmap(2), logmap(3), logmap(4), logmap(5));
                    fprintf(f_logs, "squaredMahalanobisDistance %f Same Direction %d _cov(T, Q)  [%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e,%+3.1e]\n", smd, same_robot_pair == 1,
                        _covariance(0, 0),
                        _covariance(1, 1),
                        _covariance(2, 2),
                        _covariance(3, 3),
                        _covariance(4, 4),
                        _covariance(5, 5));
                }
                
                if (param.debug_write_pcm_errors) {
                    pcm_errors << edge1.id << " " << edge2.id << " "  << smd << " " << std::endl;
                }

            }
        }
        _all_loops.push_back(edge1);
        _all_loop_set.insert(edge1.id);
        all_loops_set.insert(edge1.id);
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
    ROS_INFO("[SWARM_LOCAL](OutlierRejection) %d<->%d compute_pcm_errors %.1fms maxCliqueHeu takes %.1fms inter_loop %ld good %ld", 
        id_a, id_b, compute_pcm_erros, tic.toc(), _all_loops.size(), max_clique_data.size());

    good_loops_set[id_a][id_b].clear();
    good_loops_set[id_b][id_a].clear();
    for (auto i : max_clique_data) {
        good_loops_set[id_a][id_b].insert(_all_loops[i].id);
        good_loops_set[id_b][id_a].insert(_all_loops[i].id);
    }
}