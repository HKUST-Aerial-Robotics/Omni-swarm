#include <swarm_localization/swarm_outlier_rejection.hpp>
#include <fstream>
#include "third_party/fast_max-clique_finder/src/graphIO.h"
#include "third_party/fast_max-clique_finder/src/findClique.h"

// #define PCM_DEBUG_OUTPUT

double computeSquaredMahalanobisDistance(Matrix<double, 6, 1> logmap, Matrix<double, 6, 1> cov_vec) {
    Matrix<double, 6, 1>  inf_vec = cov_vec.cwiseInverse();
    auto inf_mat = cov_vec.asDiagonal();
    Eigen::Matrix<double, 1, 1> ret = logmap.transpose() * inf_mat * logmap;
    return std::sqrt(ret(0, 0));
}


std::vector<Swarm::LoopEdge> SwarmLocalOutlierRejection::OutlierRejectionIntraLoopEdges(const std::vector<Swarm::LoopEdge> & intra_loops) {
    return intra_loops;
}

std::vector<Swarm::LoopEdge> SwarmLocalOutlierRejection::OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops) {
    std::vector<Swarm::LoopEdge> inter_loops;
    std::vector<Swarm::LoopEdge> intra_loops;
    for (auto & edge: available_loops) {
        if (edge.is_inter_loop()) {
            inter_loops.emplace_back(edge);
        } else {
            intra_loops.emplace_back(edge);
        }
    }

    auto good_loops = OutlierRejectionIntraLoopEdges(intra_loops);
    auto good_inter_loops = OutlierRejectionInterLoopEdges(inter_loops);

    good_loops.insert( good_loops.end(), good_inter_loops.begin(), good_inter_loops.end() );
    return good_loops;
}

std::vector<Swarm::LoopEdge> SwarmLocalOutlierRejection::OutlierRejectionInterLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops) {
    std::map<FrameIdType, int> bad_pair_count;
    std::fstream pcm_errors;
    std::fstream pcm_good;

    if (!param.enable_pcm) {
        return available_loops;
    }

    if (param.debug_write_pcm_errors) {
        pcm_errors.open("/root/output/pcm_errors.txt", std::ios::out);
    }

    if (param.debug_write_pcm_good) {
        pcm_good.open("/root/output/pcm_good.txt", std::ios::out);
    }
    
        
    std::vector<std::vector<int>> pcm_graph(available_loops.size());
    TicToc tic1;

    for (size_t i = 0; i < available_loops.size(); i++) {
        auto & edge1 = available_loops[i];
        //Now only process inter-edges
        if (!edge1.is_inter_loop()) {
            ROS_ERROR("[SWARM_LOCAL](OutlierRejection) OutlierRejectionInterLoopEdges Accept only inter loops");
            exit(-1);
        }

        auto p_edge1 = edge1.relative_pose;
        Matrix<double, 6, 1> _cov_vec_1 = edge1.get_cov_vec();

        for (size_t j = 0; j < i; j ++) {
            //Now only process inter-edges
            auto & edge2 = available_loops[j];
            Matrix<double, 6, 1> _cov_vec = _cov_vec_1 + edge2.get_cov_vec();

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
                double smd = computeSquaredMahalanobisDistance(logmap, _cov_vec);

                if (smd < param.pcm_thres) {
                    //Add edge i to j
                    pcm_graph[i].push_back(j);
                    pcm_graph[j].push_back(i);
                }

#ifdef PCM_DEBUG_OUTPUT
                printf("\n");
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) EdgePair %ld->%ld ", edge1.id, edge2.id);
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) Edge1 %ld@%d->%ld@%d %s", 
                    edge1.ts_a, edge1.id_a,
                    edge1.ts_b, edge1.id_b,
                    edge1.relative_pose.tostr().c_str()
                );
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) Edge2 %ld@%d->%ld@%d %s", 
                    edge2.ts_a, edge2.id_a,
                    edge2.ts_b, edge2.id_b,
                    edge2.relative_pose.tostr().c_str()
                );
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) odom_a %s", odom_a.tostr().c_str());
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) odom_b %s", odom_b.tostr().c_str());

                printf("[SWARM_LOCAL](OutlierRejection) squaredMahalanobisDistance %f Same Direction %d _cov_vec %f\n", smd, same_robot_pair == 1, _cov_vec.norm());
#endif
                // printf("[SWARM_LOCAL](OutlierRejection) err_pose %s logmap", err_pose.tostr().c_str());
                // std::cout << logmap.transpose() << std::endl;
                if (param.debug_write_pcm_errors) {
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

    if (param.debug_write_pcm_good) {
        pcm_errors.close();
        pcm_good.close();
    }

    return good_loops;
}