#include <swarm_localization/swarm_outlier_rejection.hpp>
#include <fstream>
#include "third_party/fast_max-clique_finder/src/graphIO.h"
#include "third_party/fast_max-clique_finder/src/findClique.h"

// #define PCM_DEBUG_OUTPUT

std::fstream pcm_good;
std::fstream pcm_errors;

std::vector<Swarm::LoopEdge> SwarmLocalOutlierRejection::OutlierRejectionLoopEdges(const std::vector<Swarm::LoopEdge> & available_loops) {
    if (param.debug_write_pcm_good) {
        pcm_good.open("/root/output/pcm_good.txt", std::ios::out);
    }

    if (param.debug_write_pcm_errors) {
        pcm_errors.open("/root/output/pcm_errors.txt", std::ios::out);
    }

    std::map<int, std::map<int, std::vector<Swarm::LoopEdge>>> inter_loops;
    std::map<int, std::vector<Swarm::LoopEdge >> intra_loops;
    std::vector<Swarm::LoopEdge> good_loops;
    for (auto & edge: available_loops) {
        if (edge.is_inter_loop()) {
            inter_loops[edge.id_a][edge.id_b].emplace_back(edge);
            inter_loops[edge.id_b][edge.id_b].emplace_back(edge);
        } else {
            intra_loops[edge.id_a].emplace_back(edge);
        }
    }

    for (auto it : intra_loops) {
        ROS_INFO("[SWARM_LOCAL](OutlierRejection) Intra-LCM drone %d", it.first);
        good_loops = OutlierRejectionLoopEdgesPCM(it.second);
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

    if (param.debug_write_pcm_good) {
        pcm_errors.close();
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
        Matrix<double, 6, 1> _cov_vec_1 = edge1.get_cov_vec();

        for (size_t j = 0; j < i; j ++) {
            //Now only process inter-edges
            auto & edge2 = available_loops[j];
            Matrix<double, 6, 1> _cov_vec = _cov_vec_1 + edge2.get_cov_vec();
            Matrix<double, 6, 1> _cov_vec_odom_a;
            Matrix<double, 6, 1> _cov_vec_odom_b;

            int same_robot_pair = edge2.same_robot_pair(edge1);
            if (same_robot_pair > 0) {
                //Now we can compute the consistency error.
                std::pair<Swarm::Pose, Eigen::Matrix<double, 6, 1>> odom_a, odom_b;
                Swarm::Pose p_edge2;

                if (same_robot_pair == 1) {
                    p_edge2 = edge2.relative_pose;
                    //ODOM is tsa->tsb
                    odom_a = ego_motion_trajs.at(edge1.id_a).get_relative_pose_by_ts(edge1.ts_a, edge2.ts_a);
                    odom_b = ego_motion_trajs.at(edge1.id_b).get_relative_pose_by_ts(edge1.ts_b, edge2.ts_b);

                    _cov_vec += odom_a.second + odom_b.second;

                }  else if (same_robot_pair == 2) {
                    p_edge2 = edge2.relative_pose.inverse();
                    odom_a = ego_motion_trajs.at(edge1.id_a).get_relative_pose_by_ts(edge1.ts_a, edge2.ts_b);
                    odom_b = ego_motion_trajs.at(edge1.id_b).get_relative_pose_by_ts(edge1.ts_b, edge2.ts_a);

                    _cov_vec_odom_a = ego_motion_trajs.at(edge1.id_a).covariance_between_ts(edge1.ts_a, edge2.ts_b);
                    _cov_vec_odom_b = ego_motion_trajs.at(edge1.id_b).covariance_between_ts(edge1.ts_b, edge2.ts_a);
                    _cov_vec += odom_a.second + odom_b.second;
                }

                Swarm::Pose err_pose = odom_a.first*p_edge2*odom_b.first.inverse()*p_edge1.inverse();
                auto logmap = err_pose.log_map();
                double smd = Swarm::computeSquaredMahalanobisDistance(logmap, _cov_vec);

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
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) odom_a %s cov T [%+3.1e] YPR [%+3.1e]", odom_a.tostr().c_str(), _cov_vec_odom_a(0), _cov_vec_odom_a(3));
                ROS_INFO("[SWARM_LOCAL](OutlierRejection) odom_b %s cov T [%+3.1e] YPR [%+3.1e]", odom_b.tostr().c_str(), _cov_vec_odom_b(0), _cov_vec_odom_b(3));
                printf("[SWARM_LOCAL](OutlierRejection) err_pose %s logmap", err_pose.tostr().c_str());
                // std::cout << logmap.transpose() << std::endl;
                printf("[SWARM_LOCAL](OutlierRejection) squaredMahalanobisDistance %f Same Direction %d _cov_vec %.3e\n", smd, same_robot_pair == 1, _cov_vec.norm());
#endif
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

    return good_loops;
}