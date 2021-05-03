#include <distributed_solver/distributed_solver.hpp>
#include <swarm_localization/localiztion_costfunction.hpp>
#include <random>
#include <boost/program_options.hpp>
#include <swarm_msgs/Pose.h>

namespace po = boost::program_options;

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}

using namespace DSLAM;
float LOOP_POS_STD_0 = 0.5;
float LOOP_POS_STD_SLOPE = 0.0;

float LOOP_YAW_STD_0 = 0.05;
float LOOP_YAW_STD_SLOPE = 0.0;

std::random_device rd{};
// std::mt19937 eng{rd()};
std::default_random_engine eng{0};

// values near the mean are the most likely
// standard deviation affects the dispersion of generated values from the mean
std::normal_distribution<double> d{0,1};


CostFunction * setup_test_loop(int ida, int idb, int ta, int tb, Eigen::Vector4d posea, Eigen::Vector4d poseb,
    double loop_pos_std = 0, double loop_yaw_std = 0) {
    Swarm::LoopConnection * loop = new Swarm::LoopConnection;
    loop->id_a = ida;
    loop->id_b = idb;
    loop->ts_a = ta;
    loop->ts_b = tb;
    loop->avg_count = 1;
    Eigen::Vector3d dpos(
        poseb(0) - posea(0) + d(eng)*loop_pos_std, 
        poseb(1) - posea(1) + d(eng)*loop_pos_std,
        poseb(2) - posea(2) + d(eng)*loop_pos_std);

    loop->relative_pose = Swarm::Pose(
        dpos, poseb(3) - posea(3) + d(eng)*loop_yaw_std
    );

    auto sle = new SwarmLoopError(loop);
    auto cost_function = new LoopCost(sle);
    cost_function->AddParameterBlock(4);
    cost_function->AddParameterBlock(4);
    cost_function->SetNumResiduals(sle->residual_count());

    return cost_function;
}

struct AgentState {
    std::vector<double *> local_poses;
    std::vector<double *> neighbor_poses;
    std::vector<std::tuple<ceres::CostFunction*, double*, double*>> related_factors;

    //Pose agentid/poseid
    std::vector<std::pair<int, int>> neighbor_poses_index;
};

struct PoseGraphGenerationParam{
    int agents_num = 0;
    int keyframe_per_agents_num = 0;
    enum {
        NO_NOISE,
        NOISE_LOCAL,
        NOISE_DRIFT
    } noise_type;
    
    enum {
        GRID_POSE_GRAPH
    }
    type; //0 Grid

    double POS_NOISE_STD = 0;
    double YAW_NOISE_STD = 0;

    double LOOP_POS_NOISE_STD = 0;
    double LOOP_YAW_NOISE_STD = 0;

    double pose_x_step = 1;
    double pose_y_step = 1;
};

void grid_poses_generation(
    std::vector<std::vector<Eigen::Vector4d>> & poses_gt,
    std::vector<std::vector<Eigen::Vector4d>> & poses,
    std::vector<AgentState> & states,
    std::vector<std::tuple<ceres::CostFunction*, double*, double*>> & factors,
    PoseGraphGenerationParam param
) {
    //Construct a grid sample  
    /*
    Pit i is id, t is stamp

    -------------width,y-----------
    | P00 P10 P20 P30 P40 P50 ... poses[0]
    l P01 P11 P21 P31 P41 P51 ... poses[1]
    x P02 P12 P22 P32 P42 P52 ... poses[2]
    | ...
    */

    int pose_grid_width = param.agents_num;
    int pose_grid_length = param.keyframe_per_agents_num;
    double pose_x_step = param.pose_x_step;
    double pose_y_step = param.pose_y_step;

    states.resize(pose_grid_width);

    for (int t = 0; t < pose_grid_length; t ++) {
        poses_gt.push_back(std::vector<Eigen::Vector4d>(pose_grid_width));
        poses.push_back(std::vector<Eigen::Vector4d>(pose_grid_width));
        for (int i = 0; i < pose_grid_width; i++)  {
            states[i].local_poses.push_back(poses[t][i].data());

            //Initial without noise
            poses_gt[t][i](0) = t*pose_x_step; //x
            poses_gt[t][i](1) = i*pose_y_step; //y
            poses_gt[t][i](2) = 0; //z
            poses_gt[t][i](3) = 0; //yaw

            memcpy(poses[t][i].data(), poses_gt[t][i].data(), sizeof(double) * 4);

            if (i > 0) {
                //Setup relative pose residual between agents
                auto cost_function = setup_test_loop(i-1, i, t, t, poses[t][i-1], poses[t][i], param.LOOP_POS_NOISE_STD, param.LOOP_YAW_NOISE_STD);
                factors.push_back(std::make_tuple(
                    cost_function, poses[t][i-1].data(), poses[t][i].data()
                ));

                states[i].neighbor_poses.push_back(poses[t][i-1].data());
                states[i].neighbor_poses_index.push_back(std::make_pair(i-1, t));

                states[i-1].neighbor_poses.push_back(poses[t][i].data());
                states[i-1].neighbor_poses_index.push_back(std::make_pair(i, t));
                
                states[i].related_factors.push_back(std::make_tuple(
                        cost_function, poses[t][i-1].data(), poses[t][i].data()
                ));
                states[i-1].related_factors.push_back(std::make_tuple(
                        cost_function, poses[t][i-1].data(), poses[t][i].data()
                ));
            }

            if (t > 0) {
                //Setup relative pose residual same agent different time
                auto cost_function = setup_test_loop(i, i, t-1, t, poses[t-1][i], poses[t][i], param.LOOP_POS_NOISE_STD, param.LOOP_YAW_NOISE_STD);
                factors.push_back(std::make_tuple(
                    cost_function, poses[t-1][i].data(), poses[t][i].data()
                ));

                states[i].related_factors.push_back(std::make_tuple(
                    cost_function, poses[t-1][i].data(), poses[t][i].data()
                ));
            }

        }
    }



    if (param.noise_type == PoseGraphGenerationParam::NOISE_LOCAL) {
        for (auto & _poses: poses) {
            for (auto & _pose : _poses) {
                _pose(0) += d(eng)*param.POS_NOISE_STD;
                _pose(1) += d(eng)*param.POS_NOISE_STD;
                _pose(2) += d(eng)*param.POS_NOISE_STD;
                _pose(3) += d(eng)*param.YAW_NOISE_STD;
            }
        }
    } else if (param.noise_type == PoseGraphGenerationParam::NOISE_DRIFT) {
        auto _poses = poses[0];
        auto _poses_gt = poses_gt[0];
        std::map<int, double> pos_drift_constant;
        std::map<int, double> yaw_drift_constant;
        for (unsigned int i = 0; i < pose_grid_width; i++) {
            pos_drift_constant[i] = 0;//d(eng)*param.POS_NOISE_STD*param.pose_y_step/10;
            yaw_drift_constant[i] = 0;//d(eng)*param.YAW_NOISE_STD*param.pose_y_step/10;
        }

        for (unsigned int i = 1; i < pose_grid_width; i++) {
            _poses[i](0) = d(eng)*param.POS_NOISE_STD*param.pose_x_step + _poses_gt[i](0) - _poses_gt[i-1](0) + _poses[i-1](0);
            _poses[i](1) = d(eng)*param.POS_NOISE_STD*param.pose_x_step + _poses_gt[i](1) - _poses_gt[i-1](1) + _poses[i-1](1);
            _poses[i](2) = d(eng)*param.POS_NOISE_STD*param.pose_x_step + _poses_gt[i](2) - _poses_gt[i-1](2) + _poses[i-1](2);
            _poses[i](3) = d(eng)*param.YAW_NOISE_STD*param.pose_x_step + _poses_gt[i](3) - _poses_gt[i-1](3) + _poses[i-1](3);
        }

        for (unsigned int t = 1; t < pose_grid_length; t++) {
            for (unsigned int i = 0; i < pose_grid_width; i++) {
                Swarm::Pose gt0(poses_gt[t-1][i].data(), true);
                Swarm::Pose gt(poses_gt[t][i].data(), true);
                Swarm::Pose pose0(poses[t-1][i].data(), true);
                Swarm::Pose relative_pose = Swarm::Pose::DeltaPose(gt0, gt, true);
                relative_pose.pos().x() += d(eng)*param.POS_NOISE_STD + pos_drift_constant[i];
                relative_pose.pos().y() += d(eng)*param.POS_NOISE_STD + pos_drift_constant[i];
                relative_pose.pos().z() += d(eng)*param.POS_NOISE_STD + pos_drift_constant[i];
                relative_pose.yaw() += d(eng)*param.YAW_NOISE_STD + yaw_drift_constant[i];
                relative_pose.update_attitude();

                Swarm::Pose pose = pose0*relative_pose;
                pose.to_vector_xyzyaw(poses[t][i].data());
            }
        }
    }
}


//Solver type 0 centralized
//Solver type 1 DGS
void PoseGraphTest(PoseGraphGenerationParam param, int solver_type, int iteration_max, int linearization_step, bool output_coor, double tolerance = 0.01) {
    std::vector<std::vector<Eigen::Vector4d>> poses;
    std::vector<std::vector<Eigen::Vector4d>> poses_gt;
    std::vector<AgentState> states;
    std::vector<std::tuple<ceres::CostFunction*, double*, double*>> factors;
    int keyframe_per_agents_num = param.keyframe_per_agents_num;
    int agents_num = param.agents_num;
    double iter_time = 0;
    double iter_cost = 0;
    double iter_cost_last = 0;
    double eta = 0;
    int factor_count = 0;
    
    std::string _solver_type = "DGS";
    if (solver_type == 0) {
        _solver_type = "Centrialized";
    }
    
    grid_poses_generation(poses_gt, poses, states, factors, param);

    printf("GridPoseGraphTest: Solver:%s Pose graph with agents %d time %d step %3.1f/%3.1f total poses %d factors %ld\n",
        _solver_type.c_str(),
        agents_num,
        keyframe_per_agents_num,
        param.pose_x_step,
        param.pose_y_step,
        agents_num * keyframe_per_agents_num,
        factors.size()
    );

    //poses_tmp for simulate inter-agent communication
    std::vector<std::vector<Eigen::Vector4d>> poses_tmp;
    for (int t = 0; t < keyframe_per_agents_num; t ++) {
        poses_tmp.push_back(std::vector<Eigen::Vector4d>(agents_num));
        for (int i = 0; i < agents_num; i++)  {
            poses_tmp[t][i] = poses[t][i];
        }
    }

    if (output_coor) {
        printf("Initial states:\n");
        for (unsigned int t = 0; t < keyframe_per_agents_num; t ++) {
            for (unsigned int i = 0; i < agents_num; i ++) {
                printf("(%3.2f,%3.2f,%3.2f)%3.1f\t",poses[t][i][0], poses[t][i][1], poses[t][i][2], poses[t][i][3]*57.3);
            }
            printf("\n");
        }
    }

    if (solver_type == 0) {
        CentrializedSolver solver;
        for (unsigned int i = 0; i < states.size(); i ++) {
            auto &local_poses = states[i].local_poses;
            auto &neighbor_poses = states[i].neighbor_poses;
            auto &neighbor_factors = states[i].related_factors;
            solver.set_local_poses(local_poses);

            if (i == 0) {
                solver.set_pose_fixed(local_poses[0]);
            }

            printf("Centralized Add Agent: %d, local poses %ld, neightbor poses %ld residuals %ld\n",
                i,
                local_poses.size(),
                neighbor_poses.size(),
                neighbor_factors.size()
            );
        }

        for (auto cost : factors) {
            std::vector<double*> _poses(2);
            _poses[0] = std::get<1>(cost);
            _poses[1] = std::get<2>(cost);
            solver.add_residual(std::get<0>(cost), _poses);
        }
        iter_cost = solver.solve(tolerance);
        factor_count = factors.size();
        iter_time = solver.get_total_iteration_time();
    } else {
        std::vector<DGSSolver> solvers(agents_num);
        for (unsigned int i = 0; i < solvers.size(); i ++) {
            auto &solver = solvers[i];
            auto &local_poses = states[i].local_poses;
            auto &neighbor_poses = states[i].neighbor_poses;
            auto &neighbor_factors = states[i].related_factors;

            solver.set_local_poses(local_poses);
            solver.set_remote_poses(neighbor_poses);
            if (i == 0) {
                solver.set_pose_fixed(local_poses[0]);
            }

            for (auto cost : neighbor_factors) {
                std::vector<double*> _poses(2);
                _poses[0] = std::get<1>(cost);
                _poses[1] = std::get<2>(cost);
                solver.add_residual(std::get<0>(cost), _poses);
            }

            printf("DGSSolver: %d, local poses %ld, neightbor poses %ld residuals %ld\n",
                i,
                local_poses.size(),
                neighbor_poses.size(),
                neighbor_factors.size()
            );
        }
        
        for (unsigned int iter = 0; iter < iteration_max; iter++) {
            printf("iter %d:", iter);
            bool need_linearization = (iter %linearization_step == 0);
            
            if (need_linearization && iter > 0) {
                //Sync pose_tmps to poses
                printf("Sync poses, relinearization... ");
                if (output_coor) {
                    printf("\n");
                }
                for (unsigned int t = 0; t < keyframe_per_agents_num; t ++) {
                    for (unsigned int i = 0; i < solvers.size(); i ++) {
                        memcpy(poses[t][i].data(), poses_tmp[t][i].data(), sizeof(double) * 4);
                        if (output_coor) {
                            printf("(%3.2f,%3.2f,%3.2f)%3.3f\t",poses[t][i](0), poses[t][i](1), poses[t][i](2), poses[t][i](3)*57.3);
                        }
                    }
                    if (output_coor) {
                        printf("\n");
                    }
                }
            }

            iter_cost = 0;
            factor_count = 0;

            for (unsigned int i = 0; i < agents_num; i ++) {
                auto &solver = solvers[i];
                solver.setup();
                iter_cost += solver.cost();
                factor_count += states[i].related_factors.size();
            }

            printf("start cost: %.1e(%.1e) ", iter_cost/factor_count, iter_cost);
            iter_cost_last = iter_cost;
            iter_cost = 0;
            factor_count = 0;

            // #pragma omp parallel for num_threads(12)
            for (unsigned int i = 0; i < agents_num; i ++) {
                auto &solver = solvers[i];

                //Update to last states
                std::vector<double*> _neighbor_poses;
                for(auto _index : states[i].neighbor_poses_index) {
                    _neighbor_poses.push_back(poses_tmp[_index.second][_index.first].data());
                }

                if (iter > 0 ) {
                    solver.update_remote_poses(_neighbor_poses);
                }

                //Perform iteration
                iter_cost += solver.iteration(need_linearization);
                factor_count += states[i].related_factors.size();

                //Update state to neighbors
                auto last_poses = solver.get_last_local_states();
                for (unsigned int t = 0; t < keyframe_per_agents_num; t++) {
                    memcpy(poses_tmp[t][i].data(), last_poses[t].data(), sizeof(double) * 4);
                }
            }

            eta = fabs(iter_cost - iter_cost_last) / iter_cost;
            // std::cout << iter_cost << "," <<  iter_cost_last << "," << eta << std::endl;

            printf("end cost: %.1e(%.1e) tolerance %.1e factors %d\n", iter_cost/factor_count, iter_cost, eta, factor_count);
            if (eta < tolerance) {
                printf("convergence: cost:%.1e at iteration %d...\n", iter_cost/factor_count, iter);
                break;
            }
        }
        for (unsigned int i = 0; i < agents_num; i ++) {
            iter_time += solvers[i].get_total_iteration_time();
        }
    }

    if(output_coor) {
        printf("final states:\n");
        for (unsigned int t = 0; t < keyframe_per_agents_num; t ++) {
            for (unsigned int i = 0; i < agents_num; i ++) {
                printf("(%3.2f,%3.2f,%3.2f),%3.3f\t",poses[t][i][0], poses[t][i][1], poses[t][i][2], poses[t][i][3]*57.3);
            }
            printf("\n");
        }
    }

    printf("total elapse time %.1ems time per agents %3.4fms final cost: %.1e(total %1.e)\n", iter_time, iter_time/agents_num, iter_cost/factor_count, iter_cost);
    std::cout << "GridPoseGraphTest Finish" << std::endl;
    return;
}

int main(int argc, char *argv[]) {
    int pose_grid_width = 100;
    int pose_grid_length = 100;
    int iteration_max = 100;
    int linearization_step = 2;
    bool output_coor = false;

    namespace po = boost::program_options;
    po::options_description desc("Allowed options");

    desc.add_options()
        ("help", "produce help message")
        ("agents,a", po::value<int>()->default_value(1), "number of simulated agents")
        ("keyframes,k", po::value<int>()->default_value(3), "number of keyframe per agents")
        ("maxiter,i", po::value<int>()->default_value(100), "number of max iterations")
        ("linearstep,l", po::value<int>()->default_value(2), "linearization per step")
        ("cost,c", po::value<double>()->default_value(0.01), "accept cost tolerance")
        ("solver,s", po::value<int>()->default_value(1), "solver type 0 for centrialized 2 for DGS")
        ("output-coor,v", "if output coordinate")
        ("loop-noise", "if noise on loop")
        ("no_initial_noise", "no initial pose noise")
        ("initial_drift_noise,d", "drift initial noise")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    PoseGraphGenerationParam param;
    param.agents_num = vm["agents"].as<int>();
    param.keyframe_per_agents_num = vm["keyframes"].as<int>();
    param.noise_type = PoseGraphGenerationParam::NOISE_LOCAL;
    param.POS_NOISE_STD = 1.0;
    param.YAW_NOISE_STD = 0.1;
    param.LOOP_POS_NOISE_STD = 0;
    param.LOOP_YAW_NOISE_STD = 0;
    param.pose_x_step = 1.0;
    param.pose_y_step = 1.0;
    
    if (vm.count("loop-noise")) {
        param.LOOP_POS_NOISE_STD = 0.2;
        param.LOOP_YAW_NOISE_STD = 0.05;
    }

    if (vm.count("no_initial_noise")) {
        param.noise_type = PoseGraphGenerationParam::NO_NOISE;
        param.POS_NOISE_STD = 0.0;
        param.YAW_NOISE_STD = 0.0;
    }

    if (vm.count("initial_drift_noise")) {
        param.noise_type = PoseGraphGenerationParam::NOISE_DRIFT;
        param.POS_NOISE_STD = 0.0109;
        param.YAW_NOISE_STD = 0.0033/180*M_PI;
        // param.YAW_NOISE_STD = 0.01/180*M_PI;
    }

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    PoseGraphTest(param, vm["solver"].as<int>(), vm["maxiter"].as<int>(), vm["linearstep"].as<int>(), vm.count("output-coor"),
        vm["cost"].as<double>());
    return 0;
}