#include <distributed_solver/distributed_solver.hpp>
#include <swarm_localization/localiztion_costfunction.hpp>
#define BACKWARD_HAS_DW 1
#include <backward.hpp>
#include <random>


namespace backward
{
    backward::SignalHandling sh;
}


using namespace DSLAM;
float LOOP_POS_STD_0 = 0.5;
float LOOP_POS_STD_SLOPE = 0.0;

float LOOP_YAW_STD_0 = 0.5;
float LOOP_YAW_STD_SLOPE = 0.0;

float POS_INITAL_NOISE_STD = 0.1;
float YAW_INITAL_NOISE_STD = 0.0;

// std::random_device rd{};
//std::mt19937 gen{rd()};
std::default_random_engine eng{0};


// values near the mean are the most likely
// standard deviation affects the dispersion of generated values from the mean
std::normal_distribution<double> d{0,1};


CostFunction * setup_test_loop(int ida, int idb, int ta, int tb, double * posea, double * poseb) {
    Swarm::LoopConnection * loop = new Swarm::LoopConnection;
    loop->id_a = ida;
    loop->id_b = idb;
    loop->ts_a = ta;
    loop->ts_b = tb;
    loop->avg_count = 1;
    Eigen::Vector3d dpos(poseb[0] -posea[0], 
        poseb[1] - posea[1],
        poseb[2] - posea[2]);

    loop->relative_pose = Swarm::Pose(
        dpos, poseb[3] - posea[3]
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
    std::vector<std::tuple<ceres::CostFunction*, double*, double*>> related_costs;
};

void DGSTest() {

    //Construct a grid sample  
    std::vector<std::tuple<ceres::CostFunction*, double*, double*>> costs;
    std::vector<AgentState> states(10);

    std::vector<std::vector<double*>> poses;
    std::vector<Swarm::LoopConnection*> loops;
    int pose_grid_width = 10;
    int pose_grid_length = 10;
    double pose_x_step = 1;
    double pose_y_step = 1;

    states.resize(pose_grid_width);
    /*
    Pit i is id, t is stamp

    -------------width,y-----------
    | P00 P10 P20 P30 P40 P50 ... poses[0]
    l P01 P11 P21 P31 P41 P51 ... poses[1]
    x P02 P12 P22 P32 P42 P52 ... poses[2]
    | ...
    */

    printf("Setup poses with agents %d time %d step %f/%f\n",
        pose_grid_width,
        pose_grid_length,
        pose_x_step,
        pose_y_step
    );

    for (int t = 0; t < pose_grid_length; t ++) {
        poses.push_back(std::vector<double*>(pose_grid_width));
        for (int i = 0; i < pose_grid_width; i++)  {
            poses[t][i] = new double[4];

            states[i].local_poses.push_back(poses[t][i]);

            //Initial without noise
            poses[t][i][0] = t*pose_x_step; //x
            poses[t][i][1] = i*pose_y_step; //y
            poses[t][i][2] = 0; //z
            poses[t][i][3] = 0; //yaw

            if (i > 0) {
                //Setup relative pose residual between agents
                auto cost_function = setup_test_loop(i-1, i, t, t, poses[t][i-1], poses[t][i]);
                costs.push_back(std::make_tuple(
                    cost_function, poses[t][i-1], poses[t][i]
                ));

                states[i].neighbor_poses.push_back(poses[t][i-1]);
                states[i].related_costs.push_back(std::make_tuple(
                        cost_function, poses[t][i-1], poses[t][i]
                    ));
            }

            if (t > 0) {
                //Setup relative pose residual same agent different time
                auto cost_function = setup_test_loop(i, i, t-1, t, poses[t-1][i], poses[t][i]);
                costs.push_back(std::make_tuple(
                    cost_function, poses[t-1][i], poses[t][i]
                ));

                states[i].related_costs.push_back(std::make_tuple(
                    cost_function, poses[t-1][i], poses[t][i]
                ));
            }

        }
    }

    for (auto & _poses: poses) {
        for (auto & _pose : _poses) {
            _pose[0] += d(eng)*POS_INITAL_NOISE_STD;
            _pose[1] += d(eng)*POS_INITAL_NOISE_STD;
            _pose[2] += d(eng)*POS_INITAL_NOISE_STD;
            _pose[3] += d(eng)*YAW_INITAL_NOISE_STD;
        }
    }


    std::vector<DGSSolver> solvers(pose_grid_width);
    for (unsigned int i = 0; i < solvers.size(); i ++) {
        auto &solver = solvers[i];
        auto &local_poses = states[i].local_poses;
        auto &neighbor_poses = states[i].neighbor_poses;
        auto &neighbor_costs = states[i].related_costs;

        solver.set_local_poses(local_poses);
        solver.set_remote_poses(neighbor_poses);

        for (auto cost : neighbor_costs) {
            std::vector<double*> _poses(2);
            _poses[0] = std::get<1>(cost);
            _poses[1] = std::get<2>(cost);
            solver.add_residual(std::get<0>(cost), _poses);
        }

        printf("DGSSolver: %d, local poses %ld, neightbor poses %ld residuals %ld\n",
            i,
            local_poses.size(),
            neighbor_poses.size(),
            neighbor_costs.size()
        );
    }

    for (unsigned int i = 0; i < solvers.size(); i ++) {
        auto &solver = solvers[i];
        printf("Agent : %d\n");
        solver.iteration();
    }
}

int main() {
    printf("Hello,world\n");
    DGSTest();
}