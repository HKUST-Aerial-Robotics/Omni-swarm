#include <distributed_solver/distributed_solver.hpp>
#include <swarm_localization/localiztion_costfunction.hpp>
#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}


using namespace DSLAM;
float LOOP_POS_STD_0 = 0.5;
float LOOP_POS_STD_SLOPE = 0.0;

float LOOP_YAW_STD_0 = 0.5;
float LOOP_YAW_STD_SLOPE = 0.0;

void DGSTest() {

    //Construct a grid sample  
    std::vector<double *> local_poses;
    std::vector<double *> neighbor_poses;
    std::vector<std::tuple<ceres::CostFunction*, double*, double*>> costs;
    std::vector<std::tuple<ceres::CostFunction*, double*, double*>> neighbor_costs;

    std::vector<std::vector<double*>> poses;
    std::vector<Swarm::LoopConnection*> loops;
    int pose_grid_width = 10;
    int pose_grid_length = 10;
    double pose_x_step = 1;
    double pose_y_step = 1;

    /*
    Pit i is id, t is stamp

    -------------width,y-----------
    | P00 P10 P20 P30 P40 P50 ... poses[0]
    l P01 P11 P21 P31 P41 P51 ... poses[1]
    x P02 P12 P22 P32 P42 P52 ... poses[2]
    | ...
    */

    int main_id = 1;

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
            if (main_id == i) {
                local_poses.push_back(poses[t][main_id]);
            }

            //Initial without noise

            poses[t][i][0] = t*pose_x_step; //x
            poses[t][i][1] = i*pose_y_step; //y
            poses[t][i][2] = 0; //z
            poses[t][i][3] = 0; //yaw

            if (i > 0) {
                Swarm::LoopConnection * loop = new Swarm::LoopConnection;
                loop->id_a = i - 1;
                loop->id_b = i;
                loop->ts_a = t;
                loop->ts_b = t;
                loop->avg_count = 1;
                Eigen::Vector3d dpos(poses[t][i][0] - poses[t][i-1][0], 
                    poses[t][i][1] - poses[t][i-1][1],
                    poses[t][i][2] - poses[t][i-1][2]);

                loop->relative_pose = Swarm::Pose(
                    dpos, poses[t][i][3] - poses[t][i-1][3]
                );

                auto sle = new SwarmLoopError(loop);
                auto cost_function = new LoopCost(sle);

                costs.push_back(std::make_tuple(
                    cost_function, poses[t][i-1], poses[t][i]
                ));

                if(i == main_id) {
                    neighbor_poses.push_back(poses[t][i-1]);
                    neighbor_costs.push_back(std::make_tuple(
                    cost_function, poses[t][i-1], poses[t][i]
                ));
                }

                loops.push_back(loop);
            }

            if (t > 0) {
                Swarm::LoopConnection * loop = new Swarm::LoopConnection;
                loop->id_a = i;
                loop->id_b = i;
                loop->ts_a = t-1;
                loop->ts_b = t;
                loop->avg_count = 1;
                Eigen::Vector3d dpos(poses[t][i][0] - poses[t-1][i][0], 
                    poses[t][i][1] - poses[t-1][i][1],
                    poses[t][i][2] - poses[t-1][i][2]);

                loop->relative_pose = Swarm::Pose(
                    dpos, poses[t][i][3] - poses[t-1][i][3]
                );

                auto sle = new SwarmLoopError(loop);
                auto cost_function = new LoopCost(sle);

                costs.push_back(std::make_tuple(
                    cost_function, poses[t-1][i], poses[t][i]
                ));

                if(i == main_id) {
                    neighbor_poses.push_back(poses[t-1][i]);
                    neighbor_costs.push_back(std::make_tuple(
                        cost_function, poses[t-1][i], poses[t][i]
                    ));
                }

                loops.push_back(loop);
            }

        }
    }

    printf("Testing DGSSolver, local poses %ld, neightbor poses %ld residuals %ld\n",
        local_poses.size(),
        neighbor_poses.size(),
        neighbor_costs.size()
    );

    DGSSolver solver;
    solver.set_local_poses(local_poses);
    solver.set_remote_poses(neighbor_poses);

    for (auto cost : neighbor_costs) {
        std::vector<double*> _poses(2);
        _poses[0] = std::get<1>(cost);
        _poses[1] = std::get<2>(cost);
        solver.add_residual(std::get<0>(cost), _poses);
    }

    solver.solve();
}

int main() {
    printf("Hello,world\n");
    DGSTest();
}