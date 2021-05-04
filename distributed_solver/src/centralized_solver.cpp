#include "distributed_solver/centralized_solver.hpp"
namespace DSLAM {
void CentralizedSolver::add_residual(ceres::CostFunction * cost_function, std::vector<double*> poses, bool is_huber_norm) {
    problem.AddResidualBlock(cost_function, nullptr, poses.data(), poses.size());
}

void CentralizedSolver::set_local_poses(std::vector<double*> poses) {
    for (auto & p : poses) {
        problem.AddParameterBlock(p, PARAM_BLOCK_SIZE);
    }
}

void CentralizedSolver::set_pose_fixed(double* pose) {
    problem.SetParameterBlockConstant(pose);
}

void CentralizedSolver::set_poses_fixed(std::vector<double*> poses) {
    for (auto & p : poses) {
        problem.SetParameterBlockConstant(p);
    }
}

double CentralizedSolver::solve(double tolerance) {
    ceres::Solver::Options options;
    //SPARSE NORMAL DOGLEG 12.5ms
    //SPARSE NORMAL 21
    //DENSE NORM DOGLEG 49.31ms
    options.max_num_iterations = 1000;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.function_tolerance = tolerance;
    options.linear_solver_type = ceres::CGNR;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    // options.linear_solver_type = ceres::LEVENBERG_MARQUARDT;

    options.num_threads = 1;
    ceres::Solver::Summary summary;
    TicToc tic_iteration;
    ceres::Solve(options, &problem, &summary);
    iteration_time += tic_iteration.toc();
    iterations ++;

    // std::cout << summary.BriefReport() << std::endl;
    std::cout << summary.FullReport() << std::endl;
    return summary.final_cost;
}
}