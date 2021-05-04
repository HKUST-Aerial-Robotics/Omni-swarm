#pragma once
#include <iostream>
#include <map>
#include <chrono>
#include <ctime>
#include <ceres/ceres.h>
#include "base_solver.hpp"
namespace DSLAM {
class CentralizedSolver : public BaseSolver{
protected:
ceres::Problem problem;
public:
    void add_residual(ceres::CostFunction * cost_function, std::vector<double*> poses, bool is_huber_norm=false) override;
    void set_local_poses(std::vector<double*> poses) override;
    void set_pose_fixed(double* pose);
    void set_poses_fixed(std::vector<double*> poses);
    double solve(double tolerance=0.01);
};
}