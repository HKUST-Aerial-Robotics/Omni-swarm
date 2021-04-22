#pragma once
#include <iostream>
#include <ceres/ceres.h>
#include <map>

namespace DSLAM {
class DistributedSolver {
    std::set<double*> local_poses;
    std::set<double*> remote_poses;
public:
    DistributedSolver() {}
    virtual void set_local_poses(std::vector<double*> poses);
    virtual void set_remote_poses(std::vector<double*> poses);
    virtual void add_residual(ceres::CostFunction * cost_function, bool is_huber_norm = false) = 0;
    virtual void solve() = 0;
};

class ADMMDistributed : public DistributedSolver {
public:
    ADMMDistributed() {}
    virtual void set_local_poses(std::vector<double*> poses);
    virtual void set_remote_poses(std::vector<double*> poses);
    virtual void add_residual(ceres::CostFunction * cost_function, bool is_huber_norm = false);
    virtual void solve();
};
}