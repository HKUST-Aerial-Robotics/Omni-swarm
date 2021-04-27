#pragma once
#include <iostream>
#include "ceres/program.h"
#include <ceres/ceres.h>
#include <map>
#include "ceres/evaluator.h"


namespace DSLAM {
class DistributedSolver {
protected:
    std::set<double*> local_poses;
    std::set<double*> remote_poses;
    std::shared_ptr<ceres::internal::Evaluator> evaluator;
    std::unique_ptr<ceres::internal::Program> reduced_program;
    std::vector<double*> removed_parameter_blocks;
    double fixed_cost;
    std::vector<double> reduced_parameters;
    int num_residuals_;
    int num_parameters_;
    ceres::internal::ProblemImpl* problem_impl;

public:
    DistributedSolver();
    ~DistributedSolver() {};
    virtual void set_local_poses(std::vector<double*> poses);
    virtual void set_remote_poses(std::vector<double*> poses);
    virtual void add_residual(ceres::CostFunction * cost_function, std::vector<double*> poses, bool is_huber_norm = false);
    virtual void setup();
    virtual void solve() {};

    virtual double get_x_jacobian_residual(
        ceres::Vector & x_,
        ceres::Vector & residual,
        ceres::Vector & gradient,
        ceres::internal::SparseMatrix * & jacobian_
    );
};

// class ADMMDistributed : public DistributedSolver {
// public:
//     ADMMDistributed() {}
//     virtual void set_local_poses(std::vector<double*> poses);
//     virtual void set_remote_poses(std::vector<double*> poses);
//     virtual void add_residual(ceres::CostFunction * cost_function, bool is_huber_norm = false);
//     virtual void solve();
// };

class DGSSolver: public  DistributedSolver{
public:
    DGSSolver();
    ~DGSSolver() {};
    virtual void solve() override;
};

}