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
    ceres::internal::Evaluator * evaluator = nullptr;
    ceres::internal::Program * reduced_program = nullptr;
    std::vector<double*> removed_parameter_blocks;
    double fixed_cost;
    std::vector<double> reduced_parameters;
    int num_residuals_;
    int num_parameters_;
    ceres::internal::ProblemImpl* problem_impl;

    //These values are at linearized point!
    ceres::Vector x, residual, gradient;
    
    ceres::internal::SparseMatrix * jacobian = nullptr;
    ceres::Matrix J, Jt;
    ceres::Matrix H,g;
public:
    DistributedSolver();
    virtual void set_local_poses(std::vector<double*> poses);
    virtual void set_remote_poses(std::vector<double*> poses);

    virtual void update_remote_poses(std::vector<double*> poses) {};
    
    virtual void set_fixed_poses(std::vector<double*> poses);
    virtual void add_residual(ceres::CostFunction * cost_function, std::vector<double*> poses, bool is_huber_norm = false);
    virtual void linearization() {};
    virtual void setup();
    virtual double iteration(bool linearization = true) {};

    virtual std::vector<double*> get_last_local_states() {};

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
    ceres::Vector delta_last;
    ceres::Vector x_last;
public:
    DGSSolver();
    ~DGSSolver() {};
    virtual void linearization();
    virtual double iteration(bool linearization = true) override;
    virtual void update_remote_poses(std::vector<double*> poses) override;
    virtual std::vector<double*> get_last_local_states() override;
};

}