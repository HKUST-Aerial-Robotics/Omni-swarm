#include <distributed_solver/distributed_solver.hpp>
#include "ceres/problem_impl.h"
#include "ceres/program.h"
#include "ceres/sparse_matrix.h"
#define PARAM_BLOCK_SIZE 4

namespace DSLAM {
    DistributedSolver::DistributedSolver():
    problem_impl(new ceres::internal::ProblemImpl) {

    }
    void DistributedSolver::set_local_poses(std::vector<double*> poses) {
        local_poses.clear();
        for (auto p: poses) {
            local_poses.insert(p);
            problem_impl->AddParameterBlock(p, PARAM_BLOCK_SIZE);
        }
    }

    void DistributedSolver::set_remote_poses(std::vector<double*> poses) {
        remote_poses.clear();
        for (auto p: poses) {
            remote_poses.insert(p);
            problem_impl->AddParameterBlock(p, PARAM_BLOCK_SIZE);
            problem_impl->SetParameterBlockConstant(p);
        }
    }
    
    void DistributedSolver::add_residual(ceres::CostFunction * cost_function, std::vector<double*> poses, bool is_huber_norm) {
        ceres::LossFunction *loss_function = nullptr;
        if (is_huber_norm) {
            loss_function = new ceres::HuberLoss(1.0);
        }
        problem_impl->AddResidualBlock(cost_function, loss_function, poses.data(), poses.size());
    }

    void DistributedSolver::setup() {
        ceres::internal::Evaluator::Options evaluator_options;
        evaluator_options.linear_solver_type = ceres::CGNR;
        
        ceres::internal::Program* program = problem_impl->mutable_program();
        std::string error;
        reduced_program.reset(program->CreateReducedProgram(
            &removed_parameter_blocks, &fixed_cost, &error));

        evaluator_options.evaluation_callback =
            reduced_program->mutable_evaluation_callback();

        evaluator_options.context = problem_impl->context();
        evaluator_options.num_eliminate_blocks = 0;
        
        evaluator.reset(ceres::internal::Evaluator::Create(
            evaluator_options, reduced_program.get(), &error));

        num_parameters_ = evaluator->NumParameters();
        num_residuals_ = evaluator->NumResiduals();

        reduced_parameters.resize(program->NumParameters());
        program->ParameterBlocksToStateVector(reduced_parameters.data());
    }


    double DistributedSolver::get_x_jacobian_residual(
        ceres::Vector & x_,
        ceres::Vector & residual,
        ceres::Vector & gradient,
        ceres::internal::SparseMatrix * & jacobian_
    ) {
        ceres::internal::Evaluator::EvaluateOptions evaluate_options;
        evaluate_options.new_evaluation_point = true;
        x_ = ceres::ConstVectorRef(reduced_parameters.data(), num_parameters_);
        double x_cost_ = 0;

        residual.resize(num_residuals_);

        int num_effective_parameters_ = evaluator->NumEffectiveParameters();
        gradient.resize(num_effective_parameters_);
        jacobian_ = evaluator->CreateJacobian();
        if (!evaluator->Evaluate(evaluate_options,
                            x_.data(),
                            &x_cost_,
                            residual.data(),
                            gradient.data(),
                            jacobian_)) {
            assert(false && "Residual evalute? failed in DGSSOlver");
            exit(-1);
        }


       
        
        return x_cost_;
    }

    DGSSolver::DGSSolver():
        DistributedSolver()
    {

    }

    
    void DGSSolver::solve() {
        //First we need to get the hessian matrix
        // H = J^T J
        // g = -J^T f(x)
        //Then use DGS/JOR/SOR... to solve H dx = g
        //Then update x <- x + dx

        //First we need to setup the problem and obtain H matrix

        setup();
        ceres::Vector x, residual, gradient;
        ceres::internal::SparseMatrix * jacobian = nullptr;
        double cost = get_x_jacobian_residual(x, residual, gradient, jacobian);
        
        ceres::Matrix J, Jt;
        jacobian->ToDenseMatrix(&J);
        Jt = J.transpose();
        auto H = Jt * J;
        auto g = - Jt * residual;

        std::cout << "Cost now: " << cost << std::endl;
        std::cout << "x (" << x.size() <<") [" << x.transpose() << "]^T" << std::endl;
        std::cout << "f(x) (" << residual.size() <<") [" << residual.transpose() << "]^T" << std::endl;
        // std::cout << "Jacobian\n" << J << std::endl;
        // std::cout << "Hessian\n" << H << std::endl;
        // std::cout << "g(" << g.size() << ") [" << g.transpose() << "]^T" << std::endl;
        ceres::Vector xnew;
        xnew.resize(x.size())
        for (unsigned int i = 0; i < x.size(); i ++ ) {
            double sum = g(i);
            xnew[i] = sum/H(i,i);
        }
    }
}