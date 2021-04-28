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
            // problem_impl->SetParameterBlockConstant(p);
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
        // reduced_program.reset(program->CreateReducedProgram(
        //     &removed_parameter_blocks, &fixed_cost, &error));
        // evaluator_options.evaluation_callback =
        //     reduced_program->mutable_evaluation_callback();
        evaluator_options.evaluation_callback =
            program->mutable_evaluation_callback();

        evaluator_options.context = problem_impl->context();
        evaluator_options.num_eliminate_blocks = 0;
        
        // evaluator.reset(ceres::internal::Evaluator::Create(
        //     evaluator_options, reduced_program.get(), &error));
        evaluator= ceres::internal::Evaluator::Create(
            evaluator_options, program, &error);

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

    void DGSSolver::linearization() {
        setup();

        double cost = get_x_jacobian_residual(x, residual, gradient, jacobian);
        
        ceres::Matrix J, Jt;
        jacobian->ToDenseMatrix(&J);
        Jt = J.transpose();
        H = Jt * J;
        g = - Jt * residual;

        std::cout << "Linearization cost: " << cost << std::endl;
        // std::cout << "x (" << x.size() <<") [" << x.transpose() << "]^T" << std::endl;
        // std::cout << "f(x) (" << residual.size() <<") [" << residual.transpose() << "]^T" << std::endl;
        // std::cout << "gradient (" << gradient.size() <<") [" << gradient.transpose() << "]^T" << std::endl;
        // std::cout << "Jacobian [" << J.rows() << "," << J.cols() << "] \n";
        // // std::cout << J << std::endl;
        // std::cout << "Hessian [" << H.rows() << "," << H.cols() << "] \n";// << H << std::endl;
        // std::cout << "g(" << g.size() << ") [" << g.transpose() << "]^T" << std::endl;
    }


    void DGSSolver::update_remote_poses(std::vector<double*> poses) {
        assert(poses.size() == remote_poses.size() && "New remote poses size must be equal!");
        int index_remote_start = local_poses.size() * PARAM_BLOCK_SIZE;
        for (int i = 0; i < poses.size(); i++) {
            delta_last(index_remote_start+i*PARAM_BLOCK_SIZE) = poses[i][0] - x(index_remote_start+i*PARAM_BLOCK_SIZE);
        }
    }
    
    void DGSSolver::iteration(bool need_linearization) {
        //First we need to get the hessian matrix
        // H = J^T J
        // g = -J^T f(x)
        //Then use DGS/JOR/SOR... to solve H dx = g
        //Then update x <- x + dx

        //First we need to setup the problem and obtain H matrix

        if (need_linearization) {
            linearization();
            delta_last.resize(x.size());
            delta_last.setZero();
            x_last = x;
        }
        
        ceres::Vector delta_;
        delta_.resize(x.size());
        delta_.setZero();

        //Assmue local pose is in the front 
        for (unsigned int i = 0; i < local_poses.size()*PARAM_BLOCK_SIZE; i ++ ) {
            double sum = g(i);
            for (unsigned int j = 0; j < delta_.size(); j++) {
                if (j!=i) {
                    sum = sum + -H(i,j)*delta_last(j);
                }
            }
            delta_[i] = sum/H(i,i);
        }

        double candidate_cost_ = 0;
        // evaluator->Plus(reduced_parameters.data(), delta_.data(), candidate_x_.data());
        x_last = x + delta_;
        evaluator->Evaluate(
            x_last.data(), &candidate_cost_, nullptr, nullptr, nullptr);
        
        std::cout << "Cost after DGS iteration" << candidate_cost_ << std::endl;
        delta_last = delta_;
        // std::cout << "delta (" << x.size() <<") [" << delta_.transpose() << "]^T" << std::endl;
        // std::cout << "xnew (" << x.size() <<") [" << (candidate_x_).transpose() << "]^T" << std::endl;
    }
}