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
            local_poses.push_back(p);
            // problem_impl->AddParameterBlock(p, PARAM_BLOCK_SIZE);
        }
        need_setup = true;
    }

    void DistributedSolver::set_remote_poses(std::vector<double*> poses) {
        remote_poses.clear();
        for (auto p: poses) {
            remote_poses.push_back(p);
            // problem_impl->AddParameterBlock(p, PARAM_BLOCK_SIZE);
        }
        need_setup = true;
    }

    void DistributedSolver::set_fixed_poses(std::vector<double*> poses) {
        for (auto p: poses) {
            problem_impl->SetParameterBlockConstant(p);
        }
    }
    
    void DistributedSolver::add_residual(ceres::CostFunction * cost_function, std::vector<double*> poses, bool is_huber_norm) {
        ceres::LossFunction *loss_function = nullptr;
        if (is_huber_norm) {
            loss_function = new ceres::HuberLoss(1.0);
        }
        residuals.push_back(std::make_pair(cost_function, poses));
    }

    void DistributedSolver::setup() {
        if (!need_setup) {
            return;
        }
        problem_impl = new ceres::internal::ProblemImpl;
        for (auto p: local_poses) {
            problem_impl->AddParameterBlock(p, PARAM_BLOCK_SIZE);
        }

        for (auto p: remote_poses) {
            problem_impl->AddParameterBlock(p, PARAM_BLOCK_SIZE);
        }

        for (auto & res : residuals) {
            problem_impl->AddResidualBlock(res.first, nullptr, res.second.data(), res.second.size());
        }

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
        x.resize(num_parameters_);
        memcpy(x.data(), reduced_parameters.data(), num_parameters_*sizeof(double));
        
        need_setup = false;
    }


    double DistributedSolver::get_x_jacobian_residual(
        ceres::Vector & x_,
        ceres::Vector & residual,
        ceres::Vector & gradient,
        ceres::internal::SparseMatrix * & jacobian_
    ) const {
        ceres::internal::Evaluator::EvaluateOptions evaluate_options;
        evaluate_options.new_evaluation_point = true;
        x_.resize(num_parameters_);
        memcpy(x_.data(), reduced_parameters.data(), num_parameters_*sizeof(double));
        
        // std::cout << "x_" << x_.transpose() << std::endl;
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
        
        jacobian->ToDenseMatrix(&J);
        Jt = J.transpose();
        H = Jt * J;
        g = - Jt * residual;

        // std::cout << "Linearization cost: " << cost << std::endl;
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
            delta_last(index_remote_start+i*PARAM_BLOCK_SIZE+1) = poses[i][1] - x(index_remote_start+i*PARAM_BLOCK_SIZE+1);
            delta_last(index_remote_start+i*PARAM_BLOCK_SIZE+2) = poses[i][2] - x(index_remote_start+i*PARAM_BLOCK_SIZE+2);
            delta_last(index_remote_start+i*PARAM_BLOCK_SIZE+3) = poses[i][3] - x(index_remote_start+i*PARAM_BLOCK_SIZE+3);
        }

        need_setup = true;
        // printf("Xold [");
        // for (int i = 0; i < poses.size(); i++) {
        //     printf("%f ", x(index_remote_start+i*PARAM_BLOCK_SIZE));
        //     printf("%f ", x(index_remote_start+i*PARAM_BLOCK_SIZE+1));
        //     printf("%f ", x(index_remote_start+i*PARAM_BLOCK_SIZE+2));
        //     printf("%f ", x(index_remote_start+i*PARAM_BLOCK_SIZE+3));
        // }

        // printf("]^T\nXNew [");
        // for (int i = 0; i < poses.size(); i++) {
        //     printf("%f ", poses[i][0]);
        //     printf("%f ", poses[i][1]);
        //     printf("%f ", poses[i][2]);
        //     printf("%f ", poses[i][3]);
        // }

        // printf("]^T\n");

    }

    std::vector<ceres::Vector> DGSSolver::get_last_local_states() {
        std::vector<ceres::Vector> ret;
        for (unsigned int i = 0; i < local_poses.size(); i ++) {
            ceres::Vector v(PARAM_BLOCK_SIZE);
            memcpy(v.data(), x_last.data() + i*PARAM_BLOCK_SIZE, sizeof(double)*PARAM_BLOCK_SIZE);
            ret.push_back(v);
        }
        return ret;
    }
    
    double DGSSolver::cost() const {
        ceres::Vector x_, residual_, gradient_;
        ceres::internal::SparseMatrix * jacobian_ = nullptr;

        ceres::internal::Evaluator::EvaluateOptions evaluate_options;
        evaluate_options.new_evaluation_point = true;
        // x = ceres::ConstVectorRef(reduced_parameters.data(), num_parameters_);
        double x_cost_ = 0;

        residual_.resize(num_residuals_);

        int num_effective_parameters_ = evaluator->NumEffectiveParameters();
        gradient_.resize(num_effective_parameters_);
        jacobian_ = evaluator->CreateJacobian();
        if (x_last.size() == 0) {
            x_ = x;
        } else{
            x_ = x_last;
        }

        if (!evaluator->Evaluate(evaluate_options,
                            x_.data(),
                            &x_cost_,
                            nullptr,
                            nullptr,
                            nullptr)) {
            assert(false && "Residual evalute? failed in DGSSOlver");
            exit(-1);
        }

        memcpy(x_.data(), reduced_parameters.data(), num_parameters_*sizeof(double));

        return x_cost_;
    }

    double DGSSolver::iteration(bool need_linearization) {
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
            x_last.resize(x.size());
            x_last = x;
        }
        
        ceres::Vector delta_;
        delta_.resize(x.size());
        delta_.setZero();

        //Assmue local pose is in the front 
        //Core of Gauss Seidel
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
        
        delta_last = delta_;
        // std::cout << "Cost after DGS iteration" << candidate_cost_ << std::endl;
        // std::cout << "delta (" << x.size() <<") [" << delta_.transpose() << "]^T" << std::endl;
        // std::cout << "xold (" << x.size() <<") [" << (x).transpose() << "]^T" << std::endl;
        // std::cout << "xnew (" << x_last.size() <<") [" << (x_last).transpose() << "]^T" << std::endl;

        return candidate_cost_;
    }
}