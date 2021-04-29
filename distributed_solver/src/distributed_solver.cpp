#include <distributed_solver/distributed_solver.hpp>
#include "ceres/problem_impl.h"
#include "ceres/program.h"
#include "ceres/sparse_matrix.h"
#include "ceres/parameter_block.h"

#define PARAM_BLOCK_SIZE 4
#define RESIDUAL_BLOCK_SIZE 4

// #define ENABLE_PROFROLING_OUTPUT

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
        local_poses_original_map.clear();
        local_poses_original_map.clear();
        local_poses_internal_index.clear();

        remote_poses_original_map.clear();
        remote_poses_original_map.clear();
        remote_poses_internal_index.clear();


        problem_impl = new ceres::internal::ProblemImpl;
        for (unsigned int i = 0; i < local_poses.size(); i++) {
            auto & p = local_poses[i];
            problem_impl->AddParameterBlock(p, PARAM_BLOCK_SIZE);
            local_poses_original_map[p] = i;
        }

        for (unsigned int i = 0; i < remote_poses.size(); i++) {
            auto & p = remote_poses[i];
            problem_impl->AddParameterBlock(p, PARAM_BLOCK_SIZE);
            remote_poses_original_map[p] = i;
        }

        for (unsigned int i = 0; i < residuals.size(); i++) {
            auto & res = residuals[i];
            problem_impl->AddResidualBlock(res.first, nullptr, res.second.data(), res.second.size());
            for (auto ptr: res.second) {
                if (involved_residuals.find(ptr) == involved_residuals.end()) {
                    involved_residuals[ptr]= std::set<int>();
                }
                involved_residuals[ptr].insert(i);
            }
        }

        ceres::internal::Evaluator::Options evaluator_options;
        evaluator_options.linear_solver_type = ceres::CGNR;
        
        program = problem_impl->mutable_program();
        
        int _param_block_count = 0;
        for (auto _param_block : program->parameter_blocks()) {
            double * _param = _param_block->mutable_user_state();
            auto it = local_poses_original_map.find(_param);
            if (it != local_poses_original_map.end()) {
                //This param block is for local
                poses_internal_map[_param] = std::make_pair(true, _param_block_count);
            } else {
                auto it = remote_poses_original_map.find(_param);
                if (it != remote_poses_original_map.end()) {
                //This param block is for remote
                    poses_internal_map[_param] = std::make_pair(false, _param_block_count);
                } else {
                    assert(true && "Parameter block belong to nothing...");
                }
            }
            _param_block_count ++;
        }

        for (auto p : local_poses) {
            if (poses_internal_map.find(p) == poses_internal_map.end()) {
                local_poses_internal_index.push_back(-1);
            } else {
                assert(poses_internal_map[p].first && "Local poses must be local only");
                local_poses_internal_index.push_back(poses_internal_map[p].second);
            }
        }

        for (auto p : remote_poses) {
            if (poses_internal_map.find(p) == poses_internal_map.end()) {
                remote_poses_internal_index.push_back(-1);
            } else {
                assert(!poses_internal_map[p].first && "Remote poses must be local only");
                remote_poses_internal_index.push_back(poses_internal_map[p].second);
            }
        }



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

    std::vector<int> DistributedSolver::factor_indexs_between_i_j(double * pi, double * pj) {
        std::vector<int> ret;
        if (involved_residuals.find(pi) == involved_residuals.end() ||
            involved_residuals.find(pj) == involved_residuals.end() ) {
            return ret;
        }
        auto factors_i = involved_residuals[pi];
        auto factors_j = involved_residuals[pj];

        for (auto factor_index : factors_i) {
            if (factors_j.find(factor_index) != factors_j.end()) {
                ret.push_back(factor_index);
            }
        }

        return ret;
    }

    ceres::Matrix DistributedSolver::H_block( int i0, int j0) {
        ceres::Matrix block(PARAM_BLOCK_SIZE, PARAM_BLOCK_SIZE);
        block.setZero();
        auto & all_blocks = program->parameter_blocks();
        double *_param_i = all_blocks[i0]->mutable_user_state();
        double *_param_j = all_blocks[j0]->mutable_user_state();
        std::vector<int> factor_indexs = factor_indexs_between_i_j(_param_i, _param_j);
        for (int i = 0; i < PARAM_BLOCK_SIZE; i ++) {
            for (int j = 0; j <= i; j ++) {
                for (auto k0 : factor_indexs) {
                    for (auto k = k0*RESIDUAL_BLOCK_SIZE; k < k0*RESIDUAL_BLOCK_SIZE+ RESIDUAL_BLOCK_SIZE; k ++)
                        block(i, j) = block(i, j) + J(k, i+i0*PARAM_BLOCK_SIZE)*J(k, j+j0*PARAM_BLOCK_SIZE);
                }
                block(j,i) = block(i,j);
            }
        }
        return block;
    }

    ceres::Matrix & DistributedSolver::setup_full_H() {
        if (H.rows() != J.cols() || H.cols()!=J.cols()) {
            H.resize(J.cols(), J.cols());
        }
        TicToc tsetzero;
        H.setZero();
        // std::cout << "set zero time" <<tsetzero.toc() << std::endl;
        auto & all_blocks = program->parameter_blocks();
        for (unsigned i0 = 0; i0 < all_blocks.size(); i0++) {
            for (unsigned j0 = 0; j0 <= i0; j0 ++){
                // printf("Blk %d,%d %d %d\n", i0*PARAM_BLOCK_SIZE, j0*PARAM_BLOCK_SIZE, PARAM_BLOCK_SIZE, PARAM_BLOCK_SIZE);
                // std::cout << H_block(i0, j0);
                auto block = H_block(i0, j0);
                H.block(i0*PARAM_BLOCK_SIZE, j0*PARAM_BLOCK_SIZE, PARAM_BLOCK_SIZE, PARAM_BLOCK_SIZE) = block;
                H.block(j0*PARAM_BLOCK_SIZE, i0*PARAM_BLOCK_SIZE, PARAM_BLOCK_SIZE, PARAM_BLOCK_SIZE) = block.transpose();
            }
        }
        // std::cout << "setup_full_H time" <<tsetzero.toc() << std::endl;
    }

    void DGSSolver::linearization() {
        TicToc tic_linearization;
        setup();

        TicToc tic;
        double cost = get_x_jacobian_residual(x, residual, gradient, jacobian);
        // printf("Linearization: Evaluation with Jacobian %3.1fms ",tic.toc());
        evaluation_with_jacobian_time += tic.toc();
        tic.stop();
        
        //ToDenseMatrix 2.9ms 100x100 direct J^T: 33.2ms Jt * residual 8.7ms LeftMultiply 0.026ms
        TicToc tic2;
        jacobian->ToDenseMatrix(&J);
        tic2.stop();

        TicToc tic3;
        // H = Jt * J;
        // tic3.stop();
        // std::cout << "Hessian [" << H.rows() << "," << H.cols() << "] \n" << H << std::endl;

        // TicToc tic_fastH;
        // setup_full_H();
        // tic_fastH.stop();
        // std::cout << "setup_full_H [" << H.rows() << "," << H.cols() << "] \n" << H << std::endl;
        
        TicToc tic4;
        g.resize(x.rows(), x.cols());
        g.setZero();
        jacobian->LeftMultiply(residual.data(), g.data());
        g = -g;
        tic4.stop();
#ifdef ENABLE_PROFROLING_OUTPUT
        printf("states: %ld residuals %ld linearization %3.1fms Evaluate %3.1fms ToDenseMatrix %3.1fms ", x.size(), residual.size(), tic_linearization.toc(), tic.toc(), tic2.toc());
        // printf("Jt * J %3.1fms  setup_full_H %fms H[%ldx%ld]",tic3.toc(), tic_fastH.toc(), H.rows(), H.cols());
        printf("J^T: %3.1fms Jt * residual %3.3fms\n",tic3.toc(), tic4.toc());
#endif
        // std::cout << "Linearization cost: " << cost << std::endl;
        // std::cout << "x (" << x.size() <<") [" << x.transpose() << "]^T" << std::endl;
        // std::cout << "f(x) (" << residual.size() <<") [" << residual.transpose() << "]^T" << std::endl;
        // std::cout << "gradient (" << gradient.size() <<") [" << gradient.transpose() << "]^T" << std::endl;
        // std::cout << "Jacobian [" << J.rows() << "," << J.cols() << "] \n";
        // std::cout << J << std::endl;
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

        TicToc tic_iteration;
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

        // Assmue local pose is in the front 
        // Core of Gauss Seidel
        
        //-O0 direct method tic_gs 253.295ms block method tic_gs 12.0005ms
        //-O3 direct method tic_gs 10.608ms block method tic_gs 0.246023ms

        TicToc tic_gs;
        // setup_full_H();
        // for (unsigned int i = 0; i < local_poses.size()*PARAM_BLOCK_SIZE; i ++ ) {
        //     double sum = g(i);
        //     for (unsigned int j = 0; j < delta_.size(); j++) {
        //         if (j!=i) {
        //             sum = sum + -H(i,j)*delta_last(j);
        //         }
        //     }
        //     delta_[i] = sum/H(i,i);
        //     // delta_last[i] = delta_[i];
        // }
        // std::cout << "direct method tic_gs" << tic_gs.toc();
        // std::cout << " delta (" << x.size() <<") [" << delta_.transpose() << "]^T" << std::endl;
        
        delta_.setZero();
        //Block method even gives a better convergence!
        // tic_gs.tic();
        for (unsigned int i = 0; i < local_poses.size(); i ++ ) {
            double * _p = local_poses[i];
            if (involved_residuals.find(_p) == involved_residuals.end()) {
                continue;
            }
            ceres::Vector sum = g.block(i*PARAM_BLOCK_SIZE, 0, PARAM_BLOCK_SIZE, 1);
            auto _residuals_indexs = involved_residuals[_p];
            for (auto _residuals_index: _residuals_indexs) {
                auto _factor = residuals[_residuals_index];
                for (auto * ptr_j : _factor.second) {
                    int j = poses_internal_map[ptr_j].second;
                    if (i!= j) {
                        auto Hij = H_block(i, j);
                        sum = sum - Hij*delta_last.block(j*PARAM_BLOCK_SIZE, 0, PARAM_BLOCK_SIZE, 1);
                    }
                }
            }
            ceres::Vector delta =  H_block(i, i).inverse() * sum;
            memcpy(delta_.data()+i*PARAM_BLOCK_SIZE, delta.data(), PARAM_BLOCK_SIZE*sizeof(double));
            memcpy(delta_last.data() + i*PARAM_BLOCK_SIZE, delta.data(), PARAM_BLOCK_SIZE*sizeof(double));
        }

        // std::cout << "block method tic_gs" << tic_gs.toc() << std::endl;
        // std::cout << " delta (" << x.size() <<") [" << delta_.transpose() << "]^T" << std::endl;

        double candidate_cost_ = 0;
        x_last = x + delta_;

        evaluator->Evaluate(
            x_last.data(), &candidate_cost_, nullptr, nullptr, nullptr);
        
        delta_last = delta_;
        // std::cout << "Cost after DGS iteration" << candidate_cost_ << std::endl;
        // std::cout << "delta (" << x.size() <<") [" << delta_.transpose() << "]^T" << std::endl;
        // std::cout << "xold (" << x.size() <<") [" << (x).transpose() << "]^T" << std::endl;
        // std::cout << "xnew (" << x_last.size() <<") [" << (x_last).transpose() << "]^T" << std::endl;
        iteration_time += tic_iteration.toc();
        iterations ++;

#ifdef ENABLE_PROFROLING_OUTPUT
        printf("Iteration time %3.1fms/AVG%3.1fms ", tic_iteration.toc(), iteration_time/iterations);
        printf("Gauss Seidel time %3.1fms\n", tic_gs.toc());
#endif
        return candidate_cost_;
    }
}