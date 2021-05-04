#include <distributed_solver/distributed_solver.hpp>
#include "ceres/problem_impl.h"
#include "ceres/program.h"
#include "ceres/sparse_matrix.h"
#include "ceres/parameter_block.h"
#include "ceres/triplet_sparse_matrix.h"
#include "ceres/block_sparse_matrix.h"


// #define ENABLE_PROFROLING_OUTPUT


namespace DSLAM {

Eigen::SparseMatrix<double> CreateBlockJacobian(
    const ceres::internal::TripletSparseMatrix& block_jacobian_transpose) {
    typedef Eigen::SparseMatrix<double> SparseMatrix;
    typedef Eigen::Triplet<double> Triplet;

    const int* rows = block_jacobian_transpose.rows();
    const int* cols = block_jacobian_transpose.cols();
    const double* values = block_jacobian_transpose.values();
    int num_nonzeros = block_jacobian_transpose.num_nonzeros();
    std::vector<Triplet> triplets;
    triplets.reserve(num_nonzeros);
    for (int i = 0; i < num_nonzeros; ++i) {
        triplets.push_back(Triplet(rows[i], cols[i], values[i]));
    }

    SparseMatrix block_jacobian(block_jacobian_transpose.num_rows(),
                                block_jacobian_transpose.num_cols());
    block_jacobian.setFromTriplets(triplets.begin(), triplets.end());
    return block_jacobian;
}

    DistributedSolver::DistributedSolver():
    problem_impl(new ceres::internal::ProblemImpl) {

    }
    void DistributedSolver::set_local_poses(std::vector<double*> poses) {
        local_poses.clear();
        for (auto p: poses) {
            local_poses.push_back(p);
        }
        need_setup = true;
    }

    void DistributedSolver::set_remote_poses(std::vector<double*> poses) {
        remote_poses.clear();
        for (auto p: poses) {
            remote_poses.push_back(p);
        }
        need_setup = true;
    }

    void DistributedSolver::set_poses_fixed(std::vector<double*> poses) {
        for (auto p: poses) {
            fixed_poses.push_back(p);
        }
    }
    
    void DistributedSolver::set_pose_fixed(double *p) {
        fixed_poses.push_back(p);
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
        poses_internal_map.clear();

        //May cause memory issue on huge problem now
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

        for (unsigned int i = 0; i < fixed_poses.size(); i++) {
            auto & p = fixed_poses[i];
            problem_impl->SetParameterBlockConstant(p);
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
        
        auto program = problem_impl->mutable_program();
        std::string error;
        reduced_program.reset(program->CreateReducedProgram(
            &removed_parameter_blocks, &fixed_cost, &error));

        int _param_block_count = 0;
        for (auto _param_block : reduced_program->parameter_blocks()) {
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

        index_remote_start = 0;
        for (auto p : local_poses) {
            if (poses_internal_map.find(p) == poses_internal_map.end()) {
                local_poses_internal_index.push_back(-1);
            } else {
                local_poses_internal_index.push_back(poses_internal_map[p].second);
                index_remote_start += PARAM_BLOCK_SIZE;
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

        // evaluator_options.evaluation_callback =
        // program->mutable_evaluation_callback();
        evaluator_options.context = problem_impl->context();
        evaluator_options.num_eliminate_blocks = 0;
        evaluator_options.evaluation_callback =
            reduced_program->mutable_evaluation_callback();

        evaluator = ceres::internal::Evaluator::Create(
            evaluator_options, reduced_program.get(), &error);
        // evaluator = ceres::internal::Evaluator::Create(
        //     evaluator_options, program, &error);

        num_parameters_ = evaluator->NumParameters();
        num_residuals_ = evaluator->NumResiduals();

        reduced_parameters.resize(reduced_program->NumParameters());
        reduced_program->ParameterBlocksToStateVector(reduced_parameters.data());
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
        auto & all_blocks = reduced_program->parameter_blocks();
        double *_param_i = all_blocks[i0]->mutable_user_state();
        double *_param_j = all_blocks[j0]->mutable_user_state();
        std::vector<int> factor_indexs = factor_indexs_between_i_j(_param_i, _param_j);
        for (int i = 0; i < PARAM_BLOCK_SIZE; i ++) {
            for (int j = 0; j <= i; j ++) {
                for (auto k0 : factor_indexs) {
                    for (auto k = k0*RESIDUAL_BLOCK_SIZE; k < k0*RESIDUAL_BLOCK_SIZE+ RESIDUAL_BLOCK_SIZE; k ++)
                        // block(i, j) = block(i, j) + J(k, i+i0*PARAM_BLOCK_SIZE)*J(k, j+j0*PARAM_BLOCK_SIZE);
                        block(i, j) = block(i, j) + J_.coeff(k, i+i0*PARAM_BLOCK_SIZE)*J_.coeff(k, j+j0*PARAM_BLOCK_SIZE);
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
        auto & all_blocks = reduced_program->parameter_blocks();
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

    
}