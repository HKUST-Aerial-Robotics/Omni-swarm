#include <distributed_solver/DGS_solver.hpp>
#include "ceres/problem_impl.h"
#include "ceres/program.h"
#include "ceres/sparse_matrix.h"
#include "ceres/parameter_block.h"
#include "ceres/triplet_sparse_matrix.h"
#include "ceres/block_sparse_matrix.h"

namespace DSLAM {
    DGSSolver::DGSSolver():
        DistributedSolver()
    {

    }

void DGSSolver::linearization() {
    TicToc tic_linearization;
    setup();

    TicToc tic;
    double cost = get_x_jacobian_residual(x, residual, gradient, jacobian);
    // printf("Linearization: Evaluation with Jacobian %3.1fms ",tic.toc());
    evaluation_with_jacobian_time += tic.toc();
    tic.stop();
    
    //ToDenseMatrix 2.9ms 100x100 direct J^T: 33.2ms Jt * residual 8.7ms LeftMultiply 0.026ms toEigenSpare 0.19ms
    // TicToc tic2;
    // jacobian->ToDenseMatrix(&J);
    // tic2.stop();

    TicToc tic3;
    auto Jtri = new ceres::internal::TripletSparseMatrix;
    ((ceres::internal::BlockSparseMatrix*) jacobian)->ToTripletSparseMatrix(Jtri);
    J_ = CreateBlockJacobian(*Jtri);
    tic3.stop();
    
    TicToc tic4;
    g.resize(x.rows(), x.cols());
    g.setZero();
    jacobian->LeftMultiply(residual.data(), g.data());
    g = -g;
    tic4.stop();
#ifdef ENABLE_PROFROLING_OUTPUT
    // printf("states: %ld residuals %ld linearization %3.1fms Evaluate %3.1fms ToDenseMatrix %3.1fms toEigenSpare %3.2fms", 
        // x.size(), residual.size(), tic_linearization.toc(), tic.toc(), tic2.toc(), tic3.toc());
    printf("states: %ld residuals %ld linearization %3.1fms Evaluate %3.1fms toEigenSpare %3.2fms", 
        x.size(), residual.size(), tic_linearization.toc(), tic.toc(), tic3.toc());

    printf("Jt * residual %3.3fms\n", tic4.toc());
#endif
    // std::cout << "Linearization cost: " << cost << std::endl;
    // std::cout << "x (" << x.size() <<") [" << x.transpose() << "]^T" << std::endl;
    // std::cout << "f(x) (" << residual.size() <<") [" << residual.transpose() << "]^T" << std::endl;
    // std::cout << "gradient (" << gradient.size() <<") [" << gradient.transpose() << "]^T" << std::endl;
    // std::cout << "Jacobian [" << J.rows() << "," << J.cols() << "] \n";
    // std::cout << J << std::endl;
    // std::cout << "Sparse J\n"<< Eigen::MatrixXd(J_) << std::endl;
    // std::cout << "Hessian [" << H.rows() << "," << H.cols() << "] \n";// << H << std::endl;
    // std::cout << "g(" << g.size() << ") [" << g.transpose() << "]^T" << std::endl;
}


void DGSSolver::update_remote_poses(std::vector<double*> poses) {
    assert(poses.size() == remote_poses.size() && "New remote poses size must be equal!");
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
    for (unsigned int ii = 0; ii < local_poses.size(); ii ++) {
        int i = local_poses_internal_index[ii];
        if (i >= 0) {
            ceres::Vector v;
            v.resize(PARAM_BLOCK_SIZE);
            v = x_last.block(i*PARAM_BLOCK_SIZE, 0, PARAM_BLOCK_SIZE, 1);
            ret.push_back(v);
        } else {
            ceres::Vector v;
            v.resize(PARAM_BLOCK_SIZE);
            memcpy(v.data(), local_poses[ii], sizeof(double)*PARAM_BLOCK_SIZE);
            ret.push_back(v);
        }
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
    for (unsigned int ii = 0; ii < local_poses.size(); ii ++ ) {
        int i = local_poses_internal_index[ii];
        if (local_poses_internal_index[ii] < 0) {
            continue;
        }
        double * _p = local_poses[ii];
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