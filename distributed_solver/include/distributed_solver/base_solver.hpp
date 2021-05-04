#pragma once
#include <iostream>
#include <map>
#include <chrono>
#include <ctime>
#include <Eigen/Sparse>
#include <ceres/ceres.h>

#define PARAM_BLOCK_SIZE 4
#define RESIDUAL_BLOCK_SIZE 4

namespace DSLAM {
class TicToc
{
    bool stopped = false;
    double delta = 0;
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        if (stopped) {
            return delta;
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

    double stop() {
        stopped = true;
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        delta = elapsed_seconds.count() * 1000;
        return delta;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

class BaseSolver {
protected:
    double evaluation_with_jacobian_time = 0;
    double iteration_time;
    int iterations = 0;
public:
    BaseSolver() {}
    virtual void add_residual(ceres::CostFunction * cost_function, std::vector<double*> poses, bool is_huber_norm=false) = 0;
    virtual void set_local_poses(std::vector<double*> poses) = 0;
    virtual double get_total_iteration_time();
    virtual double get_average_iteration_time();

    virtual void set_poses_fixed(std::vector<double*> poses) = 0;
    virtual void set_pose_fixed(double* poses) = 0;
};


};