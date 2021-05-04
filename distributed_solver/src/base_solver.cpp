#include <distributed_solver/distributed_solver.hpp>
namespace DSLAM {
double BaseSolver::get_total_iteration_time() {
    return iteration_time;
}

double BaseSolver::get_average_iteration_time() {
    return iteration_time/iterations;
}
}