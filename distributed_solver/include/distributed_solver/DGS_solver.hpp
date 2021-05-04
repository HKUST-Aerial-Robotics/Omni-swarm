#include "distributed_solver.hpp"
namespace DSLAM{
class DGSSolver: public  DistributedSolver{
    ceres::Vector delta_last;
    ceres::Vector x_last;
public:
    DGSSolver();
    ~DGSSolver() {};
    virtual void linearization();
    virtual double iteration(bool linearization = true) override;
    virtual double cost() const override;
    virtual void update_remote_poses(std::vector<double*> poses) override;
    virtual std::vector<ceres::Vector> get_last_local_states() override;

};
}