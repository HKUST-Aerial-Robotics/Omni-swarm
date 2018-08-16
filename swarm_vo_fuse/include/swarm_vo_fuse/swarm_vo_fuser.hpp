#include <iostream>
#include "glog/logging.h"
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <time.h>
#include <thread>  
#include <unistd.h>
#include "swarm_vo_costfunc.hpp"
#include <functional>

typedef std::map<unsigned int, Eigen::Vector3d> ID2Vector3d;
typedef std::function<void(const ID2Vector3d &)> ID2VecCallback;

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace swarm_drone_proxy;

using namespace Eigen;

float rand_FloatRange(float a, float b)
{
    return ((b - a) * ((float)rand() / RAND_MAX)) + a;
}



class UWBVOFuser
{

    CostFunction*
        _setup_cost_function(const Eigen::MatrixXd & dis_mat,const vec_array& self_pos, std::vector<unsigned int> _ids)
    {
        CostFunction* cost_function =
            new SwarmDistanceResidual(dis_mat, self_pos, _ids, id_to_index);
        return cost_function;
    }

    std::vector<Eigen::MatrixXd> past_dis_matrix;
    std::vector<vec_array> past_self_pos;
    std::vector<std::vector<unsigned int>> past_ids;
    int drone_num;

    int solve_count = 0;
    const double min_accept_keyframe_movement = 0.2;

    double Zxyzth[10000] = 
            {-21.48147416 , 12.7661877,   21.20343478,0,
             -3.3077791 ,  -9.87719654,  14.30700117,0,
            -35.50198334, -21.12191708,  32.77340531,0,
            -22.59650833,  -2.95609427, -20.10679965,0,
            -31.24850392,  19.58565513,  -4.74885159,0,
             33.30543244, -13.61200742, -10.19166553,0,
              7.25038821, -20.35836745,   5.3823983 ,0,
              8.73040171,  -5.20697205,  23.1825567 ,0,
             11.51975686,   3.42533134,   3.74197347,0};

    double covariance_xx[10000];

public:
    std::map<int, int> id_to_index;
    int max_frame_number;
    int self_id = -1;
    ID2VecCallback * callback = nullptr;
    UWBVOFuser(int _max_frame_number, ID2VecCallback* _callback=nullptr):
        max_frame_number(_max_frame_number),callback(_callback)
    {
       random_init_Zxyz();
    }

    void random_init_Zxyz()
    {
        for (int i=0;i<1000;i++)
        {
            Zxyzth[i*4] = rand_FloatRange(-30,30);
            Zxyzth[i*4+1] = rand_FloatRange(-30,30); 
            Zxyzth[i*4+2] = rand_FloatRange(-30,30); 
            Zxyzth[i*4 +3] = rand_FloatRange(0,6.28); 
        }
    }

    void constrain_Z()
    {
        for (int i=0;i<9;i++)
        {
            Zxyzth[i*4 + 3] = fmodf(Zxyzth[i*4 +3], 2*M_PI);
        }
    }

    std::vector<Eigen::Vector3d> last_key_frame_self_pos;
    std::vector<bool> last_key_frame_has_id;
    std::map<int, Vector3d> est_pos;
    
    bool has_new_keyframe = false;

    bool judge_is_key_frame(const Eigen::MatrixXd & dis_matrix, const vec_array & self_pos,
        const std::vector<unsigned int> & _ids)
    {
        if (past_dis_matrix.size() ==0)
            return true;
        if (_ids.size() > drone_num)
            return true;

        
        for (int _id : _ids)
        {
            int _index = (id_to_index)[_id];
            if (last_key_frame_has_id[_index])
            {
                Eigen::Vector3d _diff = self_pos[_id] - last_key_frame_self_pos[_index];
                if (_diff.norm() > min_accept_keyframe_movement)
                {
                    return true;
                }
            }
        }
        return false;
    }

    void add_new_data_tick(Eigen::MatrixXd dis_matrix, vec_array self_pos, std::vector<unsigned int> _ids)
    {
        if (judge_is_key_frame(dis_matrix, self_pos, _ids))
        {

            last_key_frame_self_pos = std::vector<Eigen::Vector3d>((id_to_index).size());
            last_key_frame_has_id =  std::vector<bool>((id_to_index).size());
            std::fill(last_key_frame_has_id.begin(), last_key_frame_has_id.end(), 0);

            for (int _id : _ids)
            {
                int _index = (id_to_index)[_id];
                last_key_frame_self_pos[_index] = self_pos[_id];
                last_key_frame_has_id[_index] = true;
            }


            past_dis_matrix.push_back(dis_matrix);
            past_self_pos.push_back(self_pos);
            past_ids.push_back(_ids);

            if (_ids.size() > drone_num)
            {
                drone_num = _ids.size();
            }

            if (past_ids.size() > max_frame_number)
            {
                // past_ids.erase(past_ids.begin());
                // past_dis_matrix.erase(past_dis_matrix.begin());
                // past_self_pos.erase(past_self_pos.begin());
            }

            has_new_keyframe = true;
        }

        else if (solve_count > 0)
        {
            auto id2vec = EvaluateEstPosition(dis_matrix, self_pos, _ids);
            if (callback != nullptr)
                (*callback)(id2vec);
        }
    }

    int last_problem_ptr = 0;
    

    Eigen::Vector3d get_estimate_pos(int _id)
    {
        return est_pos[_id];
    }

    ID2Vector3d EvaluateEstPosition(Eigen::MatrixXd dis_matrix, vec_array self_pos, std::vector<unsigned int> _ids)
    {
        ID2Vector3d id2vec;

        SwarmDistanceResidual swarmRes(dis_matrix, self_pos, _ids, id_to_index);

        int drone_num_now = _ids.size();

        int self_ptr = 0;
        for (int i = 0; i< _ids.size(); i++)
        {
            if (_ids[i] == self_id)
            {
                self_ptr = i;
                break;
            }
        }

        for (int i = 0; i < _ids.size(); i++)
        {
            int _id = _ids[i];
            Eigen::Vector3d pos = swarmRes.est_id_pose_in_k(i, self_ptr, Zxyzth);
            est_pos[_id] = pos;
            id2vec[_id] = pos;
        }


        return id2vec;

        /*
        for (int i = 0;i<drone_num_now;i++)
        {
            int _id = _ids[i];
            int _index = id_to_index->at(_id);
            printf("i %d index %d id %d x %5.4f y %5.4f z %5.4f :dyaw %5.4f estpos %5.4f %5.4f %5.4f self %3.2f %3.2f %3.2f\n", 
                i,
                _index,
                _id,
                Zxyzth[(_index-1)*4],
                Zxyzth[(_index-1)*4+1],
                Zxyzth[(_index-1)*4+2],
                Zxyzth[(_index-1)*4+3],
                est_pos[_id].x(),
                est_pos[_id].y(),
                est_pos[_id].z(),
                self_pos[i].x(),
                self_pos[i].y(),
                self_pos[i].z()
            );
        }
        */
    }

    void solve()
    {


        Problem problem;

        if (past_dis_matrix.size() < max_frame_number || ! has_new_keyframe)
            return;
        
        if (solve_count % 10 == 0)
            printf("TICK %d Trying to solve size %ld\n", solve_count, past_dis_matrix.size());

        int end_ptr = past_dis_matrix.size();
        int start_ptr = end_ptr - max_frame_number;
        has_new_keyframe = false;

        for (int i=start_ptr; i < end_ptr; i++)
        {
            problem.AddResidualBlock(
                _setup_cost_function(past_dis_matrix[i], past_self_pos[i], past_ids[i]),
                NULL,
                Zxyzth
            );
        }

        // std::cout<<"Finish build problem"<<std::endl;
        last_problem_ptr = past_dis_matrix.size();

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 200;
        options.num_threads = 1;
        Solver::Summary summary;
        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::DOGLEG;

        // std::cout << "Start solving problem" << std::endl;
        ceres::Solve(options, &problem, &summary);

        // EvaluateEstPosition(past_dis_matrix.back(), past_self_pos.back(), past_ids.back());
        if (solve_count % 10 == 0)
            std::cout << summary.BriefReport()<< " Time : " << summary.total_time_in_seconds << "\n";
        // std::cout << summary.FullReport()<< "\n";


        Covariance::Options cov_options;
        Covariance covariance(cov_options);

        std::vector<std::pair<const double*, const double*> > covariance_blocks;
        covariance_blocks.push_back(std::make_pair(Zxyzth, Zxyzth));
        bool ret = covariance.Compute(covariance_blocks, &problem);

        if (ret)
        {
            // auto id2vec = EvaluateEstPosition(dis_matrix, self_pos, _ids);
            auto id2vec = EvaluateEstPosition(
                past_dis_matrix.back(), past_self_pos.back(), past_ids.back());

            if (callback != nullptr)
                (*callback)(id2vec);

            covariance.GetCovarianceBlock(Zxyzth, Zxyzth, covariance_xx);
            if (solve_count % 100 == 0)
                for (int i = 0;i<drone_num - 1;i++)
                {
                    int _id = past_ids.back()[i];
                    printf("i %d id %d x %5.4f y %5.4f z %5.4f :dyaw %5.4f estpos %5.4f %5.4f %5.4f self %3.2f %3.2f %3.2f covr %3.2f %3.2f %3.2f %3.2f\n", 
                        i,
                        _id,
                        Zxyzth[i*4],
                        Zxyzth[i*4+1],
                        Zxyzth[i*4+2],
                        Zxyzth[i*4+3],
                        est_pos[_id].x(),
                        est_pos[_id].y(),
                        est_pos[_id].z(),
                        past_self_pos.back()[i].x(),
                        past_self_pos.back()[i].y(),
                        past_self_pos.back()[i].z(),
                        covariance_xx[(i*4)*(drone_num-1)*4 + (i*4)],
                        covariance_xx[(i*4 + 1)*(drone_num-1)*4 + (i*4 + 1)],
                        covariance_xx[(i*4 + 2)*(drone_num-1)*4 + (i*4 + 2)],
                        covariance_xx[(i*4 + 3)*(drone_num-1)*4 + (i*4 + 3)]
                    );
                }
        }
        else
        {
            if (solve_count % 100 == 0)
                for (int i = 0;i<drone_num - 1;i++)
                {
                    int _id = past_ids.back()[i];
                    printf("i %d id %d x %5.4f y %5.4f z %5.4f :dyaw %5.4f estpos %5.4f %5.4f %5.4f self %3.2f %3.2f %3.2f\n", 
                        i,
                        _id,
                        Zxyzth[i*4],
                        Zxyzth[i*4+1],
                        Zxyzth[i*4+2],
                        Zxyzth[i*4+3],
                        est_pos[_id].x(),
                        est_pos[_id].y(),
                        est_pos[_id].z(),
                        past_self_pos.back()[i].x(),
                        past_self_pos.back()[i].y(),
                        past_self_pos.back()[i].z()
                    );
                }
        }

        solve_count ++;

    } 
};
