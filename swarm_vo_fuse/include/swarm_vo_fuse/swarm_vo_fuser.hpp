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
typedef std::map<unsigned int, Eigen::Quaterniond> ID2Quat;

typedef std::function<void(const ID2Vector3d &, const ID2Vector3d &, const ID2Quat &)> ID2VecCallback;

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace Eigen;

float rand_FloatRange(float a, float b)
{
    return ((b - a) * ((float)rand() / RAND_MAX)) + a;
}



class UWBVOFuser
{
    enum FrameLevel {
        SWARM_KEYFRAME,
        SWARM_NONKEYFRAME
    };

    CostFunction*
        _setup_cost_function(const Eigen::MatrixXd & dis_mat,
            const vec_array& self_pos, const vec_array& self_vel, const quat_array & self_quat,
            std::vector<unsigned int> _ids)
    {
        CostFunction* cost_function =
            new SwarmDistanceResidual(dis_mat, self_pos, self_vel, self_quat, _ids, id_to_index, ann_pos);
        return cost_function;
    }

    std::vector<Eigen::MatrixXd> past_dis_matrix;
    std::vector<vec_array> past_self_pos, past_self_vel;
    std::vector<quat_array> past_self_quat;
    std::vector<std::vector<unsigned int>> past_ids;
    std::vector<FrameLevel> past_frame_level;
    int drone_num = 0;

    int solve_count = 0;
    const double min_accept_keyframe_movement = 0.2;

    double Zxyzth[1000] = {0};

            // {-21.48147416 , 12.7661877,   21.20343478,0,
            //  -3.3077791 ,  -9.87719654,  14.30700117,0,
            // -35.50198334, -21.12191708,  32.77340531,0,
            // -22.59650833,  -2.95609427, -20.10679965,0,
            // -31.24850392,  19.58565513,  -4.74885159,0,
            //  33.30543244, -13.61200742, -10.19166553,0,
            //   7.25038821, -20.35836745,   5.3823983 ,0,
            //   8.73040171,  -5.20697205,  23.1825567 ,0,
            //  11.51975686,   3.42533134,   3.74197347,0};

    double covariance_xx[10000];

public:
    std::map<int, int> id_to_index;
    int max_frame_number = 20;
    int min_frame_number = 10;
    int last_drone_num = 0;
    int self_id = -1;
    int thread_num;
    double cost_now = 0;
    double acpt_cost = 0.4;
    ID2VecCallback callback;
    Eigen::Vector3d ann_pos;
    UWBVOFuser(int _max_frame_number,int _min_frame_number, Eigen::Vector3d _ann_pos, double _acpt_cost = 0.4 ,int _thread_num=4):
        max_frame_number(_max_frame_number), min_frame_number(_min_frame_number), ann_pos(_ann_pos),
        thread_num(_thread_num),last_key_frame_self_pos(100),last_key_frame_has_id(100),acpt_cost(_acpt_cost)
    {
       random_init_Zxyz(Zxyzth);
    }

    void random_init_Zxyz(double * _Zxyzth, int start=0)
    {
        for (int i=start;i<100;i++)
        {
            _Zxyzth[i*4] = rand_FloatRange(-30,30);
            _Zxyzth[i*4+1] = rand_FloatRange(-30,30); 
            _Zxyzth[i*4+2] = rand_FloatRange(-30,30); 
            _Zxyzth[i*4 +3] = rand_FloatRange(0,6.28); 
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
    std::map<int, Vector3d> est_vel;

    
    bool has_new_keyframe = false;

    int keyframe_num = 0;

    bool judge_is_key_frame(const Eigen::MatrixXd & dis_matrix, const vec_array & self_pos, const vec_array & self_vel,
        const std::vector<unsigned int> & _ids)
    {
        if (_ids.size() < 2)
            return false;
        if (past_dis_matrix.size() ==0)
            return true;

        if (_ids.size() > drone_num)
        {
            drone_num = _ids.size();
            return true;
        }


        int ptr = 0;
        for (int i = 0; i < _ids.size(); i ++)
        {
            if (_ids[i] == self_id)
            {
                ptr = i;
                // ROS_INFO("self_id %d ptr is %d, pos %f %f %f",self_id, ptr,self_pos[ptr].x(), self_pos[ptr].y(), self_pos[ptr].z() );
                break;
            }
        }

        int _index = (id_to_index)[self_id];
        //self_pos 0 must be self
        /*
        ROS_INFO("index %d ptr %d lapos %f %f %f pos %f %f %f", _index, ptr,
            last_key_frame_self_pos[_index].x(),
            last_key_frame_self_pos[_index].y(),
            last_key_frame_self_pos[_index].z(),
            self_pos[ptr].x(),
            self_pos[ptr].y(),
            self_pos[ptr].z()
         );
        */
        Eigen::Vector3d _diff = self_pos[ptr] - last_key_frame_self_pos[_index];

        if (_diff.norm() > min_accept_keyframe_movement)
        {
            ROS_INFO("KF move %f %f %f, is kf, count %d", _diff.x(), _diff.y(), _diff.z(), past_self_pos.size());
            return true;
        }
        /*
        for (int i = 0; i < _ids.size() ; i ++)
        {
            int _id = _ids[i];
            int _index = (id_to_index)[_id];
            if (last_key_frame_has_id[_index])
            {
                // ROS_INFO("%d %f %f %f", _id, self_pos[i].x(), self_pos[i].y(), self_pos[i].z());
                Eigen::Vector3d _diff = self_pos[i] - last_key_frame_self_pos[_index];
                if (_diff.norm() > min_accept_keyframe_movement)
                {
                    return true;
                }
            }
        }*/
        return false;
    }

    void delete_frame_i(int i)
    {
        past_ids.erase(past_ids.begin() + i);
        past_dis_matrix.erase(past_dis_matrix.begin() + i);
        past_self_pos.erase(past_self_pos.begin() + i);
        past_self_vel.erase(past_self_vel.begin() + i);
        past_self_quat.erase(past_self_quat.begin() + i);
        past_frame_level.erase(past_frame_level.begin() + i);
    }

    void process_frame_clear()
    { 
        int i = 0;

        //Delete non keyframe first
        while ( i < past_ids.size() && past_ids.size() > max_frame_number)
        {
            if (past_frame_level[i] == FrameLevel::SWARM_NONKEYFRAME)
            {
                delete_frame_i(i);
            }
            else
            {
                i++;
            }
        }

        //Delete keyframe in head then
        while (past_ids.size() > max_frame_number)
        {
            delete_frame_i(0);
            keyframe_num --;
        }
    }

    void add_new_data_tick(
        Eigen::MatrixXd dis_matrix,const vec_array & self_pos, 
        const vec_array & self_vel, const quat_array & self_quat, std::vector<unsigned int> _ids)
    {
        process_frame_clear();

        if (judge_is_key_frame(dis_matrix, self_pos, self_vel, _ids))
        {

            keyframe_num ++;
            // ROS_INFO("Its keyframe %d", past_ids.size() + 1);
            past_frame_level.push_back(FrameLevel::SWARM_KEYFRAME);
            // last_key_frame_self_pos = std::vector<Eigen::Vector3d>((id_to_index).size());
            // last_key_frame_has_id =  std::vector<bool>((id_to_index).size());


            std::fill(last_key_frame_has_id.begin(), last_key_frame_has_id.end(), 0);

            for (int i = 0 ; i < _ids.size(); i++)
            {
                int _id = _ids[i];
                // ROS_INFO("lf %d %f %f %f", _id, self_pos[_id].x(),self_pos[_id].y(),self_pos[_id].z() );
                int _index = (id_to_index)[_id];
                last_key_frame_self_pos[_index] = Vector3d(self_pos[i]);
                last_key_frame_has_id[_index] = true;
            }
            has_new_keyframe = true;
        }
        else
        {
            if (past_frame_level.size() < max_frame_number)
            {
                // past_frame_level.push_back(FrameLevel::SWARM_NONKEYFRAME);
            }

        }
        
        if (finish_init)
            EvaluateEstPosition(dis_matrix, self_pos, self_vel, self_quat, _ids, true);

        past_dis_matrix.push_back(dis_matrix);
        past_self_pos.push_back(self_pos);
        past_self_vel.push_back(self_vel);
        past_self_quat.push_back(self_quat);
        past_ids.push_back(_ids);

        if (_ids.size() > drone_num)
        {
            drone_num = _ids.size();
        }


    }


    int last_problem_ptr = 0;
    bool finish_init = false;
    

    Eigen::Vector3d get_estimate_pos(int _id)
    {
        return est_pos[_id];
    }

    void EvaluateEstPosition(Eigen::MatrixXd dis_matrix, vec_array self_pos, vec_array self_vel, quat_array self_quat, std::vector<unsigned int> _ids, bool call_cb = false)
    {

        ID2Vector3d id2vec;
        ID2Vector3d id2vel;
        ID2Quat id2quat;
        SwarmDistanceResidual swarmRes(dis_matrix, self_pos, self_vel, self_quat, _ids, id_to_index, ann_pos);

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
            Eigen::Vector3d vel = swarmRes.est_id_vel_in_k(i, self_ptr, Zxyzth);
            Eigen::Quaterniond quat = swarmRes.est_id_quat_in_k(i, self_ptr, Zxyzth);

            est_pos[_id] = pos;
            est_vel[_id] = vel;
            id2vec[_id] = pos;
            id2vel[_id] = vel;
            id2quat[_id] = quat;
        }

        printf("\nDistance Matrix\n\t\t");
        for (int i = 0;i<drone_num_now;i++)
        {
            int _id_i = _ids[i];
            printf("%d:\t", _id_i);
        }

        for (int i = 0;i<drone_num_now;i++)
        {
            int _id_i = _ids[i];
            int _index_i = id_to_index.at(_id_i);
            printf("\nE/M%d:\t", _id_i);
            for (int j = 0;j<drone_num_now;j++)
            {
                int _id_j = _ids[j];
                int _index_j = id_to_index.at(_id_j);

                double est_d = swarmRes.distance_j_i(j, i, Zxyzth).norm();
                printf("%3.2f:%3.2f\t", est_d, dis_matrix(i, j));
            }

        }

        if (callback != nullptr && call_cb)
            (callback)(id2vec, id2vel, id2quat);
    }
    
    double _ZxyTest[1000] = {0};

    bool solve_with_multiple_init(int start_drone_num, int min_number = 5, int max_number = 10)
    {
        
        double cost = acpt_cost;
        bool cost_updated = false;

        // ROS_INFO("Try to use multiple init to solve expect cost %f", cost);

        for (int i = 0; i < max_number; i++)
        {
            random_init_Zxyz(_ZxyTest, start_drone_num);
            double c = solve_once(_ZxyTest, false);
                ROS_INFO("Got better cost %f", c);

            if (c < cost)
            {
                ROS_INFO("Got better cost %f", c);
                cost_updated = true;
                cost_now = cost = c;
                memcpy(Zxyzth, _ZxyTest, 1000*sizeof(double));
                if (i > min_number)
                {
                    return true;
                }
            }     
        }
        
        return cost_updated;
    }

    double solve()
    {
        if( keyframe_num < min_frame_number)
            return -1;

        if(! has_new_keyframe)
            return cost_now;
        
        if(!finish_init || drone_num > last_drone_num)
        {
            finish_init = solve_with_multiple_init(last_drone_num);
            if (finish_init)
            {
                last_drone_num = drone_num;
                ROS_INFO("Finish init\n");
            }

        }
        else if (has_new_keyframe)
        {
            cost_now = solve_once(this->Zxyzth, true);
        }
        
        if (cost_now > acpt_cost)
            finish_init = false;
        return cost_now;
    }

    double solve_once(double * Zxyzth, bool report=false)
    {

        Problem problem;

        if (solve_count % 10 == 0)
            printf("TICK %d Trying to solve size %ld\n", solve_count, past_dis_matrix.size());

        has_new_keyframe = false;

        for (int i = 0 ; i < past_dis_matrix.size(); i++)
        {
            problem.AddResidualBlock(
                _setup_cost_function(past_dis_matrix[i], past_self_pos[i], past_self_vel[i], past_self_quat[i], past_ids[i]),
                NULL,
                Zxyzth
            );
        }

        // std::cout<<"Finish build problem"<<std::endl;
        last_problem_ptr = past_dis_matrix.size();

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 200;
        options.num_threads = thread_num;
        Solver::Summary summary;
        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::DOGLEG;

        // std::cout << "Start solving problem" << std::endl;
        ceres::Solve(options, &problem, &summary);

        double equv_cost = 2 * summary.final_cost  / past_dis_matrix.size();
        
        equv_cost = equv_cost /(double) (drone_num * (drone_num-1) / 2);
        equv_cost = sqrt(equv_cost);
        if (!report)
        {
            return equv_cost;
        }

        // if (solve_count % 10 == 0)
        std::cout<<"\n\nSize:" << past_dis_matrix.size() <<"\n" << summary.BriefReport() << " Equv cost : " << equv_cost << " Time : " << summary.total_time_in_seconds * 1000 << "ms\n";
        // std::cout << summary.FullReport()<< "\n";


        Covariance::Options cov_options;
        Covariance covariance(cov_options);

        std::vector<std::pair<const double*, const double*> > covariance_blocks;
        covariance_blocks.push_back(std::make_pair(Zxyzth, Zxyzth));
        // bool ret = covariance.Compute(covariance_blocks, &problem);
        bool ret = false;
        EvaluateEstPosition(
            past_dis_matrix.back(), past_self_pos.back(),past_self_vel.back(),past_self_quat.back(), past_ids.back(), true);
        if (ret)
        {
            covariance.GetCovarianceBlock(Zxyzth, Zxyzth, covariance_xx);
            if (solve_count % 10 == 0)
                for (int i = 0;i<drone_num - 1;i++)
                {
                    int _id = past_ids.back()[i];
                    ROS_INFO("i %d id %d x %5.4f y %5.4f z %5.4f :dyaw %5.4f estpos %5.4f %5.4f %5.4f self %3.2f %3.2f %3.2f covr %3.2f %3.2f %3.2f %3.2f\n", 
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
            if (solve_count % 10 == 0)
                for (int i = 0;i<drone_num - 1;i++)
                {
                    int _id = past_ids.back()[i];
                    ROS_INFO("i %d id %d x %5.4f y %5.4f z %5.4f :dyaw %5.4f estpos %5.4f %5.4f %5.4f self %3.2f %3.2f %3.2f\n", 
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

        return equv_cost;
    } 
};
