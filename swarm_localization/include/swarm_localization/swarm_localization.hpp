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
#include <functional>
#include <swarm_localization/swarm_types.hpp>
#include <set>

#define DEBUG_OUTPUT_POSES

typedef std::map<int, Eigen::Vector3d> ID2Vector3d;
typedef std::map<int, Eigen::Quaterniond> ID2Quat;

using namespace swarm;
using namespace Eigen;

struct SwarmFrameError;
struct SwarmHorizonError;

inline float rand_FloatRange(float a, float b) {
    return ((b - a) * ((float) rand() / RAND_MAX)) + a;
}

typedef ceres::DynamicAutoDiffCostFunction<SwarmFrameError, 7>  SFErrorCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmHorizonError, 7> HorizonCost;

//Poses is dict of timestamp and then id;
//state<ts,id>
typedef std::map<int64_t, std::map<int,double*>> EstimatePoses;
typedef std::map<int, std::map<int64_t,double*>> EstimatePosesIDTS;

class SwarmLocalizationSolver {


    std::vector<SwarmFrame> sf_sld_win;
    std::map<int64_t, SwarmFrame> all_sf;
    int64_t last_kf_ts = 0;
    unsigned int drone_num = 0;

    unsigned int solve_count = 0;
    const double min_accept_keyframe_movement = 0.2;

//    std::vector<double*> _swarm_est_poses;
//

    EstimatePoses swarm_poses_state;
    EstimatePosesIDTS nodeid_ts_poses;

public:
    std::set<int> all_nodes;
    unsigned int max_frame_number = 20;
    unsigned int min_frame_number = 10;
    unsigned int last_drone_num = 0;
    int self_id = -1;
    unsigned int thread_num;
    double cost_now = 0;
    double acpt_cost = 0.4;

    int last_problem_ptr = 0;
    bool finish_init = false;

    std::map<int, Vector3d> est_pos;
    std::map<int, Vector3d> est_vel;


    ros::Time last_est_time_tick = ros::Time::now();

    std::map<unsigned int, unsigned int> node_kf_count;


    bool has_new_keyframe = false;


    SwarmLocalizationSolver(int _max_frame_number, int _min_frame_number, double _acpt_cost = 0.4,
                            int _thread_num = 4) :
            max_frame_number(_max_frame_number), min_frame_number(_min_frame_number),
            thread_num(_thread_num), acpt_cost(_acpt_cost) {
    }


    void random_init_pose(EstimatePoses &est_poses, EstimatePosesIDTS &est_poses2, int start = 0, int end = 100);

    bool detect_outlier(const SwarmFrame &sf);

    std::vector<int> judge_is_key_frame(const SwarmFrame &sf);

    void delete_frame_i(int i);

    bool is_frame_useful(unsigned int i);

    void process_frame_clear();

    void init_pose(int _id, int64_t ts, double *_p);


    void add_as_keyframe(const SwarmFrame &sf);

    void add_new_swarm_frame(const SwarmFrame &sf);

    Eigen::Vector3d get_estimate_pos(int _id);
    // Eigen::Vector3d predict_pos(Eigen::Vector3d pos, Eigen::e)

    bool PredictSwarm(const SwarmFrame &sf, SwarmFrameState &_s_state);


    bool solve_with_multiple_init(int start_drone_num, int min_number = 5, int max_number = 10);

    double solve();

    inline unsigned int sliding_window_size() const;

    CostFunction *
    _setup_cost_function_by_sf(SwarmFrame &sf, std::map<int, int> id2poseindex, bool is_lastest_frame);


    void
    setup_problem_with_sferror(EstimatePoses &swarm_est_poses, Problem &problem, SwarmFrame &sf, bool is_lastest_frame);

    CostFunction *
    _setup_cost_function_by_nf_win(std::vector<NodeFrame> &nf_win, std::map<int64_t, int> ts2poseindex, bool is_self);

    void setup_problem_with_sfherror(EstimatePosesIDTS est_poses_idts, Problem &problem, int _id);

    double solve_once(EstimatePoses &swarm_est_poses, EstimatePosesIDTS &est_poses_idts, bool report = false);
};