#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include <algorithm>
#include <map>
#include <time.h>
#include <thread>
#include <unistd.h>
#include <functional>
#include <swarm_localization/swarm_types.hpp>
#include <mutex>
#include <swarm_msgs/LoopConnection.h>

typedef std::map<int, Eigen::Vector3d> ID2Vector3d;
typedef std::map<int, Eigen::Quaterniond> ID2Quat;

using namespace Swarm;
using namespace Eigen;
using namespace ceres;

struct SwarmFrameError;
struct SwarmHorizonError;
struct SwarmLoopError;

class LocalizationDAInit;

inline float rand_FloatRange(float a, float b) {
    return ((b - a) * ((float) rand() / RAND_MAX)) + a;
}

inline Eigen::Vector3d rand_FloatRange_vec(float a, float b) {
    return Eigen::Vector3d(
        rand_FloatRange(a, b),
        rand_FloatRange(a, b),
        rand_FloatRange(a, b)
    );
}

typedef ceres::DynamicAutoDiffCostFunction<SwarmFrameError, 7>  SFErrorCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmHorizonError, 7> HorizonCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmLoopError, 7> LoopCost;

//Poses is dict of timestamp and then id;
//state<ts,id>
typedef std::map<int64_t, std::map<int,double*>> EstimatePoses;
typedef std::map<int64_t, std::map<int, Eigen::Matrix4d>> EstimateCOV;
typedef std::map<int, std::map<int64_t,double*>> EstimatePosesIDTS;
typedef std::vector<std::pair<int64_t, int>> TSIDArray;
typedef std::map<int, std::map<int64_t, int>>  IDTSIndex;


struct swarm_localization_solver_params{
    int max_frame_number = 100;
    int min_frame_number = 5;
    int dense_frame_number = 20;
    float acpt_cost = 10;
    int thread_num = 1;
    float kf_movement = 0.2;
    float init_xy_movement = 2.0;
    float init_z_movement = 1.0;
    int self_id = -1;
    std::string cgraph_path;
    float DA_TRI_accept_thres = 0.1;
    bool enable_cgraph_generation = false;
    float loop_outlier_threshold_pos = 1.0;
    float loop_outlier_threshold_yaw = 1.0;
    bool enable_detection;
    bool enable_loop;
    bool enable_distance;
};

class SwarmLocalizationSolver {

    std::mutex solve_lock;
    std::mutex predict_lock;
    std::vector<SwarmFrame> sf_sld_win;
    std::map<int64_t, SwarmFrame> all_sf;
    int64_t last_kf_ts = 0;
    std::vector<int64_t> last_saved_est_kf_ts;
    unsigned int drone_num = 0;

    unsigned int solve_count = 0;

    swarm_localization_solver_params params;


    std::vector<swarm_msgs::LoopConnection> all_loops;
    std::vector<swarm_msgs::node_detected_xyzyaw> all_detections;

    EstimatePoses est_poses_tsid, est_poses_tsid_saved;
    EstimatePosesIDTS est_poses_idts, est_poses_idts_saved;
    EstimateCOV est_cov_tsid;

    unsigned int max_frame_number = 100;
    unsigned int min_frame_number = 5;
    unsigned int dense_frame_number = 20;

    std::set<int> all_nodes;

    unsigned int last_drone_num = 0;

    std::map<unsigned int, unsigned int> node_kf_count;

    std::vector<Swarm::GeneralMeasurement2Drones*> good_2drone_measurements;
    std::map<int, std::set<int>> loop_edges;

    bool has_new_keyframe = false;

    bool enable_cgraph_generation;

    void delete_frame_i(int i);

    bool is_frame_useful(unsigned int i) const;

    void process_frame_clear();

    void random_init_pose(EstimatePoses &swarm_est_poses, EstimatePosesIDTS &est_poses_idts);

    void init_dynamic_nf_in_keyframe(int64_t ts, NodeFrame &_nf);

    void init_static_nf_in_keyframe(int64_t ts, NodeFrame &_nf);

    void sync_est_poses(const EstimatePoses &_est_poses_tsid);

    std::vector<Swarm::GeneralMeasurement2Drones*> find_available_loops_detections(std::map<int, std::set<int>> & loop_edges) const;

    bool find_node_frame_for_measurement_2drones(const Swarm::GeneralMeasurement2Drones * loc, int & _index_a, int &_index_b) const;

    bool loop_from_src_loop_connection(const swarm_msgs::LoopConnection & _loc, Swarm::LoopConnection & loc_ret, double & dt_err, double & dpos) const;

    bool detection_from_src_node_detection(const swarm_msgs::node_detected_xyzyaw & _loc, Swarm::DroneDetection & loc_ret, double & dt_err, double & dpos) const;

    CostFunction *
    _setup_cost_function_by_sf(const SwarmFrame &sf, std::map<int, int> id2poseindex, bool is_lastest_frame, int & res_num) const;


    void
    setup_problem_with_sferror(const EstimatePoses &swarm_est_poses, Problem &problem, const SwarmFrame &sf, TSIDArray & param_indexs, bool is_lastest_frame) const;

    CostFunction *
    _setup_cost_function_by_nf_win(std::vector<NodeFrame> &nf_win, const std::map<int64_t, int> & ts2poseindex, bool is_self) const;
    
    void setup_problem_with_sfherror(const EstimatePosesIDTS & est_poses_idts, Problem &problem, int _id) const;
    
    CostFunction *
    _setup_cost_function_by_loop(const std::vector<Swarm::GeneralMeasurement2Drones*> & loops, IDTSIndex _id_ts_poseindex) const;

    void setup_problem_with_loops(const EstimatePosesIDTS & est_poses_idts, Problem &problem) const;
    
    
    void cutting_edges();

    double solve_once(EstimatePoses &swarm_est_poses, EstimatePosesIDTS &est_poses_idts, bool report = false);
    
    int judge_is_key_frame(const SwarmFrame &sf);

    void add_as_keyframe(const SwarmFrame &sf);
    void print_frame(const SwarmFrame & sf) const;
    void replace_last_kf(const SwarmFrame & sf);
    
    bool solve_with_multiple_init(int max_number = 10);
    
   

    inline unsigned int sliding_window_size() const;
    bool NFnotMoving(const NodeFrame & _nf1, const NodeFrame & nf2) const;

    std::pair<Eigen::Vector3d, Eigen::Vector3d> boundingbox_sldwin(int _id) const;

    void estimate_observability();
    std::set<int> loop_observable_set(const std::map<int, std::set<int>> & loop_edges) const;

    void generate_cgraph();

    bool generate_full_path = false;

public:
    int self_id = -1;
    unsigned int thread_num;
    double cost_now = 0;
    double acpt_cost = 0.4;
    double min_accept_keyframe_movement = 0.2;

    bool finish_init = false;

    bool enable_to_init = false;
    ros::Time last_est_time_tick = ros::Time::now();
    float init_xy_movement = 2.0;
    float init_z_movement = 1.0;
    float loop_outlier_threshold_pos = 1.0;
    float loop_outlier_threshold_yaw = 1.0;

    bool enable_detection;
    bool enable_loop;
    bool enable_distance;

    std::map <int, bool> yaw_observability;
    std::map <int, bool> pos_observability;

    std::map<int, Swarm::Path> pathes;

    std::string cgraph_path = "";

    SwarmLocalizationSolver(const swarm_localization_solver_params & params);
    
    void add_new_swarm_frame(const SwarmFrame &sf);

    void add_new_loop_connection(const swarm_msgs::LoopConnection & loop_con);

    void add_new_detection(const swarm_msgs::node_detected_xyzyaw & detected);

    SwarmFrameState PredictSwarm(const SwarmFrame &sf) const;

    bool PredictNode(const NodeFrame & nf, Pose & _pose, Eigen::Matrix4d & cov) const;
    bool NodeCooridnateOffset(int _id, Pose & _pose, Eigen::Matrix4d & cov) const;

    bool CanPredictSwarm() {
        return finish_init;
    }


    double solve_time_count = 0;


    double solve();


    
};