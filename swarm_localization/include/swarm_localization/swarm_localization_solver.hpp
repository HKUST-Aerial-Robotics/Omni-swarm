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
#include <swarm_msgs/swarm_types.hpp>
#include <mutex>
#include <swarm_msgs/LoopEdge.h>
#include <swarm_localization/swarm_outlier_rejection.hpp>
#include <swarm_localization/swarm_localization_params.hpp>


using namespace Swarm;
using namespace Eigen;
using namespace ceres;

struct SwarmFrameError;
struct SwarmHorizonError;
struct RelativePoseFactor4d;

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


Swarm::Pose Predict_By_VO(const Swarm::Pose & vo_now, const Swarm::Pose & vo_ref, const Swarm::Pose & est_pose_ref, bool is_yaw_only);


//Poses is dict of timestamp and then id;
//state<ts,id>
typedef std::map<TsType, std::map<int,double*>> EstimatePoses;
typedef std::map<TsType, std::map<int, Eigen::Matrix4d>> EstimateCOV;
typedef std::map<int, std::map<TsType,double*>> EstimatePosesIDTS;
typedef std::vector<std::pair<TsType, int>> TSIDArray;
typedef std::map<int, std::map<TsType, int>>  IDTSIndex;


class SwarmLocalizationSolver {

    std::mutex solve_lock;
    std::mutex predict_lock;
    std::vector<SwarmFrame> sf_sld_win;
    std::map<TsType, SwarmFrame> all_sf;
    TsType last_kf_ts = 0;
    ros::Time last_loop_ts;
    std::vector<TsType> last_saved_est_kf_ts;
    unsigned int drone_num = 0;

    unsigned int solve_count = 0;


    swarm_localization_solver_params params;

    std::vector<Swarm::LoopEdge> all_detections_6d; //Actually 4d
    std::vector<Swarm::LoopEdge> good_loops;
    std::vector<Swarm::LoopEdge> all_loops;

    EstimatePoses est_poses_tsid, est_poses_tsid_saved;
    EstimatePosesIDTS est_poses_idts, est_poses_idts_saved;
    EstimateCOV est_cov_tsid;

    unsigned int max_frame_number = 100;
    unsigned int min_frame_number = 5;
    unsigned int dense_frame_number = 20;
    
    float max_solver_time;

    std::set<int> all_nodes;
    std::set<int> estimated_nodes;

    unsigned int last_drone_num = 0;

    std::map<unsigned int, unsigned int> node_kf_count;

    std::map<int, std::set<int>> loop_edges;
    int good_loop_num = 0;
    int good_dets = 0;

    bool has_new_keyframe = false;

    bool enable_cgraph_generation;

    void delete_frame_i(int i);

    bool is_frame_useful(unsigned int i) const;

    void process_frame_clear();

    void random_init_pose(EstimatePoses &swarm_est_poses, std::set<int> ids_to_init);
    void init_pose_by_loops(EstimatePoses &swarm_est_poses, std::set<int> ids_to_init);
    void init_pose_by_loop(EstimatePoses &swarm_est_poses, int _id, int id_estimated, Swarm::LoopEdge loc);

    void init_dynamic_nf_in_keyframe(TsType ts, NodeFrame &_nf);

    void init_static_nf_in_keyframe(TsType ts, const NodeFrame &_nf);

    void sync_est_poses(const EstimatePoses &_est_poses_tsid, bool is_init_solve);


    std::vector<Swarm::GeneralMeasurement2Drones*> find_available_loops_detections(std::map<int, std::set<int>> & loop_edges);

    bool find_node_frame_for_measurement_2drones(const Swarm::GeneralMeasurement2Drones * loc, int & _index_a, int &_index_b, double & dt_err) const;

    int loop_from_src_loop_connection(const Swarm::LoopEdge & _loc, Swarm::LoopEdge & loc_ret, double & dt_err, double & dpos) const;

    CostFunction *
    _setup_cost_function_by_sf(const SwarmFrame &sf, std::map<int, int> id2poseindex, bool is_lastest_frame, int & res_num) const;


    void
    setup_problem_with_sferror(const EstimatePoses &swarm_est_poses, Problem &problem, const SwarmFrame &sf, TSIDArray & param_indexs, bool is_lastest_frame);

    void setup_problem_with_ego_motion(const EstimatePosesIDTS & est_poses_idts, Problem &problem, int _id) const;
    
    void setup_problem_with_loops_and_detections(const EstimatePosesIDTS & est_poses_idts, Problem &problem) const;
    
    
    void cutting_edges();

    double solve_once(EstimatePoses &swarm_est_poses, EstimatePosesIDTS &est_poses_idts, bool report = false);
    
    int judge_is_key_frame(const SwarmFrame &sf);

    void add_as_keyframe(SwarmFrame sf);
    void outlier_rejection_frame(SwarmFrame & sf) const;
    void print_frame(const SwarmFrame & sf) const;
    void replace_last_kf(const SwarmFrame & sf);
    
    bool solve_with_multiple_init(int max_number, std::set<int> new_init_ids);
    
    std::pair<bool, Swarm::Pose> get_estimated_pose(int _int, TsType ts) const;

    inline unsigned int sliding_window_size() const;
    bool NFnotMoving(const NodeFrame & _nf1, const NodeFrame & nf2) const;

    std::pair<Eigen::Vector3d, Eigen::Vector3d> boundingbox_sldwin(int _id) const;

    std::set<int> estimate_observability();
    std::set<int> loop_observable_set(const std::map<int, std::set<int>> & loop_edges) const;

    void generate_cgraph();

    bool generate_full_path = false;

    SwarmLocalOutlierRejection * outlier_rejection = nullptr;
public:
    int self_id = -1;
    unsigned int thread_num;
    double cost_now = 0;
    double acpt_cost = 0.4;
    double min_accept_keyframe_movement = 0.2;

    bool finish_init = false;

    bool enable_to_solve_master = false;
    bool first_init = true;
    std::map<int, bool> enable_to_init_by_drone;
    ros::Time last_est_time_tick = ros::Time::now();
    float init_xy_movement = 2.0;
    float init_z_movement = 1.0;

    bool enable_detection;
    bool enable_loop;
    bool enable_distance;
    bool enable_detection_depth;

    bool kf_use_all_nodes;
    bool system_is_initied_by_motion = false;

    std::map <int, bool> yaw_observability;
    std::map <int, bool> pos_observability;
    std::map<int, int> anyoumos_det_mapper;

    std::map<int, Swarm::DroneTrajectory> keyframe_trajs;
    std::map<int, Swarm::DroneTrajectory> full_trajs;
    std::map<int, Swarm::DroneTrajectory> ego_motion_trajs;

    std::string cgraph_path = "";

    SwarmLocalizationSolver(const swarm_localization_solver_params & params);
    
    void add_new_swarm_frame(const SwarmFrame &sf);

    void add_new_loop_connection(const swarm_msgs::LoopEdge & loop_con);

    void add_new_detection(const swarm_msgs::node_detected_xyzyaw & detected);
    void add_new_detection(const swarm_msgs::node_detected & detected);

    SwarmFrameState PredictSwarm(const SwarmFrame &sf) const;

    bool PredictNode(const NodeFrame & nf, Pose & _pose, Eigen::Matrix4d & cov) const;
    bool NodeCooridnateOffset(int _id, Pose & _pose, Eigen::Matrix4d & cov) const;
    bool CanPredictSwarm() {
        return finish_init;
    }

    const std::vector<int64_t> get_good_loops() const { 
        return outlier_rejection->good_loops();
    };


    double count_opti_time = 0;
    double sum_opti_time = 0;

    double count_solve_time = 0;
    double sum_solve_time = 0;

    double count_outlier_rejection_time  = 0;
    double sum_outlier_rejection_time  = 0;
    std::vector<Swarm::GeneralMeasurement2Drones*> good_2drone_measurements;


    double solve();


    
};