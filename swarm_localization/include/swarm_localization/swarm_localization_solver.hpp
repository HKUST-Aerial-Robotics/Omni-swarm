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


Swarm::Pose Predict_By_VO(Swarm::Pose vo_now, Swarm::Pose vo_ref, Swarm::Pose est_pose_ref, bool is_yaw_only = true);


//Poses is dict of timestamp and then id;
//state<ts,id>
typedef std::map<TsType, std::map<int,double*>> EstimatePoses;
typedef std::map<TsType, std::map<int, Eigen::Matrix4d>> EstimateCOV;
typedef std::map<int, std::map<TsType,double*>> EstimatePosesIDTS;
typedef std::vector<std::pair<TsType, int>> TSIDArray;
typedef std::map<int, std::map<TsType, int>>  IDTSIndex;


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
    float loop_outlier_threshold_distance = 2.0;
    float loop_outlier_threshold_distance_init = 2.0;
    float det_dpos_thres = 1.0;
    float detection_outlier_thres;
    float detection_inv_dep_outlier_thres;
    bool enable_detection;
    bool enable_loop;
    bool enable_distance;
    bool enable_detection_depth;
    bool kf_use_all_nodes;
    bool generate_full_path;
    float max_solver_time;
    float distance_outlier_threshold;
    float distance_height_outlier_threshold;


    float VO_METER_STD_TRANSLATION = 0.01;
    float VO_METER_STD_ANGLE = 0.01;
    float DISTANCE_STD = 0.1;
    SwarmLocalOutlierRejectionParams outlier_rejection_params;
};

class SwarmLocalizationSolver {

    std::mutex solve_lock;
    std::mutex predict_lock;
    std::vector<SwarmFrame> sf_sld_win;
    std::map<TsType, SwarmFrame> all_sf;
    TsType last_kf_ts = 0;
    std::vector<TsType> last_saved_est_kf_ts;
    unsigned int drone_num = 0;

    unsigned int solve_count = 0;


    swarm_localization_solver_params params;

    int detection_in_keyframes = 0;
    std::vector<swarm_msgs::LoopEdge> all_loops;
    std::vector<swarm_msgs::node_detected_xyzyaw> all_detections;

    EstimatePoses est_poses_tsid, est_poses_tsid_saved;
    EstimatePosesIDTS est_poses_idts, est_poses_idts_saved;
    EstimateCOV est_cov_tsid;

    unsigned int max_frame_number = 100;
    unsigned int min_frame_number = 5;
    unsigned int dense_frame_number = 20;
    
    float max_solver_time;

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

    void init_dynamic_nf_in_keyframe(TsType ts, NodeFrame &_nf);

    void init_static_nf_in_keyframe(TsType ts, const NodeFrame &_nf);

    void sync_est_poses(const EstimatePoses &_est_poses_tsid, bool is_init_solve);


    std::vector<Swarm::GeneralMeasurement2Drones*> find_available_loops_detections(std::map<int, std::set<int>> & loop_edges);

    bool find_node_frame_for_measurement_2drones(const Swarm::GeneralMeasurement2Drones * loc, int & _index_a, int &_index_b, double & dt_err) const;

    int loop_from_src_loop_connection(const swarm_msgs::LoopEdge & _loc, Swarm::LoopEdge & loc_ret, double & dt_err, double & dpos) const;

    bool detection_from_src_node_detection(const swarm_msgs::node_detected_xyzyaw & _loc, Swarm::DroneDetection & loc_ret, double & dt_err, double & dpos) const;

    bool check_outlier_detection(const NodeFrame & _nf_a, const NodeFrame & _nf_b, const DroneDetection & det_ret) const;

    CostFunction *
    _setup_cost_function_by_sf(const SwarmFrame &sf, std::map<int, int> id2poseindex, bool is_lastest_frame, int & res_num) const;


    int
    setup_problem_with_sferror(const EstimatePoses &swarm_est_poses, Problem &problem, const SwarmFrame &sf, TSIDArray & param_indexs, bool is_lastest_frame) const;

    CostFunction *
    _setup_cost_function_by_nf_win(std::vector<NodeFrame> &nf_win, const std::map<TsType, int> & ts2poseindex, bool is_self) const;
    
    void setup_problem_with_sfherror(const EstimatePosesIDTS & est_poses_idts, Problem &problem, int _id) const;
    
    CostFunction *
    _setup_cost_function_by_loop(const Swarm::GeneralMeasurement2Drones* loops) const;

    void setup_problem_with_loops(const EstimatePosesIDTS & est_poses_idts, Problem &problem) const;
    
    
    void cutting_edges();

    double solve_once(EstimatePoses &swarm_est_poses, EstimatePosesIDTS &est_poses_idts, bool report = false);
    
    int judge_is_key_frame(const SwarmFrame &sf);

    void add_as_keyframe(SwarmFrame sf);
    void outlier_rejection_frame(SwarmFrame & sf) const;
    void print_frame(const SwarmFrame & sf) const;
    void replace_last_kf(const SwarmFrame & sf);
    
    bool solve_with_multiple_init(int max_number = 10);
    
    std::pair<bool, Swarm::Pose> get_estimated_pose(int _int, TsType ts) const;

    inline unsigned int sliding_window_size() const;
    bool NFnotMoving(const NodeFrame & _nf1, const NodeFrame & nf2) const;

    std::pair<Eigen::Vector3d, Eigen::Vector3d> boundingbox_sldwin(int _id) const;

    void estimate_observability();
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

    bool enable_to_init = false;
    ros::Time last_est_time_tick = ros::Time::now();
    float init_xy_movement = 2.0;
    float init_z_movement = 1.0;
    float loop_outlier_threshold_pos = 1.0;
    float loop_outlier_threshold_yaw = 1.0;
    float detection_inv_dep_outlier_thres;
    float distance_outlier_threshold;
    float distance_height_outlier_threshold;
    float detection_outlier_thres;
    float loop_outlier_threshold_distance;
    float loop_outlier_threshold_distance_init;

    float det_dpos_thres;

    bool enable_detection;
    bool enable_loop;
    bool enable_distance;
    bool enable_detection_depth;

    bool kf_use_all_nodes;

    std::map <int, bool> yaw_observability;
    std::map <int, bool> pos_observability;

    std::map<int, Swarm::DroneTrajectory> keyframe_trajs;
    std::map<int, Swarm::DroneTrajectory> full_trajs;
    std::map<int, Swarm::DroneTrajectory> ego_motion_trajs;

    std::string cgraph_path = "";

    SwarmLocalizationSolver(const swarm_localization_solver_params & params);
    
    void add_new_swarm_frame(const SwarmFrame &sf);

    void add_new_loop_connection(const swarm_msgs::LoopEdge & loop_con);

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