#pragma once
#include <eigen3/Eigen/Dense>

extern float distance_measurement_cov;

extern float DETECTION_SPHERE_STD;
extern float DETECTION_INV_DEP_STD;
extern float DETECTION_DEP_STD;
extern Eigen::Vector3d CG;

struct swarm_localization_solver_params {
    int max_frame_number = 100;
    int min_frame_number = 5;
    int dense_frame_number = 20;
    float acpt_cost = 10;
    int thread_num = 1;
    float kf_movement = 0.2;
    float kf_time_with_half_movement = 1.0;
    float init_xy_movement = 2.0;
    float init_z_movement = 1.0;
    int self_id = -1;
    std::string cgraph_path;
    float DA_accept_thres = 3.345;
    bool enable_cgraph_generation = false;
    float loop_outlier_distance_threshold = 2.0;
    float det_dpos_thres = 1.0;

    bool enable_detection;
    bool enable_loop;
    bool enable_distance;
    bool enable_detection_depth;
    bool kf_use_all_nodes;
    bool generate_full_path;
    float max_solver_time;
    float distance_measurement_outlier_threshold;
    float distance_measurement_outlier_elevation_threshold;
    float minimum_distance = 0.2;
    bool debug_no_rejection = false;


    float vo_cov_pos_per_meter = 0.1;
    float vo_cov_yaw_per_meter = 0.1;
    float distance_measurement_cov = 0.1;
    bool enable_random_keyframe_deletetion = false;
    SwarmLocalOutlierRejectionParams outlier_rejection_params;

    //Debug 
    bool debug_loop_initial_only = false;
    bool debug_no_relocalization = false;
    bool enable_data_association = true;
};

