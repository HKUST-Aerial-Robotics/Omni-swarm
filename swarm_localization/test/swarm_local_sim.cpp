#include <swarm_msgs/swarm_types.hpp>
#include <ros/ros.h>
#include <swarm_msgs/node_detected.h>
#include <swarm_msgs/swarm_frame.h>
#include <swarm_msgs/node_frame.h>
#include <swarm_msgs/swarm_fused.h>
#include "fisheye_undist.hpp"
#include <random>
#include <opencv2/opencv.hpp>
#include "opencv2/core/eigen.hpp"
#include <faiss/IndexFlat.h>
#include <geometry_msgs/PoseStamped.h>

std::random_device rd{};
std::default_random_engine eng{0};
std::normal_distribution<double> d{0,1};

#define MAX_DETECTOR_ID 1000000
#define MAX_LOOP_ID 100000000
#define MAX_DRONE_ID 1000

#define VCAMERA_TOP 0
#define VCAMERA_LEFT 1
#define VCAMERA_FRONT 2
#define VCAMERA_RIGHT 3
#define VCAMERA_REAR 4
#define SEARCH_NEAREST_NUM 5
#define MATCH_INDEX_DIST 10

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
}

Swarm::Pose random_pose_4dof(double cov_pos, double cov_yaw) {
    double std_pos = sqrt(cov_pos);
    double std_yaw = sqrt(cov_yaw);
    Eigen::Vector3d noise_pos(d(eng)*std_pos, d(eng)*std_pos, d(eng)*std_pos);
    return Swarm::Pose(noise_pos, d(eng)*std_yaw);
}

double overlap(const cv::Rect2d bbox, const cv::Rect2d bbox2) {
        double XA1 = bbox.x;
        double XA2 = bbox.x + bbox.width;
        double XB1 = bbox2.x;
        double XB2 = bbox2.x + bbox2.width;

        double YA1 = bbox.y;
        double YA2 = bbox.y + bbox.height;
        double YB1 = bbox2.y;
        double YB2 = bbox2.y + bbox2.height;

        double SA = bbox.area();
        double SB = bbox2.area();

        double SI = std::max(0.0, std::min(XA2, XB2) - std::max(XA1, XB1)) * std::max(0.0, std::min(YA2, YB2) - std::max(YA1, YB1));
        double SU = SA + SB - SI;
        return SI / std::max(SA, SB);
    }

class SwarmLocalSim {
    int drone_num = 0;
    enum { 
        PARALLEL = 0,
        RANDOM_3D = 1
    } task;
    ros::Publisher swarm_frame_pub, swarm_frame_predict_pub, node_detected_pub, loop_edge_pub;
    ros::Subscriber swarm_fused_sub;
    std::vector<Swarm::DroneTrajectory> ground_truth_trajs, ego_motion_trajs, est_trajs;

    double uwb_cov = 0.01;
    double vo_cov_pos_per_meter = 0.0001;
    double vo_cov_yaw_per_meter = 0.0001;
    double loop_cov_pos = 0.01;
    double loop_cov_ang = 0.01;
    double det_cov_pos = 0.0001;
    double det_cov_ang = 0.0001;
    double loop_cov_pos_label = 0.01;
    double loop_cov_ang_label = 0.01;
    double det_cov_pos_label = 0.0001;
    double det_cov_ang_label = 0.0001;

    int img_width = 600;
    double det_cov_len = 0;
    
    double min_distance_det = 0.3;
    double max_distance_det = 2.0;
    double max_t = 100;

    double loop_max_distance = 2.0;

    int main_id = 0;
    int loop_count = 0;
    
    enum { 
        ParalCirc = 0,
        ParalCircRand = 1,
        RandWalk = 2
    };

    double circle_radius = 20;
    double circle_radius_z = 3;
    double circle_T = 50;

    int mission_type = ParalCirc; // Draw circle,
    int initial_setup = 0; // Parallel on y
    int initial_pattern = 0;

    ros::Time last_send_det;
    ros::Time last_send_loop;

    int detect_no = 0;
    std::vector<Vector3d> boundbox3d_corners;
    swarm_detector_pkg::FisheyeUndist* fisheye = nullptr;
    std::vector<Swarm::Pose> vcam_poses;
    bool det_only_front = false;

    faiss::IndexFlatL2 poses_index;
    std::map<int, std::pair<int, int>> poses_index_map;

    ros::Time t0;

    double initial_dis = 1.0;

    ros::Timer timer;

    std::vector<ros::Publisher> pose_gt_pubs;
    std::vector<double> t0_diff;
    std::vector<double> T_diff;
    std::vector<double> R_diff;
    std::vector<double> R_diff_z;

    double initial_t = 10;
    double kf_dis_thres = 0.5;

    int col_width = 1;

public:
    SwarmLocalSim(ros::NodeHandle & nh):
        poses_index(3), last_send_det(0), last_send_loop(0){
        nh.param<int>("drone_num", drone_num, 1);
        nh.param<int>("main_id", main_id, 1);
        std::string extrinsic_path, camera_config_file;

        nh.param<int>("mission_type", mission_type, 0);
        nh.param<int>("initial_pattern", initial_pattern, 0);
        nh.param<double>("distance_measurement_cov", uwb_cov, 0);
        nh.param<double>("vo_cov_pos_per_meter", vo_cov_pos_per_meter, 0);
        nh.param<double>("vo_cov_yaw_per_meter", vo_cov_yaw_per_meter, 0);
        nh.param<double>("loop_cov_pos", loop_cov_pos, 0);
        nh.param<double>("loop_cov_ang", loop_cov_ang, 0);
        nh.param<double>("det_cov_pos", det_cov_pos, 0);
        nh.param<double>("det_cov_ang", det_cov_ang, 0);
        nh.param<double>("det_cov_len", det_cov_len, 0);

        nh.param<double>("loop_cov_pos_label", loop_cov_pos_label, 0);
        nh.param<double>("loop_cov_ang_label", loop_cov_ang_label, 0);
        nh.param<double>("det_cov_pos_label", det_cov_pos_label, 0);
        nh.param<double>("det_cov_ang_label", det_cov_ang_label, 0);

        nh.param<bool>("only_front", det_only_front, false);
        nh.param<double>("loop_max_distance", loop_max_distance, 2.0);
        nh.param<double>("det_max_distance", max_distance_det, 2.0);
        nh.param<double>("circle_radius", circle_radius, 20);
        nh.param<double>("circle_radius_z", circle_radius_z, 5);
        nh.param<double>("circle_T", circle_T, 50);
        nh.param<double>("initial_dis", initial_dis, 1.0);
        nh.param<double>("initial_t", initial_t, 10);
        nh.param<double>("max_t", max_t, 10);
        nh.param<double>("min_distance_det", min_distance_det, 0.2);
        nh.param<std::string>("extrinsic_path", extrinsic_path, "");
        nh.param<std::string>("cam_file", camera_config_file, "");
        fisheye = new swarm_detector_pkg::FisheyeUndist(camera_config_file, 235, true, img_width);
    
        ground_truth_trajs.resize(drone_num);
        ego_motion_trajs.resize(drone_num);
        est_trajs.resize(drone_num);
        swarm_frame_pub = nh.advertise<swarm_msgs::swarm_frame>("/swarm_drones/swarm_frame", 10);
        swarm_frame_predict_pub = nh.advertise<swarm_msgs::swarm_frame>("/swarm_drones/swarm_frame_predict", 10);
        node_detected_pub = nh.advertise<swarm_msgs::node_detected>("/swarm_drones/node_detected_6d", 10);
        loop_edge_pub = nh.advertise<swarm_msgs::LoopEdge>("/swarm_loop/loop_connection", 10);
        swarm_fused_sub = nh.subscribe("/swarm_drones/swarm_drone_fused_pc", 3, &SwarmLocalSim::swarm_fused_callback, this);

        for (int i = 0; i < drone_num; i++) {
            char topic[1024] = {0};
            sprintf(topic, "/SwarmNode%d/pose", i);
            pose_gt_pubs.push_back(nh.advertise<geometry_msgs::PoseStamped>(topic, 10));

            T_diff.push_back(d(eng)*0.5);
            t0_diff.push_back(d(eng)*0.2);
            R_diff.push_back(d(eng)*0.1*circle_radius);
            R_diff_z.push_back(d(eng)*0.1*circle_radius_z);
        }

        double w_2 = 0.2;
        double h_2 = 0.2;
        double w_g_2 = 0.15;
        double h_g_2 = 0.15;
        double z_max = 0.115;
        double z_min = -0.071;
        double z_mid = 0.05;

        col_width = drone_num;
        if (initial_pattern == 1) {
            col_width = ceil(sqrtf(drone_num));
        }

        if (col_width < 2) {
            col_width = 2;
        }

        ROS_INFO("Simulator grid col width %d", col_width);

    
        Vector3d Gc_imu = Vector3d(-0.06, 0, 0.00);

        boundbox3d_corners = std::vector<Vector3d>{
            Vector3d(w_2, h_2, z_mid) + Gc_imu,
            Vector3d(w_2, -h_2, z_mid) + Gc_imu,
            Vector3d(-w_2, h_2, z_mid) + Gc_imu,
            Vector3d(-w_2, -h_2, z_mid) + Gc_imu,
            Vector3d(w_g_2, h_g_2, z_min) + Gc_imu,
            Vector3d(w_g_2, -h_g_2, z_min) + Gc_imu,
            Vector3d(-w_g_2, h_g_2, z_min) + Gc_imu,
            Vector3d(-w_g_2, -h_g_2, z_min) + Gc_imu,
            Vector3d(0, 0, z_max) + Gc_imu,
        };

        init_cameras(extrinsic_path, camera_config_file);

        t0 = ros::Time::now();
        ROS_INFO("[SWARM_SIM] Sim start.");

        timer = nh.createTimer(ros::Duration(0.01), &SwarmLocalSim::timer_callback, this);

       
    }


    void swarm_fused_callback(const swarm_msgs::swarm_fused & sf) {
        for (size_t i = 0; i < sf.ids.size(); i ++) {
            est_trajs[sf.ids[i]].push(sf.header.stamp, Swarm::Pose(sf.local_drone_position[i], sf.local_drone_rotation[i]));
        }
    }
    
    void init_cameras(const std::string & extrinsic_path, const std::string & camera_config_file) { 
        ROS_INFO("[SWARM_SIM] Try to read extrinsic from %s camera from %s", extrinsic_path.c_str(), camera_config_file.c_str());
        FILE *fh = fopen(extrinsic_path.c_str(), "r");

        Eigen::Matrix3d Rcam;
        Eigen::Vector3d Pcam;
        cv::FileStorage fsSettings(extrinsic_path, cv::FileStorage::READ);
        cv::Mat _T;
        fsSettings["body_T_cam0"] >> _T;
        fsSettings["image_width"] >> img_width;

        Eigen::Matrix4d T;
        cv::cv2eigen(_T, T);
        Rcam = T.block<3, 3>(0, 0);
        Pcam = T.block<3, 1>(0, 3);
        auto extrinsic = Swarm::Pose(Rcam, Pcam);

        fsSettings.release();

        ROS_INFO("[SWARM_SIM] Camera width %d, Pose", img_width);
        std::cout << "R" << Rcam << std::endl;
        std::cout << "P" << Pcam.transpose() << std::endl;

        std::vector<Eigen::Quaterniond> Rvcams;
        Rvcams.push_back(Eigen::Quaterniond::Identity());                                             //0 top (up half)
        Rvcams.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(1, 0, 0)))); //1 left
        Rvcams.push_back(Rvcams.back() * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 1, 0)));      //2 front
        Rvcams.push_back(Rvcams.back() * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 1, 0)));      //3 right
        Rvcams.push_back(Rvcams.back() * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 1, 0)));      //4 rear

        vcam_poses.emplace_back(Swarm::Pose(Rcam, Pcam));

        for (size_t i = 1; i < Rvcams.size(); i ++) {
            vcam_poses.emplace_back(Swarm::Pose(Rcam*Rvcams[i], Pcam));
        }
    }


    std::pair<bool, Eigen::Vector2d> reproject_point_to_vcam(int direction, Eigen::Vector3d corner, Swarm::Pose est, Swarm::Pose cur) const {
        auto cam = fisheye->cam_side;
        if (direction == 0) {
            cam = fisheye->cam_top;
        }
        Eigen::Vector2d ret(0, 0);

        auto corner3d_body = est * corner;
        corner3d_body = cur.apply_inv_pose_to(corner3d_body);
        if (corner3d_body.z() < 0) {
            return std::make_pair(false, ret);
        }

        cam->spaceToPlane(corner3d_body, ret);
        return std::make_pair(true, ret);
    }

    std::pair<bool, cv::Rect2d> reproject_drone_to_vcam(int direction, Swarm::Pose est, Swarm::Pose cur, const std::vector<Vector3d> & corners) const {
        cv::Rect2d reproject_bbox;
        MatrixXd corners2d_body(2, corners.size());
        auto cam = fisheye->cam_side;
        if (direction == 0) {
            cam = fisheye->cam_top;
        }

        // std::cout << "VCam\t" << direction << "\test pose\t" << est.tostr() << "\tcam pose\t" << cur.tostr()  << std::endl;
        for (size_t i = 0; i < corners.size(); i ++) {
            auto corner = corners[i];
            auto corner3d_body = est * corner;
            corner3d_body = cur.apply_inv_pose_to(corner3d_body);

            if (corner3d_body.z() < 0) {
                return std::make_pair(false, reproject_bbox);
            }

            Vector2d corner2d;
            cam->spaceToPlane(corner3d_body, corner2d);
            corners2d_body.block(0, i, 2, 1) = corner2d;
            // std::cout << "corner3d\t" << (est * corner).transpose() << "body\t" << corner3d_body.transpose() << "2d" << corners2d_body.transpose() << std::endl;
        }


        auto xs = corners2d_body.block(0, 0, 1, corners2d_body.cols());
        auto ys = corners2d_body.block(1, 0, 1, corners2d_body.cols());

        reproject_bbox.x = xs.minCoeff();
        reproject_bbox.width = xs.maxCoeff() - reproject_bbox.x;
        reproject_bbox.y = ys.minCoeff();
        reproject_bbox.height = ys.maxCoeff() - reproject_bbox.y;

        // std::cout << "corner3d_body\n" << corners2d_body << std::endl;
        // std::cout << "corner3d_body_col_xs\n" << xs << std::endl;
        // std::cout << "corner3d_body_col_ys\n" << ys << std::endl;
        // std::cout << "bbox\n" << reproject_bbox << std::endl;

        if (reproject_bbox.x < 0 || reproject_bbox.x + reproject_bbox.width  > cam->imageWidth() || 
                reproject_bbox.y  < 0 || reproject_bbox.y + reproject_bbox.height > cam->imageWidth()) {
            return std::make_pair(false, reproject_bbox);
        }

        return std::make_pair(true, reproject_bbox);
    }

    double generate_distance_measurement(int i, int j) const {
        Swarm::Pose posea = ground_truth_trajs[i].get_latest_pose();
        Swarm::Pose poseb = ground_truth_trajs[j].get_latest_pose();
        return Swarm::Pose::DeltaPose(posea, poseb).pos().norm() + d(eng)*sqrt(uwb_cov);
    }


    swarm_msgs::node_detected generate_det_measurement(ros::Time stamp, int i, int j) {
        Swarm::Pose posea = ground_truth_trajs[i].get_latest_pose();
        Swarm::Pose poseb = ground_truth_trajs[j].get_latest_pose();
        Swarm::Pose relative_pose = Swarm::Pose::DeltaPose(posea, poseb, true) * random_pose_4dof(det_cov_pos, det_cov_ang);
        relative_pose.pos() *= (1 + d(eng)*sqrt(det_cov_len));
        swarm_msgs::node_detected nd;
        nd.local_pose_self = ego_motion_trajs[i].get_latest_pose().to_ros_pose();
        nd.is_yaw_valid = true;
        nd.self_drone_id = i;
        nd.remote_drone_id = j;
        nd.header.stamp = stamp;
        nd.probaility = 1.0;
        nd.dof_4 = true;
        nd.id = MAX_DETECTOR_ID*i + detect_no;
        Eigen::Matrix6d cov;
        cov.setZero();
        cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * det_cov_pos_label;
        cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * det_cov_ang_label;
        nd.relative_pose.pose = relative_pose.to_ros_pose();
        memcpy(nd.relative_pose.covariance.data(), cov.data(), sizeof(double)*36);
        detect_no++;
        return nd;
    }

    int is_detect_valid(Swarm::Pose rel_pose_gt, int i, int j, int direction) {
        //Now we reproject_bbox
        auto ret = reproject_drone_to_vcam(direction, rel_pose_gt, vcam_poses[direction], boundbox3d_corners);
        if (!ret.first) {
            return 0;
        }

        if (est_trajs[i].trajectory_size() == 0 || est_trajs[j].trajectory_size() == 0) {
            return 1;
        }

        auto ret2 = reproject_drone_to_vcam(direction, est_trajs[j].get_latest_pose(), 
                est_trajs[i].get_latest_pose()*vcam_poses[direction], boundbox3d_corners);
        
        if (!ret2.first) {
            return 1;
        }

        if (overlap(ret.second, ret2.second) > 0.05) {
            //This matched to the est, so it has id.
            return 2;
        }
        return 1;
    }


    void generate_pub_det_measurement(ros::Time stamp) {
        for (int i = 0; i < drone_num; i++) {
            Swarm::Pose posea = ground_truth_trajs[i].get_latest_pose();
            for (int j = 0; j < drone_num; j++) {
                if (i == j) {
                    continue;
                }
                Swarm::Pose poseb = ground_truth_trajs[j].get_latest_pose();
                Swarm::Pose rel_pose = Swarm::Pose::DeltaPose(posea, poseb);
                double distance = rel_pose.pos().norm();
                if (distance > max_distance_det || distance < min_distance_det) {
                    continue;
                }

                for (int d = VCAMERA_LEFT; d < vcam_poses.size(); d++) {
                    if (det_only_front && d!=VCAMERA_FRONT) {
                        continue;
                    }

                    int valid = is_detect_valid(rel_pose, i, j, d);
                    if (valid > 0) {
                        ROS_INFO("Vaild %d detect %d->%d RP %s", valid, i, j, rel_pose.tostr().c_str());
                        auto det = generate_det_measurement(stamp, i, j);
                        if (valid == 1) {
                            det.remote_drone_id = i*MAX_DRONE_ID + j;
                        }
                        node_detected_pub.publish(det);
                        continue;
                    }
                }
            }
        }
    }

    void publish_loop_edge(ros::Time ta, ros::Time tb, int ida, int idb, 
                int kf_ida, int kf_idb, Swarm::Pose ego_posea, Swarm::Pose ego_poseb,
                Swarm::Pose rp_gt) {
        swarm_msgs::LoopEdge ret;
        Swarm::Pose rel_pose = rp_gt * random_pose_4dof(loop_cov_pos, loop_cov_ang);
        ret.relative_pose = rel_pose.to_ros_pose();

        ret.drone_id_a = ida;
        ret.ts_a = ta;

        ret.drone_id_b = idb;
        ret.ts_b = tb;

        ret.self_pose_a = ego_posea.to_ros_pose();
        ret.self_pose_b = ego_poseb.to_ros_pose();

        ret.keyframe_id_a = kf_ida;
        ret.keyframe_id_b = kf_idb;

        ret.pos_cov.x = loop_cov_pos_label;
        ret.pos_cov.y = loop_cov_pos_label;
        ret.pos_cov.z = loop_cov_pos_label;

        ret.ang_cov.x = loop_cov_ang_label;
        ret.ang_cov.y = loop_cov_ang_label;
        ret.ang_cov.z = loop_cov_ang_label;

        ret.pnp_inlier_num = 100;
        ret.id = ida*MAX_LOOP_ID + loop_count;
        loop_count++;

        loop_edge_pub.publish(ret);
    }

    void generate_loop_measurment(ros::Time stamp, int i) {
        Swarm::Pose posei_gt = ground_truth_trajs[i].get_latest_pose();
        Swarm::Pose posei_ego = ego_motion_trajs[i].get_latest_pose();

        Eigen::Vector3f pos_gt_f = posei_gt.pos().template cast<float>();

        //First we found the nearest poses
        int search_num = SEARCH_NEAREST_NUM + MATCH_INDEX_DIST + drone_num;
        float distances[1024] = {0};
        faiss::Index::idx_t labels[1024];
        poses_index.search(1, pos_gt_f.data(), search_num, distances, labels);
        
        for (int j = 0; j < search_num; j++) {
            if (labels[j] < 0) {
                continue;
            }

            int count = 0;

            if ( (labels[j] <= poses_index.ntotal - MATCH_INDEX_DIST - drone_num || labels[j]!= i)  && distances[j] < loop_max_distance) {
                auto ret = poses_index_map.at(labels[j]);
                auto idj = ret.first;
                auto indexj = ret.second;
                auto ts = ground_truth_trajs[idj].get_ts(indexj);
                auto posej = ground_truth_trajs[idj].get_pose(indexj);

                Swarm::Pose ego_poseb = ego_motion_trajs[idj].get_pose(indexj);
                ros::Time timej;
                timej.fromNSec(ts);

                Swarm::Pose rp_gt = Swarm::Pose::DeltaPose(posei_gt, posej, true);
                ROS_INFO("[LOCAL_SIM] Loop generated %d->%d dt %f distance %f relpose_gt %s",
                    i, idj, (stamp - timej).toSec(), distances[j], rp_gt.tostr().c_str());
                publish_loop_edge(stamp, timej, i, idj, poses_index.ntotal, indexj, posei_ego, ego_poseb, rp_gt);
                // break;    
                count ++;
                if (count >= 2) {
                    break;
                }
            }
        }
        
        if (ground_truth_trajs[i].trajectory_size() == 1)
        {
            poses_index.add(1, pos_gt_f.data());
            poses_index_map[poses_index.ntotal - 1] = std::make_pair(i, ground_truth_trajs[i].trajectory_size() - 1);
            return;
        }

        Swarm::Pose posei_gt_last = ground_truth_trajs[i].get_pose(ground_truth_trajs[i].trajectory_size()-2);

        if (Swarm::Pose::DeltaPose(posei_gt, posei_gt_last).pos().norm() > kf_dis_thres) {
            poses_index.add(1, pos_gt_f.data());
            poses_index_map[poses_index.ntotal - 1] = std::make_pair(i, ground_truth_trajs[i].trajectory_size() - 1);
        }
    }


    void generate_egomotion(int i) {
        Swarm::Pose posei_gt_latest = ground_truth_trajs[i].get_latest_pose();
        TsType ts = ground_truth_trajs[i].get_ts(ground_truth_trajs[i].trajectory_size() - 1);
        ros::Time stamp;
        stamp.fromNSec(ts);
            
        if (ground_truth_trajs[i].trajectory_size() == 1) {
            Swarm::Pose pose_i_vo;
            ego_motion_trajs[i].push(stamp, pose_i_vo);
            return;
        }
        Swarm::Pose posei_gt_last = ground_truth_trajs[i].get_pose(ground_truth_trajs[i].trajectory_size() - 2);
        Swarm::Pose rel_pose_gt = Swarm::Pose::DeltaPose(posei_gt_last, posei_gt_latest, true);
        Swarm::Pose rel_pose_noise = rel_pose_gt * random_pose_4dof(vo_cov_pos_per_meter*rel_pose_gt.pos().norm(), 
                vo_cov_yaw_per_meter*rel_pose_gt.pos().norm());
        Swarm::Pose posei_vo_last = ego_motion_trajs[i].get_latest_pose();
        Swarm::Pose posei_vo_latest = posei_vo_last * rel_pose_noise;
        ego_motion_trajs[i].push(stamp, posei_vo_latest);
    }

    swarm_msgs::swarm_frame construct_sf(ros::Time stamp, int kf_id) const {
        swarm_msgs::swarm_frame sf;
        sf.header.stamp = stamp;
        for (int i = 0; i < drone_num; i++) {
            Swarm::Pose ego_pose = ego_motion_trajs[i].get_latest_pose();
            swarm_msgs::node_frame nf;
            nf.header.stamp = stamp;
            nf.drone_id = i;
            nf.keyframe_id = kf_id*100 + i;
            nf.position.x = ego_pose.pos().x();
            nf.position.y = ego_pose.pos().y();
            nf.position.z = ego_pose.pos().z();
            nf.quat.w = ego_pose.att().w();
            nf.quat.x = ego_pose.att().x();
            nf.quat.y = ego_pose.att().y();
            nf.quat.z = ego_pose.att().z();
            nf.yaw = ego_pose.yaw();
            nf.pitch = 0;
            nf.roll = 0;
            nf.vo_available = true;
            for (int j = 0; j < drone_num; j ++) {
                if (i != j) {
                    nf.dismap_ids.push_back(j);
                    nf.dismap_dists.push_back(generate_distance_measurement(i, j));
                }
            }

            sf.node_frames.push_back(nf);
        }

        sf.self_id = 0;
        return sf;
    }

    void update_motions(ros::Time stamp, int i) {
        double _t = stamp.toSec() - t0.toSec();
        double t = _t - initial_t;
        if (mission_type == ParalCirc) {
            if (_t > initial_t) {
                double x = circle_radius * sin(2*M_PI*t/circle_T)  + (i / col_width)* initial_dis;
                double y = circle_radius * (1 - cos(2*M_PI*t/circle_T)) + (i%col_width) * initial_dis;
                double z = circle_radius_z * sin(2*M_PI*t/circle_T);

                Swarm::Pose pose_gt(Eigen::Vector3d(x, y, z), 0);
                ground_truth_trajs[i].push(stamp, pose_gt);
            } else {
                double x = (i / col_width)* initial_dis;
                double y = (i%col_width) * initial_dis;
                double z = 0;
                Swarm::Pose pose_gt(Eigen::Vector3d(x, y, z), 0);
                ground_truth_trajs[i].push(stamp, pose_gt);
            }
            // ROS_INFO("%d GT %s", i, pose_gt.tostr().c_str());
        }

        if (mission_type == ParalCircRand) {
            double _circle_T = circle_T + T_diff[i];
            double r = circle_radius + R_diff[i];
            double rz = circle_radius_z + R_diff_z[i];
            // ROS_INFO("Unsync Traj _circle_T %f t0_diff %f", _circle_T, t0_diff[i]);
            if (_t > initial_t) {
                double phase = (2*M_PI* (t+ t0_diff[i]))/_circle_T ;
                double x = r * sin(phase);
                double y = r * (1 - cos(phase)) + i * initial_dis;
                double z = rz * sin(phase);

                Swarm::Pose pose_gt(Eigen::Vector3d(x, y, z), 0);
                ground_truth_trajs[i].push(stamp, pose_gt);
            } else {
                double phase = (2*M_PI*t0_diff[i])/_circle_T ;
                
                double x = r * sin(phase);
                double y = r * (1 - cos(phase)) + i * initial_dis;
                double z = rz * sin(phase);
                Swarm::Pose pose_gt(Eigen::Vector3d(x, y, z), 0);
                ground_truth_trajs[i].push(stamp, pose_gt);
            }
            // ROS_INFO("%d GT %s", i, pose_gt.tostr().c_str());
        }

        if (t > max_t) {
            exit(0);
        }

        generate_egomotion(i);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = stamp;
        pose_stamped.pose = ground_truth_trajs[i].get_latest_pose().to_ros_pose();
        pose_gt_pubs[i].publish(pose_stamped);
    }

    int timer_count = 0;
    void timer_callback(const ros::TimerEvent & e) {
        ros::Time stamp = ros::Time::now();
        if (timer_count % 100 == 0) {
            ROS_INFO("[LOCAL_SIM] T %.3fs", (stamp-t0).toSec());
        }
        for (int i = 0; i < drone_num; i++) {
            update_motions(stamp, i);
        }

        auto sf = construct_sf(stamp, main_id);
        swarm_frame_pub.publish(sf);
        swarm_frame_predict_pub.publish(sf);

        if (timer_count % 100 == 0) {
            for (int i = 0; i < drone_num; i++) {
                generate_loop_measurment(stamp, i);
            }
        }

        if (timer_count % 100 == 0) {
            generate_pub_det_measurement(stamp);
        }

        timer_count++;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "swarm_local_sim");
    ros::NodeHandle nh("swarm_local_sim");
    SwarmLocalSim sim(nh);
    ros::spin();
}
