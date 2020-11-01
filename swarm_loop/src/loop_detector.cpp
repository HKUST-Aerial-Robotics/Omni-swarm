#include <loop_detector.h>
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <opencv2/opencv.hpp>
#include <chrono> 
#include <opencv2/core/eigen.hpp>

using namespace std::chrono; 

#define ESTIMATE_AFFINE3D

void debug_draw_kpts(const ImageDescriptor_t & img_des, cv::Mat img) {
    auto pts = toCV(img_des.landmarks_2d);
    for (auto pt : pts) {
        cv::circle(img, pt, 1, cv::Scalar(255, 0, 0), -1);
    }
    cv::resize(img, img, cv::Size(), 3.0, 3.0);
    cv::imshow("DEBUG", img);
    cv::waitKey(30);
}

template<typename T>
void reduceVector(std::vector<T> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


void LoopDetector::on_image_recv(const FisheyeFrameDescriptor_t & flatten_desc, std::vector<cv::Mat> imgs) {
    auto start = high_resolution_clock::now();
    assert(flatten_desc.images.size() > 0 && "FlattenDesc must carry more than zero images");

    int drone_id = flatten_desc.drone_id;
    int images_num = flatten_desc.images.size();

    if (imgs.size() < images_num) {
        imgs.resize(images_num);
    }
    
    if (drone_id!= this->self_id && database_size() == 0) {
        ROS_INFO("Empty local database, where giveup remote image");
        return;
    } else {
        ROS_INFO("Detetor start process images from %d with %d images and landmark: %d", drone_id, flatten_desc.images.size(), 
            flatten_desc.landmark_num);
    }

    success_loop_nodes.insert(self_id);
    bool new_node = all_nodes.find(flatten_desc.drone_id) == all_nodes.end();

    all_nodes.insert(flatten_desc.drone_id);

    if (flatten_desc.landmark_num >= MIN_LOOP_NUM) {
        bool init_mode = false;
        if (drone_id != self_id) {
            init_mode = true;
        }

        //Initialize images for visualization
        if (enable_visualize) {
            for (unsigned int i = 0; i < images_num; i++) {
                auto & img_des = flatten_desc.images[i];
                if (imgs[i].empty()) {
                    if (img_des.image.size() != 0) {
                        imgs[i] = decode_image(img_des);
                    } else {
                        imgs[i] = cv::Mat(208, 400, CV_8UC3, cv::Scalar(255, 255, 255));
                    }
                }
            }
        }

        if (!flatten_desc.prevent_adding_db || new_node) {
            add_to_database(flatten_desc);
            msgid2cvimgs[flatten_desc.msg_id] = imgs;
        } else {
            ROS_INFO("This image is prevent to adding to DB");
        }

        bool success = false;

        if (database_size() > MATCH_INDEX_DIST || init_mode || drone_id != self_id) {

            ROS_INFO("Querying image from database size %d init_mode %d nonkeyframe %d", database_size(), init_mode, flatten_desc.prevent_adding_db);
            
            int direction = 1;
            int direction_old = -1;
            FisheyeFrameDescriptor_t & _old_fisheye_img = query_fisheyeframe_from_database(flatten_desc, init_mode, flatten_desc.prevent_adding_db, direction, direction_old);

            auto stop = high_resolution_clock::now(); 

            if (direction_old >= 0 ) {
                LoopConnection ret;

                if (_old_fisheye_img.drone_id == self_id) {
                    success = compute_loop(_old_fisheye_img, flatten_desc, direction_old, direction, msgid2cvimgs[_old_fisheye_img.msg_id],  imgs, ret, init_mode);
                } else {
                    //We grab remote drone from database
                    if (flatten_desc.drone_id == self_id) {
                        success = compute_loop(flatten_desc, _old_fisheye_img, direction, direction_old, imgs, msgid2cvimgs[_old_fisheye_img.msg_id], ret, init_mode);
                    } else {
                        ROS_WARN("Will not compute loop, drone id is %d(self %d)", flatten_desc.drone_id, self_id);
                    }
                }

                if (success) {
                    ROS_INFO("Adding success matched drone %d to database", flatten_desc.drone_id);
                    success_loop_nodes.insert(flatten_desc.drone_id);

                    ROS_INFO("\n Loop Detected %d->%d DPos %4.3f %4.3f %4.3f Dyaw %3.2fdeg. Will publish\n",
                        ret.id_a, ret.id_b,
                        ret.dpos.x, ret.dpos.y, ret.dpos.z,
                        ret.dyaw*57.3
                    );

                    on_loop_connection(ret);
                }
            } else {
                std::cout << "No matched image" << std::endl;
            }      
        } 

        // std::cout << "LOOP Detector cost" << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 <<"ms" << std::endl;
    } else {
        ROS_WARN("Frame contain too less landmark %d, give up", flatten_desc.landmark_num);
    }

}



std::vector<cv::KeyPoint> cvPoints2Keypoints(std::vector<cv::Point2f> pts) {
    std::vector<cv::KeyPoint> cvKpts;

    for (auto pt : pts) {
        cv::KeyPoint kp;
        kp.pt = pt;
        cvKpts.push_back(kp);
    }

    return cvKpts;
}


cv::Mat LoopDetector::decode_image(const ImageDescriptor_t & _img_desc) {
    
    auto start = high_resolution_clock::now();
    // auto ret = cv::imdecode(_img_desc.image, cv::IMREAD_GRAYSCALE);
    auto ret = cv::imdecode(_img_desc.image, cv::IMREAD_COLOR);
    // std::cout << "IMDECODE Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;

    return ret;
}

void PnPInitialFromCamPose(const Swarm::Pose &p, cv::Mat & rvec, cv::Mat & tvec) {
    Eigen::Matrix3d R_w_c = p.att().toRotationMatrix();
    Eigen::Matrix3d R_inital = R_w_c.inverse();
    Eigen::Vector3d T_w_c = p.pos();
    cv::Mat tmp_r;
    Eigen::Vector3d P_inital = -(R_inital * T_w_c);

    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, tvec);
}

Swarm::Pose PnPRestoCamPose(cv::Mat rvec, cv::Mat tvec) {
    cv::Mat r;
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d R_pnp, R_w_c_old;
    cv::cv2eigen(r, R_pnp);
    R_w_c_old = R_pnp.transpose();
    Eigen::Vector3d T_pnp, T_w_c_old;
    cv::cv2eigen(tvec, T_pnp);
    T_w_c_old = R_w_c_old * (-T_pnp);

    return Swarm::Pose(R_w_c_old, T_w_c_old);
}

Swarm::Pose AffineRestoCamPose(Eigen::Matrix4d affine) {
    Eigen::Matrix3d R;
    Eigen::Vector3d T;

    R = affine.block<3, 3>(0, 0);
    T = affine.block<3, 1>(0, 3);
    
    R = (R.normalized()).transpose();
    T = R *(-T);

    std::cout << "R of affine\n" << R << std::endl;
    std::cout << "T of affine\n" << T << std::endl;
    std::cout << "RtR\n" << R.transpose()*R << std::endl;
    return Swarm::Pose(R, T);
}

int LoopDetector::add_to_database(const FisheyeFrameDescriptor_t & new_fisheye_desc) {
    for (size_t i = 0; i < new_fisheye_desc.images.size(); i++) {
        auto & img_desc = new_fisheye_desc.images[i];
        if (img_desc.landmark_num > 0) {
            int index = add_to_database(img_desc);
            imgid2fisheye[index] = new_fisheye_desc.msg_id;
            imgid2dir[index] = i;
            ROS_INFO("Add keyframe from %d(dir %d) to local keyframe database index: %d", img_desc.drone_id, i, index);
        }
    }
    fisheyeframe_database[new_fisheye_desc.msg_id] = new_fisheye_desc;
    return new_fisheye_desc.msg_id;
}

int LoopDetector::add_to_database(const ImageDescriptor_t & new_img_desc) {
    if (new_img_desc.drone_id == self_id) {
        local_index.add(1, new_img_desc.image_desc.data());
        return local_index.ntotal - 1;
    } else {
        remote_index.add(1, new_img_desc.image_desc.data());
        return remote_index.ntotal - 1 + REMOTE_MAGIN_NUMBER;
    }
    return -1;
}


int LoopDetector::query_from_database(const ImageDescriptor_t & img_desc, bool init_mode, bool nonkeyframe, double & distance) {
    double thres = INNER_PRODUCT_THRES;
    if (init_mode) {
        thres = INIT_MODE_PRODUCT_THRES;
    }

    if (img_desc.drone_id == self_id) {
        //Then this is self drone
        int _id = query_from_database(img_desc, remote_index, true, thres, 1, distance);
        if (_id != -1) {
            return _id;
        } else if(!nonkeyframe){
            int _id = query_from_database(img_desc, local_index, false, thres, MATCH_INDEX_DIST, distance);
            return _id;
        }
    } else {
        int _id = query_from_database(img_desc, local_index, false, thres, 1, distance);
        return _id;
    }
    return -1;
}

int LoopDetector::query_from_database(const ImageDescriptor_t & img_desc, faiss::IndexFlatIP & index, bool remote_db, double thres, int max_index, double & distance) {
    float distances[1000] = {0};
    faiss::Index::idx_t labels[1000];

    int index_offset = 0;
    if (remote_db) {
        index_offset = REMOTE_MAGIN_NUMBER;
    }
    
    for (int i = 0; i < 1000; i++) {
        labels[i] = -1;
    }

    int search_num = SEARCH_NEAREST_NUM + max_index;
    index.search(1, img_desc.image_desc.data(), search_num, distances, labels);
    
    for (int i = 0; i < search_num; i++) {
        if (labels[i] < 0) {
            continue;
        }

        if (imgid2fisheye.find(labels[i] + index_offset) == imgid2fisheye.end()) {
            ROS_WARN("Can't find image %d; skipping", labels[i] + index_offset);
            continue;
        }

        // int return_msg_id = imgid2fisheye.at(labels[i] + index_offset);
        int return_msg_id = labels[i] + index_offset;
        int return_drone_id = fisheyeframe_database[return_msg_id].drone_id;

        ROS_INFO("Return Label %d from %d, distance %f", labels[i] + index_offset, return_drone_id, distances[i]);
        if (labels[i] < database_size() - max_index && distances[i] < thres) {
            //Is same id, max index make sense
            distance = distances[i];
            ROS_INFO("Database return %ld on drone %d, radius %f msg_id %d", labels[i] + index_offset, return_drone_id, distances[i], return_msg_id);
            return return_msg_id;
        }
    }

    return -1;
}


FisheyeFrameDescriptor_t & LoopDetector::query_fisheyeframe_from_database(const FisheyeFrameDescriptor_t & new_img_desc, bool init_mode, bool nonkeyframe, int & direction_new, int & direction_old) {
    double best_distance = 1000;
    int best_image_id = -1;
    //Strict use direction 1 now
    direction_new = 1;
    if (new_img_desc.images[direction_new].landmark_num > 0) {
        double distance = 1000;
        int id = query_from_database(new_img_desc.images.at(direction_new), init_mode, nonkeyframe, distance);
        
        if (id != -1 && distance < best_distance) {
            best_image_id = id;
        }

        if (best_image_id != -1) {
            int msg_id = imgid2fisheye[best_image_id];
            direction_old = imgid2dir[best_image_id];
            FisheyeFrameDescriptor_t & ret = fisheyeframe_database[msg_id];
            ROS_INFO("Database return image %d fisheye frame from drone %d with direction %d", 
                best_image_id, ret.drone_id, direction_old);
            return ret;
        }
    }

    direction_old = -1;
    FisheyeFrameDescriptor_t ret;
    ret.msg_id = -1;
    return ret;
}


int LoopDetector::database_size() const {
    return local_index.ntotal + remote_index.ntotal;
}

bool pnp_result_verify(bool pnp_success, bool init_mode, int inliers, double rperr, const Swarm::Pose & DP_old_to_new) {
    bool success = pnp_success;
    if (!pnp_success) {
        return false;
    }

    if (rperr > RPERR_THRES) {
        ROS_INFO("Check failed on RP error %f", rperr*57.3);
        return false;
    }

    if (init_mode) {
        bool _success = (inliers >= INIT_MODE_MIN_LOOP_NUM) && fabs(DP_old_to_new.yaw()) < ACCEPT_LOOP_YAW_RAD && DP_old_to_new.pos().norm() < MAX_LOOP_DIS;            
        if (!_success) {
        _success = (inliers >= INIT_MODE_MIN_LOOP_NUM_LEVEL2) && fabs(DP_old_to_new.yaw()) < ACCEPT_LOOP_YAW_RAD && DP_old_to_new.pos().norm() < MAX_LOOP_DIS_LEVEL2;            
        }
        success = _success;
    } else {
        success = (inliers >= MIN_LOOP_NUM) && fabs(DP_old_to_new.yaw()) < ACCEPT_LOOP_YAW_RAD && DP_old_to_new.pos().norm() < MAX_LOOP_DIS;
    }        

    return success;
}


double RPerror(const Swarm::Pose & p_drone_old_in_new, const Swarm::Pose & drone_pose_old, const Swarm::Pose & drone_pose_now) {
    Swarm::Pose DP_old_to_new_6d =  Swarm::Pose::DeltaPose(p_drone_old_in_new, drone_pose_now, false);
    Swarm::Pose Prediect_new_in_old_Pose = drone_pose_old * DP_old_to_new_6d;
    auto AttNew_in_old = Prediect_new_in_old_Pose.att().normalized();
    auto AttNew_in_new = drone_pose_now.att().normalized();
    auto dyaw = quat2eulers(AttNew_in_new).z() - quat2eulers(AttNew_in_old).z();
    AttNew_in_old = Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ())*AttNew_in_old;
    auto RPerr = (quat2eulers(AttNew_in_old) - quat2eulers(AttNew_in_new)).norm();
    // std::cout << "New In Old" << quat2eulers(AttNew_in_old) << std::endl;
    // std::cout << "New In New"  << quat2eulers(AttNew_in_new);
    // std::cout << "Estimate RP error" <<  (quat2eulers(AttNew_in_old) - quat2eulers(AttNew_in_new))*57.3 << std::endl;
    // std::cout << "Estimate RP error2" <<  quat2eulers( (AttNew_in_old.inverse()*AttNew_in_new).normalized())*57.3 << std::endl;
    return RPerr;
}

 

int LoopDetector::compute_relative_pose(const std::vector<cv::Point2f> matched_2d_norm_old,
        const std::vector<cv::Point3f> matched_3d_now,
        const std::vector<cv::Point2f> matched_2d_norm_now,
        const std::vector<cv::Point3f> matched_3d_old,
        Swarm::Pose old_extrinsic,
        Swarm::Pose drone_pose_now,
        Swarm::Pose drone_pose_old,
        Swarm::Pose & DP_old_to_new,
        bool init_mode,
        int drone_id_new, int drone_id_old,
        std::vector<cv::DMatch> &matches,
        int &inlier_num) {
        //Compute PNP
    ROS_INFO("Matched features %ld", matched_2d_norm_old.size());
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);

    cv::Mat r, rvec, rvec2, t, t2, D, tmp_r;
    cv::Mat inliers;


    // Swarm::Pose initial_old_drone_pose = drone_pose_old;
    // Swarm::Pose initial_old_cam_pose = initial_old_drone_pose * old_extrinsic;
    // Swarm::Pose old_cam_in_new_initial = drone_pose_now.inverse() * initial_old_cam_pose;
    // Swarm::Pose old_drone_to_new_initial = drone_pose_old.inverse() * drone_pose_now;
    // PnPInitialFromCamPose(initial_old_cam_pose, rvec, t);
    
    int iteratives = 100;

    if (init_mode) {
        iteratives = 1000;
    }

    bool success = solvePnPRansac(matched_3d_now, matched_2d_norm_old, K, D, rvec, t, false,   
        iteratives,  0.02, 0.99,  inliers);

    auto p_cam_old_in_new = PnPRestoCamPose(rvec, t);
    auto p_drone_old_in_new = p_cam_old_in_new*(old_extrinsic.to_isometry().inverse());
    
    if (!success) {
        return 0;
    }

    Swarm::Pose DP_old_to_new_6d = Swarm::Pose::DeltaPose(p_drone_old_in_new, drone_pose_now, false);
    std::cout << "DP_old_to_new_6d inliers" << inliers.size();
    DP_old_to_new_6d.print();

    DP_old_to_new =  Swarm::Pose::DeltaPose(p_drone_old_in_new, drone_pose_now, true);
    //As our pose graph uses 4D pose, here we must solve 4D pose
    //6D pose could use to verify the result but not give to swarm_localization
    std::cout << "PnP solved DPose 4D";
    DP_old_to_new.print();
    
    auto RPerr = RPerror(p_drone_old_in_new, drone_pose_old, drone_pose_now);

    success = pnp_result_verify(success, init_mode, inliers.rows, RPerr, DP_old_to_new);

    ROS_INFO("PnPRansac %d inlines %d, dyaw %f dpos %f. Geometry Check %f", success, inliers.rows, fabs(DP_old_to_new.yaw())*57.3, DP_old_to_new.pos().norm(), RPerr);
    inlier_num = inliers.rows;

    for (int i = 0; i < inlier_num; i++) {
        matches.push_back(cv::DMatch(inliers.at<int>(i, 0), inliers.at<int>(i, 0), 0));
    }
    return success;
}

cv::Point2f rotate_pt_norm2d(cv::Point2f pt, Eigen::Quaterniond q) {
    Eigen::Vector3d pt3d(pt.x, pt.y, 1);
    pt3d = q * pt3d;

    if (pt3d.z() < 1e-3 && pt3d.z() > 0) {
        pt3d.z() = 1e-3;
    }

    if (pt3d.z() > -1e-3 && pt3d.z() < 0) {
        pt3d.z() = -1e-3;
    }

    return cv::Point2f(pt3d.x()/ pt3d.z(), pt3d.y()/pt3d.z());
}

//Note! here the norms are both projected to main dir's unit sphere.
bool LoopDetector::compute_correspond_features(const FisheyeFrameDescriptor_t & new_frame_desc,
    const FisheyeFrameDescriptor_t & old_frame_desc, 
    int main_dir_new,
    int main_dir_old,
    std::vector<cv::Point2f> &new_norm_2d,
    std::vector<cv::Point3f> &new_3d,
    std::vector<std::vector<int>> &new_idx,
    std::vector<cv::Point2f> &old_norm_2d,
    std::vector<cv::Point3f> &old_3d,
    std::vector<std::vector<int>> &old_idx,
    std::vector<int> &dirs_new,
    std::vector<int> &dirs_old,
    std::map<int, std::pair<int, int>> &index2dirindex_new,
    std::map<int, std::pair<int, int>> &index2dirindex_old
) {
    //For each FisheyeFrameDescriptor_t, there must be 4 frames
    //However, due to the transmission and parameter, some may be empty.
    //We will only matched the frame which isn't empty
    for (int _dir_new = main_dir_new; _dir_new < main_dir_new + MAX_DIRS; _dir_new ++) {
        int dir_new = _dir_new % MAX_DIRS;
        int dir_old = ((main_dir_old - main_dir_new + MAX_DIRS) % MAX_DIRS + _dir_new)% MAX_DIRS;

        if (old_frame_desc.images[dir_old].landmark_num > 0 && new_frame_desc.images[dir_new].landmark_num > 0) {
            dirs_new.push_back(dir_new);
            dirs_old.push_back(dir_old);
        }
    }


    ROS_INFO("compute_correspond_features on main dir %d: %d", main_dir_old, main_dir_new);

    Swarm::Pose extrinsic_new(new_frame_desc.images[main_dir_new].camera_extrinsic);
    Swarm::Pose extrinsic_old(old_frame_desc.images[main_dir_old].camera_extrinsic);
    Eigen::Quaterniond main_quat_new =  extrinsic_new.att();
    Eigen::Quaterniond main_quat_old =  extrinsic_old.att();

    for (size_t i = 0; i < dirs_new.size(); i++) {
        int dir_new = dirs_new[i];
        int dir_old = dirs_old[i];
        std::vector<cv::Point2f> _new_norm_2d;
        std::vector<cv::Point3f> _new_3d;
        std::vector<int> _new_idx;
        std::vector<cv::Point2f> _old_norm_2d;
        std::vector<cv::Point3f> _old_3d;
        std::vector<int> _old_idx;

        compute_correspond_features(
            new_frame_desc.images[dir_new],
            old_frame_desc.images[dir_old],
            _new_norm_2d,
            _new_3d,
            _new_idx,
            _old_norm_2d,
            _old_3d,
            _old_idx
        );

        ROS_INFO("compute_correspond_features on direction %d:%d gives %d common features", dir_old, dir_new, _new_3d.size());

        new_3d.insert(new_3d.end(), _new_3d.begin(), _new_3d.end());
        old_3d.insert(old_3d.end(), _old_3d.begin(), _old_3d.end());
        new_idx.push_back(_new_idx);
        old_idx.push_back(_old_idx);

        Swarm::Pose _extrinsic_new(new_frame_desc.images[dir_new].camera_extrinsic);
        Swarm::Pose _extrinsic_old(old_frame_desc.images[dir_old].camera_extrinsic);

        Eigen::Quaterniond dq_new = main_quat_new.inverse() * _extrinsic_new.att();
        Eigen::Quaterniond dq_old = main_quat_old.inverse() * _extrinsic_old.att();

        for (size_t id = 0; id < _old_norm_2d.size(); id++) {
            auto pt = _old_norm_2d[id];
            // std::cout << "PT " << pt << " ROTATED " << rotate_pt_norm2d(pt, dq_old) << std::endl;
            index2dirindex_old[old_norm_2d.size()] = std::make_pair(dir_old, _old_idx[id]);
            old_norm_2d.push_back(rotate_pt_norm2d(pt, dq_old));
        }

        for (size_t id = 0; id < _new_norm_2d.size(); id++) {
            auto pt = _new_norm_2d[id];
            index2dirindex_new[new_norm_2d.size()] = std::make_pair(dir_new, _new_idx[id]);
            new_norm_2d.push_back(rotate_pt_norm2d(pt, dq_new));
        }
    }

    if(new_norm_2d.size() > 0) {
        return true;
    } else {
        return false;
    }
}

bool LoopDetector::compute_correspond_features(const ImageDescriptor_t & new_img_desc, const ImageDescriptor_t & old_img_desc, 
        std::vector<cv::Point2f> &new_norm_2d,
        std::vector<cv::Point3f> &new_3d,
        std::vector<int> &new_idx,
        std::vector<cv::Point2f> &old_norm_2d,
        std::vector<cv::Point3f> &old_3d,
        std::vector<int> &old_idx) {

    assert(new_img_desc.landmarks_2d.size() * LOCAL_DESC_LEN == new_img_desc.feature_descriptor.size() && "Desciptor size of new img desc must equal to to landmarks*256!!!");
    assert(old_img_desc.landmarks_2d.size() * LOCAL_DESC_LEN == old_img_desc.feature_descriptor.size() && "Desciptor size of old img desc must equal to to landmarks*256!!!");

    auto _old_norm_2d = toCV(old_img_desc.landmarks_2d_norm);
    auto _old_3d = toCV(old_img_desc.landmarks_3d);
    
    auto _now_norm_2d = toCV(new_img_desc.landmarks_2d_norm);
    auto _now_3d = toCV(new_img_desc.landmarks_3d);

    cv::Mat desc_old( old_img_desc.landmarks_2d.size(), LOCAL_DESC_LEN, CV_32F);
    memcpy(desc_old.data, old_img_desc.feature_descriptor.data(), old_img_desc.feature_descriptor.size()*sizeof(float));
    cv::Mat desc_now( new_img_desc.landmarks_2d.size(), LOCAL_DESC_LEN, CV_32F);
    memcpy(desc_now.data, new_img_desc.feature_descriptor.data(), new_img_desc.feature_descriptor.size()*sizeof(float));

    cv::BFMatcher bfmatcher(cv::NORM_L2, true);
    std::vector<cv::DMatch> _matches;
    bfmatcher.match(desc_now, desc_old, _matches);
    for (auto match : _matches) {
        if (match.distance < ACCEPT_SP_MATCH_DISTANCE) {
            int now_id = match.queryIdx;
            int old_id = match.trainIdx;

            new_idx.push_back(now_id);
            old_idx.push_back(old_id);

            new_3d.push_back(_now_3d[now_id]);
            new_norm_2d.push_back(_now_norm_2d[now_id]);

            old_3d.push_back(_old_3d[old_id]);
            old_norm_2d.push_back(_old_norm_2d[old_id]);
        } else {
            // printf("Give up distance too high %f\n", match.distance);
        }
    }

    return true;
}

bool LoopDetector::compute_loop(const FisheyeFrameDescriptor_t & new_frame_desc, const FisheyeFrameDescriptor_t & old_frame_desc,
    int main_dir_new, int main_dir_old,
    std::vector<cv::Mat> imgs_new, std::vector<cv::Mat> imgs_old,
    LoopConnection & ret, bool init_mode) {

    if (new_frame_desc.landmark_num < MIN_LOOP_NUM) {
        return false;
    }
    //Recover imformation

    assert(new_frame_desc.drone_id == self_id && "old img desc must from self drone!");

    bool success = false;

    ROS_INFO("Compute loop drone %d(dir %d)->%d(dir %d) landmarks %d:%d. Init %d", old_frame_desc.drone_id, main_dir_old,  new_frame_desc.drone_id, main_dir_new,
        old_frame_desc.landmark_num,
        new_frame_desc.landmark_num,
        init_mode);

    std::vector<cv::Point2f> new_norm_2d;
    std::vector<cv::Point3f> new_3d;
    std::vector<std::vector<int>> new_idx;
    std::vector<cv::Point2f> old_norm_2d;
    std::vector<cv::Point3f> old_3d;
    std::vector<std::vector<int>> old_idx;
    std::vector<int> dirs_new;
    std::vector<int> dirs_old;
    Swarm::Pose DP_old_to_new;
    std::vector<cv::DMatch> matches;
    std::map<int, std::pair<int, int>> index2dirindex_old;
    std::map<int, std::pair<int, int>> index2dirindex_new;
    int inlier_num = 0;
    
    compute_correspond_features(new_frame_desc, old_frame_desc, 
        main_dir_new, main_dir_old,
        new_norm_2d, new_3d, new_idx,
        old_norm_2d, old_3d, old_idx, dirs_new, dirs_old, 
        index2dirindex_new, index2dirindex_old);
    
    if(new_norm_2d.size() > MIN_LOOP_NUM || (init_mode && new_norm_2d.size() > INIT_MODE_MIN_LOOP_NUM  )) 
    {
        success = compute_relative_pose(
                new_norm_2d, new_3d, 
                old_norm_2d, old_3d,
                Swarm::Pose(old_frame_desc.images[main_dir_old].camera_extrinsic),
                Swarm::Pose(new_frame_desc.pose_drone),
                Swarm::Pose(old_frame_desc.pose_drone),
                DP_old_to_new,
                init_mode,
                new_frame_desc.drone_id,
                old_frame_desc.drone_id,
                matches,
                inlier_num
        );
    } else {
        ROS_INFO("Too less common feature %ld, will give up", new_norm_2d.size());
    }
    cv::Mat show;

    if (enable_visualize) {
        char title[100] = {0};
        std::vector<cv::Mat> _matched_imgs;
        _matched_imgs.resize(imgs_old.size());
        for (size_t i = 0; i < imgs_old.size(); i ++) {
            int dir_new = ((-main_dir_old + main_dir_new + MAX_DIRS) % MAX_DIRS + i)% MAX_DIRS;
            cv::vconcat(imgs_old[i], imgs_new[dir_new], _matched_imgs[i]);
        } 

        for (size_t i = 0; i < new_norm_2d.size(); i ++) {
            int old_pt_id = index2dirindex_old[i].second;
            int old_dir_id = index2dirindex_old[i].first;

            int new_pt_id = index2dirindex_new[i].second;
            int new_dir_id = index2dirindex_new[i].first;
            auto pt_old = toCV(old_frame_desc.images[old_dir_id].landmarks_2d[old_pt_id]);
            auto pt_new = toCV(new_frame_desc.images[new_dir_id].landmarks_2d[new_pt_id]);

            cv::line(_matched_imgs[old_dir_id], pt_old, pt_new + cv::Point2f(0, imgs_old[old_dir_id].rows), cv::Scalar(0, 0, 255));
            cv::circle(_matched_imgs[old_dir_id], pt_old, 3, cv::Scalar(255, 0, 0), 1);
            cv::circle(_matched_imgs[old_dir_id], pt_new + cv::Point2f(0, imgs_old[old_dir_id].rows), 3, cv::Scalar(255, 0, 0), 1);
        
        }

        for (auto match: matches) {
            int idi = match.queryIdx;
            int idj = match.trainIdx;
            int old_pt_id = index2dirindex_old[idi].second;
            int old_dir_id = index2dirindex_old[idi].first;

            int new_pt_id = index2dirindex_new[idi].second;
            int new_dir_id = index2dirindex_new[idi].first;
            auto pt_old = toCV(old_frame_desc.images[old_dir_id].landmarks_2d[old_pt_id]);
            auto pt_new = toCV(new_frame_desc.images[new_dir_id].landmarks_2d[new_pt_id]);

            cv::line(_matched_imgs[old_dir_id], pt_old, pt_new + cv::Point2f(0, imgs_old[old_dir_id].rows), cv::Scalar(0, 255, 0));
            cv::circle(_matched_imgs[old_dir_id], pt_old, 3, cv::Scalar(255, 0, 0), 1);
            cv::circle(_matched_imgs[new_dir_id], pt_new + cv::Point2f(0, imgs_old[old_dir_id].rows), 3, cv::Scalar(255, 0, 0), 1);
        }
        

        show = _matched_imgs[0];
        for (size_t i = 1; i < _matched_imgs.size(); i ++) {
            cv::line(_matched_imgs[i], cv::Point2f(0, 0), cv::Point2f(0, _matched_imgs[i].rows), cv::Scalar(255, 255, 0), 2);
            cv::hconcat(show, _matched_imgs[i], show);
        }


         if (success) {
            sprintf(title, "SUCCESS LOOP %d->%d inliers %d", old_frame_desc.drone_id, new_frame_desc.drone_id, inlier_num);
            cv::putText(show, title, cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 3);
        } else {
            sprintf(title, "FAILED LOOP %d->%d inliers %d", old_frame_desc.drone_id, new_frame_desc.drone_id, inlier_num);
            cv::putText(show, title, cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 3);
        }

        // cv::resize(show, show, cv::Size(), 2, 2);
        cv::imshow("Matches", show);
        cv::waitKey(10);
    }

    if (success) {
        ret.dpos.x = DP_old_to_new.pos().x();
        ret.dpos.y = DP_old_to_new.pos().y();
        ret.dpos.z = DP_old_to_new.pos().z();
        ret.dyaw = DP_old_to_new.yaw();

        ret.id_a = old_frame_desc.drone_id;
        ret.ts_a = toROSTime(old_frame_desc.timestamp);

        ret.id_b = new_frame_desc.drone_id;
        ret.ts_b = toROSTime(new_frame_desc.timestamp);

        ROS_INFO("OLD TS %ld NEW TS %ld", old_frame_desc.timestamp.sec, new_frame_desc.timestamp.sec);

        ret.self_pose_a = toROSPose(old_frame_desc.pose_drone);
        ret.self_pose_b = toROSPose(new_frame_desc.pose_drone);

        return true;
    }
    return false;
}

void LoopDetector::on_loop_connection(LoopConnection & loop_conn) {
    on_loop_cb(loop_conn);
}

LoopDetector::LoopDetector(): local_index(DEEP_DESC_SIZE), remote_index(DEEP_DESC_SIZE) {
    cv::RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(cv::Scalar(r,g,b));
    }
}



Swarm::Pose solve_affine_pts3d(std::vector<cv::Point3f> matched_3d_now, std::vector<cv::Point3f> matched_3d_old) {
    cv::Mat affine;
    std::vector<uchar> _inliners;
    Eigen::Matrix<double, 3, Eigen::Dynamic> src (3, matched_3d_now.size ());
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt (3, matched_3d_old.size ());
    
    for (std::size_t i = 0; i < matched_3d_now.size (); ++i)
    {
        src (0, i) = matched_3d_now[i].x;
        src (1, i) = matched_3d_now[i].y;
        src (2, i) = matched_3d_now[i].z;
    
        tgt (0, i) = matched_3d_old[i].x;
        tgt (1, i) = matched_3d_old[i].y;
        tgt (2, i) = matched_3d_old[i].z;
    }
    Eigen::Matrix4d transformation_matrix = Eigen::umeyama (src, tgt, false);

    std::cout << transformation_matrix << std::endl;
    Swarm::Pose p_cam_old_in_new_affine = AffineRestoCamPose(transformation_matrix);

    int c = 0;
    for (auto i : _inliners) {
        c += i;
    }

    std::cout << "Affine gives" << c << " inliers" << std::endl;
    return p_cam_old_in_new_affine;
}
