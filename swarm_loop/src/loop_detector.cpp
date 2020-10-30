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
        ROS_INFO("Receive image from %d with %d images and landmark: %d", drone_id, flatten_desc.images.size(), 
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
            
            int direction = 2;
            FisheyeFrameDescriptor_t & _old_fisheye_img = query_fisheyeframe_from_database(flatten_desc, init_mode, flatten_desc.prevent_adding_db, direction);

            auto stop = high_resolution_clock::now(); 

            if (_old_fisheye_img.msg_id > 0 ) {
                LoopConnection ret;

                if (_old_fisheye_img.drone_id == self_id) {
                    success = compute_loop(_old_fisheye_img, flatten_desc, msgid2cvimgs[_old_fisheye_img.msg_id],  imgs, ret, init_mode);
                } else {
                    //We grab remote drone from database
                    if (flatten_desc.drone_id == self_id) {
                        success = compute_loop(flatten_desc, _old_fisheye_img, imgs, msgid2cvimgs[_old_fisheye_img.msg_id], ret, init_mode);
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
        int index = add_to_database(img_desc);
        imgid2fisheye[index] = new_fisheye_desc.msg_id;
        imgid2dir[index] = i;
    }

    return new_fisheye_desc.msg_id;
}

int LoopDetector::add_to_database(const ImageDescriptor_t & new_img_desc) {
    if (new_img_desc.drone_id == self_id) {
        local_index.add(1, new_img_desc.image_desc.data());
        ROS_INFO("Add keyframe from %d to local keyframe database index: %d", new_img_desc.drone_id, local_index.ntotal - 1);
        return local_index.ntotal - 1;
    } else {
        remote_index.add(1, new_img_desc.image_desc.data());
        ROS_INFO("Add keyframe from %d to remote keyframe database index: %d", new_img_desc.drone_id, remote_index.ntotal - 1);
        return remote_index.ntotal - 1 + REMOTE_MAGIN_NUMBER;
    }
    return -1;
}


FisheyeFrameDescriptor_t & LoopDetector::query_fisheyeframe_from_database(const FisheyeFrameDescriptor_t & new_img_desc, bool init_mode, bool nonkeyframe, int & direction) {
    double best_distance = 1000;
    int best_image_id = -1;
    for (size_t dir = 0; dir < new_img_desc.images.size(); dir ++) {
        double distance = 1000;
        int id = query_from_database(new_img_desc.images.at(dir), init_mode, nonkeyframe, distance);
        if (id > 0 && distance < best_distance) {
            best_image_id = id;
        }
    }

    if (best_image_id >= 0) {
        int msg_id = imgid2fisheye[best_image_id];
        direction = imgid2dir[best_image_id];
        FisheyeFrameDescriptor_t & ret = fisheyeframe_database[best_image_id];
        ROS_INFO("Database return fishe frame from drone %d with direction %d", 
            ret.drone_id, direction);
        return ret;
    }

    direction = -1;
    FisheyeFrameDescriptor_t ret;
    return ret;
}

int LoopDetector::query_from_database(const ImageDescriptor_t & img_desc, bool init_mode, bool nonkeyframe, double & distance) {
    double thres = INNER_PRODUCT_THRES;
    if (init_mode) {
        thres = INIT_MODE_PRODUCT_THRES;
    }

    if (img_desc.drone_id == self_id) {
        //Then this is self drone
        int _id = query_from_database(img_desc, remote_index, true, thres, 1, distance);
        if (_id > 0) {
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

        int return_msg_id = imgid2fisheye.at(labels[i] + index_offset);
        int return_drone_id = fisheyeframe_database[return_msg_id].drone_id;

        ROS_INFO("Return Label %d from %d, distance %f", labels[i] + index_offset, return_drone_id, distances[i]);

        if (labels[i] < database_size() - max_index && distances[i] < thres) {
            //Is same id, max index make sense
            ROS_INFO("Suitable Find %ld on drone %d->%d, radius %f", labels[i] + index_offset, return_drone_id, img_desc.drone_id, distances[i]);
            return return_msg_id;
        }
    }

    return -1;
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



int LoopDetector::compute_relative_pose(const std::vector<cv::Point2f> now_norm_2d,
        const std::vector<cv::Point3f> now_3d,
        const cv::Mat desc_now,

        const std::vector<cv::Point2f> old_norm_2d,
        const std::vector<cv::Point3f> old_3d,
        const cv::Mat desc_old,

        Swarm::Pose old_extrinsic,
        Swarm::Pose drone_pose_now,
        Swarm::Pose drone_pose_old,
        Swarm::Pose & DP_old_to_new,
        bool init_mode,
        int drone_id_new, int drone_id_old,
        std::vector<cv::DMatch> &matches,
        int &inlier_num) {
    
    cv::BFMatcher bfmatcher(cv::NORM_L2, true);
    //query train

    std::vector<cv::DMatch> _matches;
    bfmatcher.match(desc_now, desc_old, _matches);

    std::vector<cv::Point3f> matched_3d_now, matched_3d_old;
    std::vector<cv::Point2f> matched_2d_norm_old, matched_2d_norm_now;

    for (auto match : _matches) {
        int now_id = match.queryIdx;
        int old_id = match.trainIdx;

        matched_3d_now.push_back(now_3d[now_id]);
        matched_2d_norm_old.push_back(old_norm_2d[old_id]);
        matched_2d_norm_now.push_back(now_norm_2d[now_id]);
        matched_3d_old.push_back(old_3d[old_id]);
    }

    std::cout << std::endl;

    if(_matches.size() > MIN_LOOP_NUM || (init_mode && matches.size() > INIT_MODE_MIN_LOOP_NUM  )) {
        //Compute PNP
        ROS_INFO("Matched features %ld/%ld", matched_2d_norm_old.size(), _matches.size());
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);

        cv::Mat r, rvec, rvec2, t, t2, D, tmp_r;
        cv::Mat inliers;


        //TODO: Prepare initial pose for swarm

        Swarm::Pose initial_old_drone_pose = drone_pose_old;
        // }

        Swarm::Pose initial_old_cam_pose = initial_old_drone_pose * old_extrinsic;
        Swarm::Pose old_cam_in_new_initial = drone_pose_now.inverse() * initial_old_cam_pose;
        Swarm::Pose old_drone_to_new_initial = drone_pose_old.inverse() * drone_pose_now;

        PnPInitialFromCamPose(initial_old_cam_pose, rvec, t);
        
        int iteratives = 100;

        if (init_mode) {
            iteratives = 1000;
        }

        bool success = solvePnPRansac(matched_3d_now, matched_2d_norm_old, K, D, rvec, t, true,   
            iteratives,  0.02, 0.99,  inliers);

        // std::set<int> inlier_cnt;
        // for (int i = 0; i < inliers.rows; i++) {
        //     int idx = inliers.at<int>(i);
        //     inlier_cnt.insert(idx);
        // }
        // std::vector<unsigned char> status(matched_3d_now.size(), 0);
        // cv::Mat inliers2;
        // bool success2 = solvePnPRansac(matched_3d_old, matched_2d_norm_now, K, D, rvec2, t2, false,   
        //     iteratives,  0.02, 0.99,  inliers2);

        // for (int i = 0; i < inliers2.rows; i++) {
        //     int idx = inliers2.at<int>(i);
        //     if (inlier_cnt.find(idx) != inlier_cnt.end()) {
        //         status[idx] = 1;
        //     }
        // }
        // auto p_cam_old_in_new2 = PnPRestoCamPose(rvec2, t2);
        // Swarm::Pose DP_new_to_old_6d = Swarm::Pose::DeltaPose(p_cam_old_in_new2*(old_extrinsic.to_isometry().inverse()), drone_pose_old, false);
        // std::cout << "DP_new_to_old_6d inliers" << inliers2.size();
        // DP_new_to_old_6d.print();

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
            int idx = inliers.at<int>(i);
            matches.push_back(_matches[idx]);
        }
        return success;
    } else {
        ROS_INFO("Matched features too less %ld", _matches.size());
    }

    return 0;
}


bool LoopDetector::compute_loop(const FisheyeFrameDescriptor_t & new_img_desc, const FisheyeFrameDescriptor_t & old_img_desc, 
    std::vector<cv::Mat> imgs_new, std::vector<cv::Mat> imgs_old, LoopConnection & ret, bool init_mode) {
    /*
    if (new_img_desc.landmark_num < MIN_LOOP_NUM) {
        return false;
    }
    //Recover imformation

    assert(new_img_desc.drone_id == self_id && "old img desc must from self drone!");

    bool success = false;
    Swarm::Pose  DP_old_to_new;

    ROS_INFO("Compute loop drone %d->%d landmarks %d:%d. Init %d", old_img_desc.drone_id, new_img_desc.drone_id, 
        old_img_desc.landmark_num,
        new_img_desc.landmark_num,
        init_mode);

    auto now_2d = toCV(new_img_desc.landmarks_2d);
    auto now_norm_2d = toCV(new_img_desc.landmarks_2d_norm);
    auto now_3d = toCV(new_img_desc.landmarks_3d);

    assert(new_img_desc.landmarks_2d.size() * LOCAL_DESC_LEN == new_img_desc.feature_descriptor.size() && "Desciptor size of new img desc must equal to to landmarks*256!!!");
    assert(old_img_desc.landmarks_2d.size() * LOCAL_DESC_LEN == old_img_desc.feature_descriptor.size() && "Desciptor size of old img desc must equal to to landmarks*256!!!");

    cv::Mat desc_now( new_img_desc.landmarks_2d.size(), LOCAL_DESC_LEN, CV_32F);
    memcpy(desc_now.data, new_img_desc.feature_descriptor.data(), new_img_desc.feature_descriptor.size()*sizeof(float));

    auto old_2d = toCV(old_img_desc.landmarks_2d);
    auto old_norm_2d = toCV(old_img_desc.landmarks_2d_norm);
    auto old_3d = toCV(old_img_desc.landmarks_3d);
    cv::Mat desc_old( old_img_desc.landmarks_2d.size(), LOCAL_DESC_LEN, CV_32F);
    memcpy(desc_old.data, old_img_desc.feature_descriptor.data(), old_img_desc.feature_descriptor.size()*sizeof(float));

    std::vector<cv::DMatch> matches;

    int inlier_num = 0;
    success = compute_relative_pose(
            now_norm_2d, now_3d, desc_now,
            old_norm_2d, old_3d, desc_old,
            Swarm::Pose(old_img_desc.camera_extrinsic),
            Swarm::Pose(new_img_desc.pose_drone),
            Swarm::Pose(old_img_desc.pose_drone),
            DP_old_to_new,
            init_mode,
            new_img_desc.drone_id,
            old_img_desc.drone_id,
            matches,
            inlier_num
    );

    cv::Mat show;

    if (enable_visualize) {
        assert(!img_new.empty() && "ERROR IMG NEW is emptry!");
        assert(!img_old.empty() && "ERROR IMG old is emptry!");
        static char title[256] = {0};
        cv::drawMatches(img_new, to_keypoints(now_2d), img_old, to_keypoints(old_2d), matches, show, cv::Scalar::all(-1), cv::Scalar::all(-1));
        // cv::resize(show, show, cv::Size(), 2, 2);
         if (success) {
            sprintf(title, "SUCCESS LOOP %d->%d inliers %d", old_img_desc.drone_id, new_img_desc.drone_id, inlier_num);
            cv::putText(show, title, cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 3);
        } else {
            sprintf(title, "FAILED LOOP %d->%d inliers %d", old_img_desc.drone_id, new_img_desc.drone_id, inlier_num);
            cv::putText(show, title, cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 3);
        }

        cv::Mat show2 = img_old;

        for (auto match : matches) {
            int now_id = match.queryIdx;
            int old_id = match.trainIdx;
            cv::arrowedLine(show2, now_2d[now_id], old_2d[old_id], cv::Scalar(255, 255, 0), 1);
        }

        cv::imshow("SHOW_FEA", show2);

        cv::imshow("Matches", show);
        cv::waitKey(10);
    }

    if (success) {
        ret.dpos.x = DP_old_to_new.pos().x();
        ret.dpos.y = DP_old_to_new.pos().y();
        ret.dpos.z = DP_old_to_new.pos().z();
        ret.dyaw = DP_old_to_new.yaw();

        ret.id_a = old_img_desc.drone_id;
        ret.ts_a = toROSTime(old_img_desc.timestamp);

        ret.id_b = new_img_desc.drone_id;
        ret.ts_b = toROSTime(new_img_desc.timestamp);

        ROS_INFO("OLD TS %ld NEW TS %ld", old_img_desc.timestamp.sec, new_img_desc.timestamp.sec);

        ret.self_pose_a = toROSPose(old_img_desc.pose_drone);
        ret.self_pose_b = toROSPose(new_img_desc.pose_drone);

        return true;
    }
    return false;
    */
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
