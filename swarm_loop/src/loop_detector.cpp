#include <loop_detector.h>
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <opencv2/opencv.hpp>
#include <chrono> 
#include <opencv2/core/eigen.hpp>

using namespace std::chrono; 


void debug_draw_kpts(const ImageDescriptor_t & img_des, cv::Mat img) {
    auto pts = toCV(img_des.landmarks_2d);
    for (auto pt : pts) {
        cv::circle(img, pt, 1, cv::Scalar(255, 0, 0), -1);
    }
    cv::resize(img, img, cv::Size(), 3.0, 3.0);
    cv::imshow("DEBUG", img);
    cv::waitKey(30);
}

void LoopDetector::on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img) {
    auto start = high_resolution_clock::now(); 
    if (img_des.drone_id!= this->self_id && database_size() == 0) {
        ROS_INFO("Empty local database, where giveup remote image");
        return;
    } 


    success_loop_nodes.insert(self_id);
    bool new_node = all_nodes.find(img_des.drone_id) == all_nodes.end();

    all_nodes.insert(img_des.drone_id);


    if (img_des.landmark_num >= MIN_LOOP_NUM) {
        // std::cout << "Add Time cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 <<"ms" << std::endl;
        bool init_mode = success_loop_nodes.find(img_des.drone_id) == success_loop_nodes.end();

        if (enable_visualize) {
            if (img.empty()) {
                img = decode_image(img_des);
            }
            debug_draw_kpts(img_des, img);
        }

        int new_added_image = -1;
        if (!img_des.prevent_adding_db || new_node) {
            new_added_image = add_to_database(img_des);
            id2imgdes[new_added_image] = img_des;
            id2cvimg[new_added_image] = img;
        } else {
            ROS_INFO("This image is prevent to adding to DB");
        }

        bool success = false;

        if (database_size() > MATCH_INDEX_DIST || init_mode || img_des.drone_id != self_id) {

            ROS_INFO("Querying image from database size %d init_mode %d....", database_size(), init_mode);
            int _old_id = -1;
            if (init_mode) {
                _old_id = query_from_database(img_des, init_mode);
            } else {
                _old_id = query_from_database(img_des);
            }

            auto stop = high_resolution_clock::now(); 

            if (_old_id >= 0 ) {
                // std::cout << "Time Cost " << duration_cast<microseconds>(stop - start).count()/1000.0 <<"ms RES: " << _old_id << std::endl;                
                LoopConnection ret;

                if (id2imgdes[_old_id].drone_id == self_id) {
                    success = compute_loop(img_des, id2imgdes[_old_id], img, id2cvimg[_old_id], ret, init_mode);
                } else {
                    //We grab remote drone from database
                    if (img_des.drone_id == self_id) {
                        success = compute_loop(id2imgdes[_old_id], img_des, id2cvimg[_old_id], img, ret, init_mode);
                    } else {
                        ROS_WARN("Will not compute loop, drone id is %d(self %d), new_added_image id %d", img_des.drone_id, self_id, new_added_image);
                    }
                }

                if (success) {
                    ROS_INFO("Adding success matched drone %d to database", img_des.drone_id);
                    success_loop_nodes.insert(img_des.drone_id);

                    ROS_INFO("\nDetected pub loop %d->%d DPos %4.3f %4.3f %4.3f Dyaw %3.2fdeg\n",
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
        ROS_WARN("Frame contain too less landmark %d, give up", img_des.landmark_num);
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

int LoopDetector::add_to_database(const ImageDescriptor_t & new_img_desc) {
#ifdef USE_DEEPNET
    index.add(1, new_img_desc.image_desc.data());
    return index.ntotal - 1;
#else
    cv::Mat feature = cvfeatureFromByte((uint8_t*)img_des.feature_descriptor.data, LOOP_FEATURE_NUM);
    int _id = db.add(feature);
    return _id;
#endif

}

int LoopDetector::query_from_database(const ImageDescriptor_t & img_desc, bool init_mode) {
#ifdef USE_DEEPNET
    float distances[SEARCH_NEAREST_NUM] = {0};
    faiss::Index::idx_t labels[SEARCH_NEAREST_NUM];
    
    for (int i = 0; i < SEARCH_NEAREST_NUM; i++) {
        labels[i] = -1;
    }

    index.search(1, img_desc.image_desc.data(), SEARCH_NEAREST_NUM, distances, labels);
    
    int max_index = MATCH_INDEX_DIST;
    
    for (int i = 0; i < SEARCH_NEAREST_NUM; i++) {
        double thres = INNER_PRODUCT_THRES;

        if (labels[i] < 0) {
            continue;
        }
        if (id2imgdes.find(labels[i]) == id2imgdes.end()) {
            ROS_WARN("Can't find image %d; skipping", labels[i]);
            continue;
        }

        int return_drone_id = id2imgdes.at(labels[i]).drone_id;
        if (init_mode) {
            thres = INIT_MODE_PRODUCT_THRES;
        }

        if (img_desc.drone_id == self_id || return_drone_id == self_id) {
            if (img_desc.drone_id != return_drone_id) {
                //Not same drone id, we don't care about the max index
                thres = INIT_MODE_PRODUCT_THRES;
                if (labels[i] < database_size() - 1 && distances[i] > thres) {
                    ROS_INFO("Suitable Find %ld on drone %d->%d, radius %f", labels[i], return_drone_id, img_desc.drone_id, distances[i]);
                    return labels[i];
                }
            }

            if (labels[i] < database_size() - max_index && distances[i] > thres) {
                //Is same id, max index make sense
                ROS_INFO("Suitable Find %ld on drone %d->%d, radius %f", labels[i], return_drone_id, img_desc.drone_id, distances[i]);
                return labels[i];
            }
        }

    }

    return -1;
#else
    cv::Mat feature = cvfeatureFromByte((uint8_t*)img_desc.feature_descriptor.data, LOOP_FEATURE_NUM);
    DBoW3::QueryResults ret;
    db.query(feature, ret, 1, db.size() - max_index);

    if (ret.size() > 0 && ret[0].Score > LOOP_BOW_THRES) {
        return ret[0].Id;
    }
    return -1;

#endif
}

int LoopDetector::database_size() const {
#ifdef USE_DEEPNET
    return index.ntotal;
#else
    return db.size();
#endif
}

std::vector<cv::DMatch> filter_by_duv(const std::vector<cv::DMatch> & matches, 
    std::vector<cv::KeyPoint> query_pts, 
    std::vector<cv::KeyPoint> train_pts) {
    std::vector<cv::DMatch> good_matches;
    std::vector<float> uv_dis;
    for (auto gm : matches) {
        if (gm.queryIdx >= query_pts.size() || gm.trainIdx >= train_pts.size()) {
            ROS_ERROR("out of size");
            exit(-1);
        } 
        uv_dis.push_back(cv::norm(query_pts[gm.queryIdx].pt - train_pts[gm.trainIdx].pt));
    }

    std::sort(uv_dis.begin(), uv_dis.end());
    
    // printf("MIN UV DIS %f, MID %f END %f\n", uv_dis[0], uv_dis[uv_dis.size()/2], uv_dis[uv_dis.size() - 1]);

    double mid_dis = uv_dis[uv_dis.size()/2];

    for (auto gm: matches) {
        if (gm.distance < mid_dis*ORB_UV_DISTANCE) {
            good_matches.push_back(gm);
        }
    }

    return good_matches;
}

std::vector<cv::DMatch> filter_by_x(const std::vector<cv::DMatch> & matches, 
    std::vector<cv::KeyPoint> query_pts, 
    std::vector<cv::KeyPoint> train_pts, double OUTLIER_XY_PRECENT) {
    std::vector<cv::DMatch> good_matches;
    std::vector<float> dxs;
    for (auto gm : matches) {
        dxs.push_back(query_pts[gm.queryIdx].pt.x - train_pts[gm.trainIdx].pt.x);
    }

    std::sort(dxs.begin(), dxs.end());

    int num = dxs.size();
    int l = num*OUTLIER_XY_PRECENT;
    if (l == 0) {
        l = 1;
    }
    int r = num*(1-OUTLIER_XY_PRECENT);
    if (r >= num - 1) {
        r = num - 2;
    }

    if (r <= l ) {
        return good_matches;
    }

    // printf("MIN DX DIS:%f, l:%f m:%f r:%f END:%f\n", dxs[0], dxs[l], dxs[num/2], dxs[r], dxs[dxs.size() - 1]);

    double lv = dxs[l];
    double rv = dxs[r];

    for (auto gm: matches) {
        if (query_pts[gm.queryIdx].pt.x - train_pts[gm.trainIdx].pt.x > lv && query_pts[gm.queryIdx].pt.x - train_pts[gm.trainIdx].pt.x < rv) {
            good_matches.push_back(gm);
        }
    }

    return good_matches;
}

std::vector<cv::DMatch> filter_by_y(const std::vector<cv::DMatch> & matches, 
    std::vector<cv::KeyPoint> query_pts, 
    std::vector<cv::KeyPoint> train_pts, double OUTLIER_XY_PRECENT) {
    std::vector<cv::DMatch> good_matches;
    std::vector<float> dys;
    for (auto gm : matches) {
        dys.push_back(query_pts[gm.queryIdx].pt.y - train_pts[gm.trainIdx].pt.y);
    }

    std::sort(dys.begin(), dys.end());

    int num = dys.size();
    int l = num*OUTLIER_XY_PRECENT;
    if (l == 0) {
        l = 1;
    }
    int r = num*(1-OUTLIER_XY_PRECENT);
    if (r >= num - 1) {
        r = num - 2;
    }

    if (r <= l ) {
        return good_matches;
    }

    // printf("MIN DX DIS:%f, l:%f m:%f r:%f END:%f\n", dys[0], dys[l], dys[num/2], dys[r], dys[dys.size() - 1]);

    double lv = dys[l];
    double rv = dys[r];

    for (auto gm: matches) {
        if (query_pts[gm.queryIdx].pt.y - train_pts[gm.trainIdx].pt.y > lv && query_pts[gm.queryIdx].pt.y - train_pts[gm.trainIdx].pt.y < rv) {
            good_matches.push_back(gm);
        }
    }

    return good_matches;
}


std::vector<cv::DMatch> filter_by_crop_x(const std::vector<cv::DMatch> & matches, 
    std::vector<cv::KeyPoint> query_pts, 
    std::vector<cv::KeyPoint> train_pts, int width) {
    std::vector<float> dxs;
    std::vector<cv::DMatch> good_matches;

    for (auto gm : matches) {
        dxs.push_back(query_pts[gm.queryIdx].pt.x - train_pts[gm.trainIdx].pt.x);
    }

    std::sort(dxs.begin(), dxs.end());
    int mid_dx = (int) dxs[dxs.size()/2];
    bool crop_width = fabs(mid_dx) > CROP_WIDTH_THRES*width;

    printf("MIN DX %f, MID %f END %f\n", dxs[0], dxs[dxs.size()/2], dxs[dxs.size() - 1]);

    if (crop_width) {
        if (mid_dx > 0) {
            printf("Cropping img1 %d->%d img2 %d->%d\n", width - mid_dx, width, 0, mid_dx);
        } else {
            printf("Cropping img1 %d->%d img2 %d->%d\n", 0, - mid_dx, width+ mid_dx, width);
        }
    }

    for (auto gm : matches) {
        bool use_match = true;
        double pt1x = train_pts[gm.trainIdx].pt.x;
        double pt2x = query_pts[gm.queryIdx].pt.x;
        if (crop_width) {
            if (mid_dx > 0) {
                if(pt1x > width - mid_dx || pt2x < mid_dx) {
                    use_match = false;
                }
            }

            if (mid_dx < 0) {
                if (pt1x < - mid_dx || pt2x > width + mid_dx) {
                    use_match = false;
                }
            }
        }
        if (use_match) {
            good_matches.push_back(gm);
        }
    }
    return good_matches;

}

inline std::vector<cv::KeyPoint> to_keypoints_with_max_height(const std::vector<cv::Point2f> & pts, int max_y) {
    std::vector<cv::KeyPoint> kps;
    for (auto pt : pts) {
        cv::KeyPoint kp;
        kp.pt = pt;
        if (pt.y < max_y) {
            kps.push_back(kp);
        }
    }
    return kps;
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

    std::vector<cv::Point3f> matched_3d_now;
    std::vector<cv::Point2f> matched_2d_norm_old;

    for (auto match : _matches) {
        int now_id = match.queryIdx;
        int old_id = match.trainIdx;
        matched_3d_now.push_back(now_3d[now_id]);
        matched_2d_norm_old.push_back(old_norm_2d[old_id]);
    }
 
    if(_matches.size() > MIN_LOOP_NUM || (init_mode && matches.size() > INIT_MODE_MIN_LOOP_NUM  )) {
        //Compute PNP

        cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);

        cv::Mat r, rvec, t, D, tmp_r;
        cv::Mat inliers;

        //TODO: Prepare initial pose for swarm

        Swarm::Pose initial_old_drone_pose = drone_pose_now;
        // }

        Swarm::Pose initial_old_cam_pose = initial_old_drone_pose * old_extrinsic;

        PnPInitialFromCamPose(initial_old_cam_pose, rvec, t);
        
        int iteratives = 100;

        if (init_mode) {
            iteratives = 1000;
        }

        bool success = solvePnPRansac(matched_3d_now, matched_2d_norm_old, K, D, rvec, t, true,            iteratives,        PNP_REPROJECT_ERROR/200,     0.995,  inliers, cv::SOLVEPNP_DLS);
        auto p_cam_old_in_new = PnPRestoCamPose(rvec, t);
        auto p_drone_old_in_new = p_cam_old_in_new*(old_extrinsic.to_isometry().inverse());
        

        Swarm::Pose DP_old_to_new_6d =  Swarm::Pose::DeltaPose(p_drone_old_in_new, drone_pose_now, false);
        DP_old_to_new =  Swarm::Pose::DeltaPose(p_drone_old_in_new, drone_pose_now, true);
        //As our pose graph uses 4D pose, here we must solve 4D pose
        //6D pose could use to verify the result but not give to swarm_localization
        std::cout << "PnP solved DPose 4d";
        DP_old_to_new.print();
        
        auto RPerr = RPerror(p_drone_old_in_new, drone_pose_old, drone_pose_now);

        success = pnp_result_verify(success, init_mode, inliers.rows, RPerr, DP_old_to_new);

        ROS_INFO("PnPRansac %d inlines %d, dyaw %f dpos %f", success, inliers.rows, fabs(DP_old_to_new.yaw())*57.3, DP_old_to_new.pos().norm());
        inlier_num = inliers.rows;

        for (int i = 0; i < inlier_num; i++) {
            int idx = inliers.at<int>(i);
            matches.push_back(_matches[idx]);
        }
        return success;
    }

    return 0;
}


bool LoopDetector::compute_loop(const ImageDescriptor_t & new_img_desc, const ImageDescriptor_t & old_img_desc, cv::Mat img_new, cv::Mat img_old, LoopConnection & ret, bool init_mode) {

    if (new_img_desc.landmark_num < MIN_LOOP_NUM) {
        return false;
    }
    //Recover imformation

    assert(old_img_desc.drone_id == self_id && "old img desc must from self drone!");

    ROS_INFO("Compute loop %d->%d", old_img_desc.drone_id, new_img_desc.drone_id);

    bool success = false;
    Swarm::Pose  DP_old_to_new;

    bool first_try_match_mode = false;

    ROS_INFO("Try solve %d->%d LANDMARK from %d, num %d, with Match Mode %d Init %d", old_img_desc.drone_id, new_img_desc.drone_id, 
        new_img_desc.drone_id, 
        new_img_desc.landmark_num,
        first_try_match_mode, init_mode);

    auto now_2d = toCV(new_img_desc.landmarks_2d);
    auto now_norm_2d = toCV(new_img_desc.landmarks_2d_norm);
    auto now_3d = toCV(new_img_desc.landmarks_3d);
    ROS_INFO("New desc %ld/%ld", new_img_desc.landmarks_2d.size(), new_img_desc.feature_descriptor.size());

    cv::Mat desc_now( new_img_desc.landmarks_2d.size(), LOCAL_DESC_LEN, CV_32F);
    memcpy(desc_now.data, new_img_desc.feature_descriptor.data(), new_img_desc.feature_descriptor.size()*sizeof(float));

    auto old_2d = toCV(old_img_desc.landmarks_2d);
    auto old_norm_2d = toCV(old_img_desc.landmarks_2d_norm);
    auto old_3d = toCV(old_img_desc.landmarks_3d);
    cv::Mat desc_old( old_img_desc.landmarks_2d.size(), LOCAL_DESC_LEN, CV_32F);
    memcpy(desc_old.data, old_img_desc.feature_descriptor.data(), old_img_desc.feature_descriptor.size()*sizeof(float));

    std::vector<cv::DMatch> matches;

    ROS_INFO("Will compute relative pose");

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
        cv::resize(show, show, cv::Size(), 2, 2);
         if (success) {
            sprintf(title, "SUCCESS LOOP %d->%d inliers %d", old_img_desc.drone_id, new_img_desc.drone_id, inlier_num);
            cv::putText(show, title, cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 3);
        } else {
            sprintf(title, "FAILED LOOP %d->%d inliers %d", old_img_desc.drone_id, new_img_desc.drone_id, inlier_num);
            cv::putText(show, title, cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 3);
        }

        cv::imshow("Matches", show);
        cv::waitKey(10);
    }

    if (success) {
        /*
        std::cout << "CamPoseOLD             ";
        initial_old_cam_pose.print();
        std::cout << "PnP solved camera Pose ";
        p_cam_old_in_new.print();
        */


        ret.dpos.x = DP_old_to_new.pos().x();
        ret.dpos.y = DP_old_to_new.pos().y();
        ret.dpos.z = DP_old_to_new.pos().z();
        ret.dyaw = DP_old_to_new.yaw();

        ret.id_a = old_img_desc.drone_id;
        ret.ts_a = toROSTime(old_img_desc.timestamp);

        ret.id_b = new_img_desc.drone_id;
        ret.ts_b = toROSTime(new_img_desc.timestamp);

        ret.self_pose_a = toROSPose(old_img_desc.pose_drone);
        ret.self_pose_b = toROSPose(new_img_desc.pose_drone);

        return true;
    }
    return false;

}

void LoopDetector::on_loop_connection(LoopConnection & loop_conn) {
    on_loop_cb(loop_conn);
}

#ifdef USE_DEEPNET
LoopDetector::LoopDetector(): index(DEEP_DESC_SIZE) {
    cv::RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(cv::Scalar(r,g,b));
    }
}
#else
LoopDetector::LoopDetector(const std::string & voc_path):
    voc(voc_path), db(voc, false, 0) {
    cv::RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(cv::Scalar(r,g,b));
    }
}
#endif



