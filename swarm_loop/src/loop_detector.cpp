#include <loop_detector.h>
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <opencv2/opencv.hpp>
#include <chrono> 
#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std::chrono; 

void LoopDetector::on_image_recv(const ImageDescriptor_t & img_des) {
    auto start = high_resolution_clock::now(); 
    if (img_des.drone_id!= this->self_id && database_size() == 0) {
        ROS_INFO("Empty local database, where giveup remote image");
        return;
    } 

    all_nodes.insert(self_id);

    if (img_des.landmark_num >= MIN_LOOP_NUM) {
        std::cout << "Add Time cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 <<"ms" << std::endl;
        bool nothis_node = all_nodes.find(img_des.drone_id) == all_nodes.end();

        if (!img_des.prevent_adding_db || nothis_node) {
            int _id = add_to_database(img_des);
            id2imgdes[_id] = img_des;
        } else {
            ROS_INFO("This image is prevent to adding to DB");
        }

        bool success = false;
        bool init_mode = nothis_node;

        if (database_size() > MATCH_INDEX_DIST || init_mode) {

            ROS_INFO("Querying image from database size %d....", database_size());
            int _old_id = -1;
            if (nothis_node) {
                _old_id = query_from_database(img_des, 1, init_mode);
            } else {
                _old_id = query_from_database(img_des, MATCH_INDEX_DIST);
            }

            auto stop = high_resolution_clock::now(); 

            if (_old_id >= 0 ) {
                std::cout << "Time Cost " << duration_cast<microseconds>(stop - start).count()/1000.0 <<"ms RES: " << _old_id << std::endl;                
                LoopConnection ret;
                success = compute_loop(img_des, _old_id, ret, init_mode);
                if (success) {
                    ROS_INFO("Adding success matched drone %d to database", img_des.drone_id);
                    all_nodes.insert(img_des.drone_id);
                    on_loop_connection(ret);
                }
            } else {
                std::cout << "No matched image" << std::endl;
            }      
        } 

        std::cout << "LOOP Detector cost" << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 <<"ms" << std::endl;
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
    auto ret = cv::imdecode(_img_desc.image, cv::IMREAD_GRAYSCALE);
    std::cout << "IMDECODE Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;

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

int LoopDetector::query_from_database(const ImageDescriptor_t & img_desc, int max_index, bool init_mode) {
#ifdef USE_DEEPNET
    float distances[SEARCH_NEAREST_NUM] = {0};
    faiss::Index::idx_t labels[SEARCH_NEAREST_NUM];
    index.search(1, img_desc.image_desc.data(), SEARCH_NEAREST_NUM, distances, labels);
    
    //Must query or return has self id
    bool return_must_self_id = (img_desc.drone_id != self_id);

    double thres = INNER_PRODUCT_THRES;
    if (init_mode) {
        thres = INIT_MODE_PRODUCT_THRES;
    }
    for (int i = 0; i < SEARCH_NEAREST_NUM; i++) {
        int return_drone_id = id2imgdes.at(labels[i]).drone_id;
        if (labels[i] < database_size() - max_index && distances[i] > thres) {
            // ROS_INFO("Suitable Find %ld, radius %f", labels[i], distances[i]);
            if ((return_must_self_id && return_drone_id == self_id) || !return_must_self_id) {
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


std::vector<cv::KeyPoint> detect_orb_by_region(cv::Mat _img, int features, int cols = 4, int rows = 3) {
    int small_width = _img.cols / cols;
    int small_height = _img.rows / rows;
    printf("Cut to W %d H %d for FAST\n", small_width, small_height);
    
    auto _orb = cv::ORB::create(features/(cols*rows));
    std::vector<cv::KeyPoint> ret;
    for (int i = 0; i < cols; i ++) {
        for (int j = 0; j < rows; j ++) {
            std::vector<cv::KeyPoint> kpts;
            _orb->detect(_img(cv::Rect(small_width*i, small_width*j, small_width, small_height)), kpts);
            printf("Detect %ld feature in reigion %d %d\n", kpts.size(), i, j);

            for (auto kp : kpts) {
                kp.pt.x = kp.pt.x + small_width*i;
                kp.pt.y = kp.pt.y + small_width*j;
                ret.push_back(kp);
            }
        }
    }

    return ret;
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

std::vector<cv::DMatch> filter_by_hamming(const std::vector<cv::DMatch> & matches) {
    std::vector<cv::DMatch> good_matches;
    std::vector<float> dys;
    for (auto gm : matches) {
        dys.push_back(gm.distance);
    }

    std::sort(dys.begin(), dys.end());

    // printf("MIN DX DIS:%f, 2min %fm ax %f\n", dys[0], 2*dys[0], dys[dys.size() - 1]);

    double max_hamming = 2*dys[0];
    if (max_hamming < ORB_HAMMING_DISTANCE) {
        max_hamming = ORB_HAMMING_DISTANCE;
    }
    for (auto gm: matches) {
        if (gm.distance < max_hamming) {
            good_matches.push_back(gm);
        }
    }

    return good_matches;
}


std::vector<cv::DMatch>  replace_match_id(const std::vector<cv::DMatch> & matches, std::map<int, int> real_id1) {
    std::vector<cv::DMatch> good_matches;
    for (auto gm : matches) {
        gm.trainIdx = real_id1[gm.trainIdx];
        good_matches.push_back(gm);
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


void LoopDetector::find_correspoding_pts(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> Pts1, std::vector<cv::Point2f> &tracked, std::vector<unsigned char> & status, bool init_mode, bool visualize) {

    std::vector<cv::KeyPoint> kps1, kps2;
    std::vector<cv::DMatch> good_matches;
    std::map<int, int> real_id1;
    bool use_surf = false;

    std::vector<cv::Point2f> new_tracked;

    // Basically we are initial on ground, so we can use 2/3 precent plane for init
    int no_gnd_height = img2.rows;
    if (init_mode) {
        no_gnd_height = img2.rows * AVOID_GROUND_PRECENT;
    } 

    cv::goodFeaturesToTrack(img2(cv::Rect(0, 0, img2.cols, no_gnd_height)), new_tracked, 500, 0.01, 5/LOOP_IMAGE_DOWNSAMPLE);
    ROS_INFO("GFTT gives %ld new track", new_tracked.size());
    kps2 = to_keypoints(new_tracked);
    kps1 = to_keypoints_with_max_height(Pts1, no_gnd_height);

    
    auto _orb = cv::ORB::create(1000, 1.2f, 8, 31, 0, 4, cv::ORB::HARRIS_SCORE, 31, 20);
    cv::Mat desc1, desc2;
    _orb->compute(img2, kps2, desc2);
    // _orb->detectAndCompute(img2, cv::Mat(), kps2, desc2);
    // _orb->detectAndCompute(img1, cv::Mat(), kps1, desc1, true);
    _orb->compute(img1, kps1, desc1);

    size_t j = 0;

    //Kps1
    for (size_t i = 0; i < kps1.size(); i ++) {
        while ( j < Pts1.size() && cv::norm(Pts1[j] - kps1[i].pt) > 1) {
            j++;
        }
        real_id1[i] = j;
    }
    cv::BFMatcher bfmatcher(cv::NORM_HAMMING2, true);
    std::vector<cv::DMatch> matches;
    bfmatcher.match(desc2, desc1, matches);
    // printf("ORIGIN MATCHES %ld\n", matches.size());
    matches = filter_by_hamming(matches);
    // printf("AFTER HAMMING X MATCHES %ld\n", matches.size());
    kps1 = to_keypoints(Pts1);
    matches = replace_match_id(matches, real_id1);
    
    
    matches = filter_by_duv(matches, kps2, kps1);
    // printf("AFTER DUV MATCHES %ld\n", matches.size());

    double thres = OUTLIER_XY_PRECENT_0;
    if (matches.size() > 40 ) {
        thres = OUTLIER_XY_PRECENT_40;
    } else if (matches.size() > 30) {
        thres = OUTLIER_XY_PRECENT_30;
    } else if (matches.size() > 20) {
        thres = OUTLIER_XY_PRECENT_20;
    }
    
    matches = filter_by_x(matches, kps2, kps1, thres);
    // printf("AFTER DX MATCHES %ld\n", matches.size());
    matches = filter_by_y(matches, kps2, kps1, thres);
    // printf("AFTER DY MATCHES %ld\n", matches.size());
    good_matches = matches;

    
    tracked = std::vector<cv::Point2f> (Pts1.size());
    status = std::vector<unsigned char>(Pts1.size());
    std::fill(status.begin(), status.end(), 0);



    for (auto gm : good_matches) {
        // printf("HAMMING DIS %f UV DIS %f\n", gm.distance, cv::norm(kps2[gm.trainIdx].pt - Pts1[real_id1[gm.queryIdx]]));        
        // printf("HAMMING DIS %f UV DIS %f\n", gm.distance, cv::norm(kps2[gm.queryIdx].pt - Pts1[real_id1[gm.trainIdx]]));
        auto _id1 = gm.trainIdx;
        auto _id2 = gm.queryIdx;
        tracked[_id1] = kps2[_id2].pt;
        status[_id1] = 1;
    }

    if (visualize) {
        kps1.clear();
        for (auto pt: Pts1) {
            cv::KeyPoint kp;
            kp.pt = pt;
            kps1.push_back(kp);
        }
        ROS_INFO("Good matches : %ld", good_matches.size());
        cv::Mat _show;
	    cv::putText(img2, "IMG2", cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 3);
	    cv::putText(img1, "IMG1", cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 3);
        
        cv::drawMatches(img2, kps2, img1, kps1, good_matches, _show);
        // cv::drawMatches(img1, kps1, img2, kps2, good_matches, _show);
        cv::resize(_show, _show, cv::Size(), VISUALIZE_SCALE, VISUALIZE_SCALE);
        cv::imshow("KNNMatch", _show);
        cv::waitKey(10);
    }
}

bool pnp_result_verify(bool pnp_success, bool init_mode, int inliers, const Swarm::Pose & DP_old_to_new) {
    bool success = pnp_success;
    if (!pnp_success) {
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
bool LoopDetector::compute_relative_pose(cv::Mat & img_new_small, cv::Mat & img_old_small, const std::vector<cv::Point2f> & nowPtsSmall, 
        const std::vector<cv::Point2f> now_norm_2d,
        const std::vector<cv::Point3f> now_3d,
        Swarm::Pose old_extrinsic,
        Swarm::Pose drone_pose_now,
        Swarm::Pose & DP_old_to_new,
        bool init_mode,
        bool use_orb_matching) {
        //Preform Optical Flow
    std::vector<cv::Point2f> tracked;// = nowPts;
    std::vector<float> err;
    std::vector<unsigned char> status;
    std::vector<cv::Point2f> matched_2d_norm_now, matched_2d_norm_old;
    std::vector<cv::Point3f> matched_3d_now;

    auto start = high_resolution_clock::now();    
    if (use_orb_matching) {
        find_correspoding_pts(img_new_small, img_old_small, nowPtsSmall, tracked, status, init_mode, enable_visualize);
    } else {
        cv::calcOpticalFlowPyrLK(img_new_small, img_old_small, nowPtsSmall, tracked, status, err, cv::Size(21, 21), 3);

        std::vector<float> uv_dis;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i]) {
                uv_dis.push_back(cv::norm(tracked[i] - nowPtsSmall[i]));
            }
        }


        std::sort(uv_dis.begin(), uv_dis.end());

        double mid_dis = uv_dis[uv_dis.size()/2];

        printf("MIN UV DIS %f, MID %f END %f\n", uv_dis[0], uv_dis[uv_dis.size()/2], uv_dis[uv_dis.size() - 1]);
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i] && cv::norm(tracked[i] - nowPtsSmall[i]) > mid_dis*ORB_UV_DISTANCE) {
                status[i] = 0;
            }
        }
    }

    std::vector<cv::Point2f> good_new;
    
    for(uint i = 0; i < now_3d.size(); i++) {
        if(status[i] == 1) {
            matched_2d_norm_now.push_back(now_norm_2d[i]);
            matched_3d_now.push_back(now_3d[i]);
            Eigen::Vector2d p_tracker(tracked[i].x, tracked[i].y);

            //If old is remote drone may course reproject error here; We
            matched_2d_norm_old.push_back(loop_cam->project_to_norm2d(tracked[i]*LOOP_IMAGE_DOWNSAMPLE));
        }
    }

    double dt = duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0;
    ROS_INFO("Matcher traced %ld/%ld cost %fms", matched_2d_norm_old.size(), nowPtsSmall.size(), dt);

    if(matched_3d_now.size() > MIN_LOOP_NUM || (init_mode && matched_3d_now.size() > INIT_MODE_MIN_LOOP_NUM  )) {
        //Compute PNP

        cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);

        cv::Mat r, rvec, t, D, tmp_r;
        cv::Mat inliers;

        //TODO: Prepare initial pose for swarm

        Swarm::Pose initial_old_drone_pose = drone_pose_now;
        // }

        Swarm::Pose initial_old_cam_pose = initial_old_drone_pose * old_extrinsic;

        PnPInitialFromCamPose(initial_old_cam_pose, rvec, t);
        
        start = high_resolution_clock::now();

        int iteratives = 100;
        if (init_mode) {
            iteratives = 1000;
        }
        bool success = solvePnPRansac(matched_3d_now, matched_2d_norm_old, K, D, rvec, t, true,            iteratives,        PNP_REPROJECT_ERROR/ img_new_small.rows,     0.995,  inliers, cv::SOLVEPNP_DLS);
        auto p_cam_old_in_new = PnPRestoCamPose(rvec, t);
        auto p_drone_old_in_new = p_cam_old_in_new*old_extrinsic.to_isometry().inverse();
        
        Swarm::Pose DP_old_to_new = Swarm::Pose::DeltaPose(p_drone_old_in_new, drone_pose_now, true);
        
        success = pnp_result_verify(success, init_mode, inliers.rows, DP_old_to_new);


        if (!success && init_mode) {
            double dt = duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0;
            ROS_WARN("Solve pnp failed cost %fms: inliers %d; retry with iterative", dt, inliers.rows);
            std::cout << "Failed PnP solved DPose ";
            DP_old_to_new.print();

            inliers = cv::Mat();
            success = solvePnPRansac(matched_3d_now, matched_2d_norm_old, K, D, rvec, t, true,            iteratives,        PNP_REPROJECT_ERROR / img_new_small.rows,     0.995,  inliers,  cv::SOLVEPNP_ITERATIVE);
            p_cam_old_in_new = PnPRestoCamPose(rvec, t);
            p_drone_old_in_new = p_cam_old_in_new*old_extrinsic.to_isometry().inverse();
            DP_old_to_new = Swarm::Pose::DeltaPose(p_drone_old_in_new, drone_pose_now, true);
            success = pnp_result_verify(success, init_mode, inliers.rows, DP_old_to_new);
            if (success) {
                ROS_INFO("DLS Success\n");
            }
        }

        double dt = duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0;

        ROS_INFO("PnPRansac %d solve %fms inlines %d, dyaw %f dpos %f", success, dt, inliers.rows, fabs(DP_old_to_new.yaw())*57.3, DP_old_to_new.pos().norm());

        // initial_old_drone_pose.print();
        // std::cout << "DRONE POSE NOW  ";
        // drone_pose_now.print();

        // std::cout << "Initial DPose    ";
        Swarm::Pose::DeltaPose(initial_old_drone_pose, drone_pose_now, true).print();
        std::cout << "PnP solved DPose ";
        DP_old_to_new.print();

        printf("\n\n\n");

        //Show Result
        if (enable_visualize) {
            // std::cout << "INLIERS" << inliers << std::endl;
            cv::Mat img_new, img_old;
            cv::cvtColor(img_new_small, img_new, cv::COLOR_GRAY2BGR);
            cv::cvtColor(img_old_small, img_old, cv::COLOR_GRAY2BGR);
            
            std::vector<cv::DMatch> matches;
            std::vector<cv::KeyPoint> oldKPs;
            matches.clear();

            for( size_t i = 0; i < nowPtsSmall.size(); i++)
            {
                if(status[i] == 1) {
                    good_new.push_back(tracked[i]);
                    // draw the tracks
                    cv::KeyPoint kp;
                    kp.pt = tracked[i];
                    // nowKPs.push_back(kp2);

                    cv::line(img_new, nowPtsSmall[i], tracked[i], colors[i], 2);
                    cv::circle(img_new, nowPtsSmall[i], 5, colors[i], -1);
                    
                    oldKPs.push_back(kp);
                    cv::DMatch dmatch;
                    dmatch.queryIdx = oldKPs.size() - 1;
                    dmatch.trainIdx = i;
                    matches.push_back(dmatch);
                }
            }

            std::vector<cv::DMatch> good_matches;

            for( int i = 0; i < inliers.rows; i++)
            {
                int n = inliers.at<int>(i);
                good_matches.push_back(matches[n]);
            }

            cv::Mat _show;
            std::cout << "NOWLPS size " << nowPtsSmall.size() << " OLDLPs " << oldKPs.size() << std::endl;
            if (success) {
                cv::drawMatches(img_old, oldKPs, img_new, cvPoints2Keypoints(nowPtsSmall), good_matches, _show);
            } else {
                cv::drawMatches(img_old, oldKPs, img_new, cvPoints2Keypoints(nowPtsSmall), matches, _show);
            }
            cv::resize(_show, _show, _show.size()*VISUALIZE_SCALE);

            char title[100] = {0};
            if (success) {
                sprintf(title, "SUCCESS LOOP inliers %d", inliers.rows);
	            cv::putText(_show, title, cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 3);
            } else {
                sprintf(title, "FAILED LOOP inliers %d", inliers.rows);
	            cv::putText(_show, title, cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 3);
            }

            cv::imshow("Track Loop", _show);
            if (success) {
                cv::waitKey(100);
            } else {
                cv::waitKey(100);
            }
        }

        return success;
    } else {
        printf("Loop failed\n\n\n");
        return false;
    }

    return false;
}

bool LoopDetector::compute_loop(const ImageDescriptor_t & new_img_desc, const unsigned int & _img_index_old, LoopConnection & ret, bool init_mode) {

    if (new_img_desc.landmark_num < MIN_LOOP_NUM) {
        return false;
    }
    //Recover imformation
    ImageDescriptor_t old_img_desc = id2imgdes[_img_index_old];

    ROS_INFO("Compute loop %d->%d", old_img_desc.drone_id, new_img_desc.drone_id);



    auto img_old_small = decode_image(old_img_desc);
    auto img_new_small = decode_image(new_img_desc);
   
  
    auto nowPts = toCV(new_img_desc.landmarks_2d);
    std::vector<cv::Point2f> nowPtsSmall;

    for (auto pt : nowPts) {
        pt.x = pt.x/LOOP_IMAGE_DOWNSAMPLE;
        pt.y = pt.y/LOOP_IMAGE_DOWNSAMPLE;
        // std::cout << "now pts" << pt;
        nowPtsSmall.push_back(pt);
    }

    bool success = false;
    Swarm::Pose  DP_old_to_new;

    bool first_try_match_mode = false;
    /*(cv::Mat & img_new_small, cv::Mat & img_old_small, const std::vector<cv::Point2f> & nowPtsSmall, 
        const std::vector<cv::Point2f> now_norm_2d,
        const std::vector<cv::Point3f> now_3d,
        Swarm::Pose old_extrinics,
        Swarm::Pose drone_pose_now,
        Swarm::Pose & DP_old_to_new,
        bool init_mode,
        bool use_orb_matching) */
    if (init_mode) {
        first_try_match_mode = true;
    }

    ROS_INFO("Try solve %d->%d LANDMARK from %d, num %d, with Match Mode %d Init %d", old_img_desc.drone_id, new_img_desc.drone_id, 
        new_img_desc.drone_id, 
        new_img_desc.landmark_num,
        first_try_match_mode, init_mode);
        success = compute_relative_pose(img_new_small, img_old_small, nowPtsSmall, 
            toCV(new_img_desc.landmarks_2d_norm), toCV(new_img_desc.landmarks_3d), 
            Swarm::Pose(old_img_desc.camera_extrinsic),
            Swarm::Pose(new_img_desc.pose_drone),
            DP_old_to_new,
            init_mode,
            first_try_match_mode
    );

    // if (!success) {
    //     ROS_WARN("First try  failed, try second time");

    //     success = compute_relative_pose(img_new_small, img_old_small, nowPtsSmall, 
    //         toCV(new_img_desc.landmarks_2d_norm), toCV(new_img_desc.landmarks_3d), 
    //         Swarm::Pose(old_img_desc.camera_extrinsic),
    //         Swarm::Pose(new_img_desc.pose_drone),
    //         DP_old_to_new,
    //         init_mode,
    //         !first_try_match_mode
    //     );

    //     if (!success) {
    //         ROS_WARN("Second try with match mode %d failed", !first_try_match_mode);
    //     }
    // }


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



