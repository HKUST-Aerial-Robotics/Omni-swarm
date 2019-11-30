#include <loop_detector.h>
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <opencv2/opencv.hpp>
#include <chrono> 
#include <opencv2/core/eigen.hpp>
#include <swarm_msgs/Pose.h>

using namespace std::chrono; 

void LoopDetector::on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img) {
    auto start = high_resolution_clock::now(); 

    cv::Mat feature = cvfeatureFromByte((uint8_t*)img_des.feature_descriptor, LOOP_FEATURE_NUM);
    // std::cout << "FEATUREX"<< featurex.size() << std::endl;
    if (img_des.landmark_num >= MIN_LOOP_NUM) {

        int _id = db.add(feature);
        std::cout << "Add Time cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 <<"ms" << std::endl;
        id2imgdes[_id] = img_des;
        if (!img.empty() ) {
            id2imgs[_id] = img;
        }

        ROS_INFO("Adding image descriptor %d to database", _id);
        bool success = false;

        if (db.size() > MATCH_INDEX_DIST) {

            DBoW3::QueryResults ret;
            ROS_INFO("Querying image....");

            db.query(feature, ret, 1, db.size() - MATCH_INDEX_DIST);
            auto stop = high_resolution_clock::now(); 

            if (ret.size() > 0 && ret[0].Score > LOOP_BOW_THRES) {
                std::cout << "Time Cost " << duration_cast<microseconds>(stop - start).count()/1000.0 <<"ms RES: " << ret << std::endl;
                
                if (ret.size() > 0) {
                    int _old_id = ret[0].Id;
                    LoopConnection ret;
                    success = compute_loop(img_des, _old_id, ret);
                    if (success) {
                        on_loop_connection(ret);
                    }
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


bool LoopDetector::compute_loop(const ImageDescriptor_t & new_img_desc, const unsigned int & _img_index_old, LoopConnection & ret) {

    if (new_img_desc.landmark_num < MIN_LOOP_NUM) {
        return false;
    }
    //Recover imformation

    ImageDescriptor_t old_img_desc = id2imgdes[_img_index_old];

    std::vector<cv::Point2f> matched_2d_norm_now, matched_2d_norm_old;
    std::vector<cv::Point3f> matched_3d_now;


    auto img_old_small = decode_image(old_img_desc);
    auto img_new_small = decode_image(new_img_desc);
   
    std::vector<cv::Point2f> tracked;// = nowPts;
    std::vector<float> err;
    std::vector<unsigned char> status;
 
    //Prepare points
    auto nowPts = loop_cam->project_to_image(
        toCV(new_img_desc.landmarks_2d_norm));
    std::vector<cv::Point2f> nowPtsSmall;

    for (auto pt : nowPts) {
        pt.x = pt.x/LOOP_IMAGE_DOWNSAMPLE;
        pt.y = pt.y/LOOP_IMAGE_DOWNSAMPLE;
        nowPtsSmall.push_back(pt);
    }

    //Preform Optical Flow
    auto start = high_resolution_clock::now();    
    cv::calcOpticalFlowPyrLK(img_new_small, img_old_small, nowPtsSmall, tracked, status, err, cv::Size(21, 21), 3);
    std::cout << "OPTICAL FLOW Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;

    std::vector<cv::Point2f> good_new;
    
    for(uint i = 0; i < nowPts.size(); i++) {
        if(status[i] == 1) {
            matched_2d_norm_now.push_back(toCV(new_img_desc.landmarks_2d_norm[i]));
            matched_3d_now.push_back(toCV(new_img_desc.landmarks_3d[i]));
            Eigen::Vector2d p_tracker(tracked[i].x, tracked[i].y);
            matched_2d_norm_old.push_back(loop_cam->project_to_norm2d(tracked[i]*LOOP_IMAGE_DOWNSAMPLE));
        }
    }
    

    if(matched_3d_now.size() > MIN_LOOP_NUM) {
        //Compute PNP

        cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);

        cv::Mat r, rvec, t, D, tmp_r;
        cv::Mat inliers;

        //TODO: Prepare initial pose for swarm
        Swarm::Pose old_extrinsic(old_img_desc.camera_extrinsic);
        Swarm::Pose drone_pose_now(new_img_desc.pose_drone);

        Swarm::Pose initial_old_drone_pose(old_img_desc.pose_drone);

        Swarm::Pose initial_old_cam_pose = initial_old_drone_pose * old_extrinsic;

        PnPInitialFromCamPose(initial_old_cam_pose, rvec, t);
        
        start = high_resolution_clock::now();

        // std::cout << "Init R " << rvec << " T" << t << std::endl;
        bool success = solvePnPRansac(matched_3d_now, matched_2d_norm_old, K, D, rvec, t, true, 100, 10.0 / 460.0, 0.99,  inliers);

        auto p_cam_old_in_new = PnPRestoCamPose(rvec, t);

        auto p_drone_old_in_new = p_cam_old_in_new*old_extrinsic.to_isometry().inverse();

        //We use yaw only DPose in pose graph
        Swarm::Pose DP_old_to_new = Swarm::Pose::DeltaPose(p_drone_old_in_new, drone_pose_now, true);

        std::cout << "SolvePnP Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 << "ms" << std::endl;

        success = success && (inliers.rows > MIN_LOOP_NUM) && fabs(DP_old_to_new.yaw()) < ACCEPT_LOOP_YAW_RAD && DP_old_to_new.pos().norm() < MAX_LOOP_DIS;

        if (success) {
            /*
            std::cout << "CamPoseOLD             ";
            initial_old_cam_pose.print();
            std::cout << "PnP solved camera Pose ";
            p_cam_old_in_new.print();
            */

            std::cout << "DRONE POSE OLD  ";
            initial_old_drone_pose.print();
            std::cout << "DRONE POSE NOW  ";
            drone_pose_now.print();

            std::cout << "Initial DPose    ";
            Swarm::Pose::DeltaPose(initial_old_drone_pose, drone_pose_now, true).print();
            std::cout << "PnP solved DPose ";
            DP_old_to_new.print();

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

        //Show Result
        if (enable_visualize) {
            cv::cvtColor(img_new_small, img_new_small, cv::COLOR_GRAY2BGR);
            cv::cvtColor(img_old_small, img_old_small, cv::COLOR_GRAY2BGR);
            
            std::vector<cv::DMatch> matches;
            std::vector<cv::KeyPoint> oldKPs;
            matches.clear();

            auto nowKPs = cvPoints2Keypoints(loop_cam->project_to_image(
                toCV(new_img_desc.landmarks_2d_norm)));

            for(uint i = 0; i < nowKPs.size(); i++)
            {
                if(status[i] == 1) {
                    good_new.push_back(tracked[i]);
                    // draw the tracks
                    cv::KeyPoint kp;
                    kp.pt = tracked[i];
                    cv::KeyPoint kp2;
                    kp2.pt = nowPts[i];
                    // nowKPs.push_back(kp2);

                    cv::line(img_new_small, nowPtsSmall[i], tracked[i], colors[i], 2);
                    cv::circle(img_new_small, nowPtsSmall[i], 5, colors[i], -1);
                    
                    oldKPs.push_back(kp);
                    cv::DMatch dmatch;
                    dmatch.queryIdx = i;
                    dmatch.trainIdx = oldKPs.size() - 1;
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
            std::cout << "NOWLPS size " << nowKPs.size() << " OLDLPs " << oldKPs.size() << std::endl;
            cv::drawMatches(img_new_small, cvPoints2Keypoints(nowPtsSmall), img_old_small, oldKPs, good_matches, _show);
            cv::resize(_show, _show, _show.size()*2);

            if (success) {
	            cv::putText(_show, "SUCCESS LOOP", cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 3);
            } else {
	            cv::putText(_show, "FAILED LOOP", cv::Point2f(20, 30), CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 3);
            }


            cv::imshow("Track Loop", _show);
            cv::waitKey(10);
        }

        return success;
    }
    return false;

}

void LoopDetector::on_loop_connection(LoopConnection & loop_conn) {
    on_loop_cb(loop_conn);
}

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