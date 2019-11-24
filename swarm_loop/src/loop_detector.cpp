#include <loop_detector.h>
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <opencv2/opencv.hpp>
#include <chrono> 
using namespace std::chrono; 

bool searchInAera(  uint8_t * window_descriptor,
                    uint8_t * descriptors_old,
                    int & bestIndex);

void searchByBRIEFDes(  std::vector<cv::Point2f> &matched_2d_old_norm,
                        std::vector<cv::Point2f> &matched_2d_new_norm,
                        std::vector<cv::Point3f> &matched_3d_new,
                        uint8_t * descriptors_old,
                        uint8_t * descriptors_now,
                        const Point2d_t * keypoints_old_norm,
                        const std::vector<Point2d_t> &keypoints_new_norm,
                        const std::vector<Point3d_t> &keypoints_new_3d);

void searchByBRIEFDesCV(  std::vector<cv::Point2f> &matched_2d_old_norm,
                        std::vector<cv::Point2f> &matched_2d_new_norm,
                        std::vector<cv::Point3f> &matched_3d_new,
                        uint8_t * descriptors_old,
                        uint8_t * descriptors_now,
                        const Point2d_t * keypoints_old_norm,
                        const std::vector<Point2d_t> &keypoints_new_norm,
                        const std::vector<Point3d_t> &keypoints_new_3d);

void LoopDetector::on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img) {
    auto start = high_resolution_clock::now(); 

    cv::Mat feature = cvfeatureFromByte((uint8_t*)img_des.feature_descriptor, LOOP_FEATURE_NUM);
    // std::cout << "FEATUREX"<< featurex.size() << std::endl;

    int _id = db.add(feature);
    std::cout << "Add Time cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 <<"ms" << std::endl;
    
    id2imgdes[_id] = img_des;

    if (!img.empty() ) {
        id2imgs[_id] = img;
    }

    if (db.size() > MATCH_INDEX_DIST) {

        DBoW3::QueryResults ret;
        ROS_INFO("Querying image....");

        db.query(feature, ret, 1, _id - MATCH_INDEX_DIST);
        auto stop = high_resolution_clock::now(); 

        if (ret.size() > 0 && ret[0].Score > LOOP_BOW_THRES) {
            std::cout << "Time Cost " << duration_cast<microseconds>(stop - start).count()/1000.0 <<"ms RES: " << ret << std::endl;
            
            if (ret.size() > 0) {
                int _old_id = ret[0].Id;
                LoopConnection ret;
                bool success = compute_loop(_id, _old_id, ret);
                if (success) {
                    on_loop_connection(ret);
                }
            }
        } else {
            std::cout << "No matched image" << std::endl;
        }      

    }



    ROS_INFO("Adding image descriptor %d to database", _id);

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

bool LoopDetector::compute_loop(const unsigned int & _img_index_now, const unsigned int & _img_index_old, LoopConnection & ret) {
    ImageDescriptor_t old_img_desc = id2imgdes[_img_index_old];
    ImageDescriptor_t new_img_desc = id2imgdes[_img_index_now];

    std::vector<cv::Point2f> matched_2d_norm_now, matched_2d_norm_old;
    std::vector<cv::Point3f> matched_3d_now;

    //Tetst BruteMatcher Here
    auto bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);

    auto img_old_small = decode_image(old_img_desc);
    auto img_new_small = decode_image(new_img_desc);
   
    std::vector<cv::Point2f> tracked;// = nowPts;
    std::vector<float> err;
    std::vector<unsigned char> status;
 
    auto nowPts = loop_cam->project_to_image(
        toCV(new_img_desc.landmarks_2d_norm));
    std::vector<cv::Point2f> nowPtsSmall;

    for (auto pt : nowPts) {
        pt.x = pt.x/LOOP_IMAGE_DOWNSAMPLE;
        pt.y = pt.y/LOOP_IMAGE_DOWNSAMPLE;
        nowPtsSmall.push_back(pt);
    }
    
    cv::calcOpticalFlowPyrLK(img_new_small, img_old_small, nowPtsSmall, tracked, status, err, cv::Size(21, 21), 3);
    std::vector<cv::Point2f> good_new;
    
    for(uint i = 0; i < nowPts.size(); i++) {
        if(status[i] == 1) {
            matched_2d_norm_now.push_back(toCV(new_img_desc.landmarks_2d_norm[i]));
            matched_3d_now.push_back(toCV(new_img_desc.landmarks_3d[i]));
            Eigen::Vector2d p_tracker(tracked[i].x, tracked[i].y);
            matched_2d_norm_old.push_back(loop_cam->project_to_norm2d(tracked[i]*LOOP_IMAGE_DOWNSAMPLE));
        }
    }

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
        cv::Mat _show;
        std::cout << "NOWLPS size " << nowKPs.size() << " OLDLPs " << oldKPs.size() << std::endl;
        cv::drawMatches(img_new_small, cvPoints2Keypoints(nowPtsSmall), img_old_small, oldKPs, matches, _show);
        cv::resize(_show, _show, _show.size()*2);
        cv::imshow("Track Loop", _show);
        cv::waitKey(10);
    }
   


    return false;

}

void LoopDetector::on_loop_connection(const LoopConnection & loop_conn) {

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

double HammingDis(uint8_t * a,  uint8_t * b)
{
    cv::Mat d1(1, ORB_FEATURE_SIZE, CV_8UC1, a);
    cv::Mat d2(1, ORB_FEATURE_SIZE, CV_8UC1, b);

    return cv::norm(d1, d2, cv::NORM_HAMMING);
}

bool searchInAera(  uint8_t * window_descriptor,
                    uint8_t * descriptors_old,
                    int & bestIndex) {
    bestIndex = -1;
    double bestDist = 10000;
    for(int i = 0; i < LOOP_FEATURE_NUM ; i++)
    {

        double dis = HammingDis(window_descriptor, descriptors_old + i*ORB_FEATURE_SIZE);
        if(dis < bestDist)
        {
            bestDist = dis;
            bestIndex = i;
        }
    }

    if (bestIndex != -1 && bestDist < 80)
    {
      return true;
    }
    else
      return false;
}

void searchByBRIEFDes(  std::vector<cv::Point2f> &matched_2d_old_norm,
                        std::vector<cv::Point2f> &matched_2d_new_norm,
                        std::vector<cv::Point3f> &matched_3d_new,
                        uint8_t * descriptors_old,
                        uint8_t * descriptors_now,
                        const Point2d_t * keypoints_old_norm,
                        const std::vector<Point2d_t> &keypoints_new_norm,
                        const std::vector<Point3d_t> &keypoints_new_3d) {
    for(int i = 0; i < keypoints_new_norm.size(); i++)
    {
        int best_index = -1;
        int landmark_new_num = keypoints_new_norm.size();
        if (searchInAera(descriptors_now + landmark_new_num * ORB_FEATURE_SIZE, descriptors_old, best_index)) {
            matched_2d_new_norm.push_back(
                    toCV(keypoints_new_norm[i]));
            matched_3d_new.push_back(
                    toCV(keypoints_new_3d[i]));
            ROS_INFO("Good match, old index %d", best_index);
            matched_2d_old_norm.push_back(
                    toCV(keypoints_old_norm[best_index])
            );
        }

    }
}


void searchByBRIEFDesCV(std::vector<cv::Point2f> &matched_2d_old_norm,
                        std::vector<cv::Point2f> &matched_2d_new_norm,
                        std::vector<cv::Point3f> &matched_3d_new,
                        uint8_t * descriptors_old,
                        uint8_t * descriptors_now,
                        const Point2d_t * keypoints_old_norm,
                        const std::vector<Point2d_t> &keypoints_new_norm,
                        const std::vector<Point3d_t> &keypoints_new_3d) {
    // bf = cv.BFMatcher()
    // matches = bf.knnMatch(des1,des2,k=2)
    auto bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    cv::Mat des_new(keypoints_new_norm.size(), ORB_FEATURE_SIZE, CV_8UC1, descriptors_now);
    cv::Mat des_old(LOOP_FEATURE_NUM, ORB_FEATURE_SIZE, CV_8UC1, descriptors_old);

    std::vector<cv::DMatch> matches;
    double max_dist = 0; double min_dist = 100;

    bf->match(des_new, des_old, matches);

    for (auto mat : matches) {
        if (mat.distance > max_dist) {
            max_dist = mat.distance;
        }
        if (mat.distance < min_dist) {
            min_dist = mat.distance;
        }
    }

    std::vector< cv::DMatch > good_matches;
    for(auto m : matches) { 
        if( m.distance < 0.6*max_dist ) {
            matched_2d_new_norm.push_back(
                toCV(keypoints_new_norm[m.queryIdx])
            );
            matched_2d_old_norm.push_back(
                toCV(keypoints_old_norm[m.trainIdx])
            );
            matched_3d_new.push_back(
                toCV(keypoints_new_3d[m.queryIdx])
            );
        }
    }

}