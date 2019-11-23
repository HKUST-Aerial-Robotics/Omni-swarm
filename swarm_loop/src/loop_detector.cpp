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

void LoopDetector::on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img) {
    auto start = high_resolution_clock::now(); 

    cv::Mat feature = cvfeatureFromByte((uint8_t*)img_des.feature_descriptor);
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


bool LoopDetector::compute_loop(const unsigned int & _img_index_now, const unsigned int & _img_index_old, LoopConnection & ret) {
    ImageDescriptor_t old_img_desc = id2imgdes[_img_index_old];
    ImageDescriptor_t new_img_desc = id2imgdes[_img_index_now];

    std::vector<cv::Point2f> matched_2d_norm_now, matched_2d_norm_old;
    std::vector<cv::Point3f> matched_3d_norm_now;

    searchByBRIEFDes(matched_2d_norm_old, matched_2d_norm_now, matched_3d_norm_now,
        old_img_desc.landmarks_descriptors.data(), new_img_desc.landmarks_descriptors.data(),
        old_img_desc.all_features_2d_norm, new_img_desc.landmarks_2d_norm, new_img_desc.landmarks_3d);

    
    if (id2imgs.find(_img_index_now) != id2imgs.end() &&
        id2imgs.find(_img_index_old) != id2imgs.end()) {
        cv::Mat _show;
        auto img_new = id2imgs[_img_index_now];
        auto img_old = id2imgs[_img_index_old];
        cv::hconcat(img_new, img_old, _show);
        cv::imshow("Loop", _show);
        cv::waitKey(10);
    }

}

void LoopDetector::on_loop_connection(const LoopConnection & loop_conn) {

}

LoopDetector::LoopDetector(const std::string & voc_path):
    voc(voc_path), db(voc, false, 0) {

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
    int bestDist = 128;
    for(int i = 0; i < LOOP_FEATURE_NUM ; i++)
    {

        int dis = HammingDis(window_descriptor, descriptors_old + i*ORB_FEATURE_SIZE);
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
            matched_2d_old_norm.push_back(
                    toCV(keypoints_old_norm[best_index])
            );
        }

    }

}