#include <loop_detector.h>
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <opencv2/opencv.hpp>
#include <chrono> 
using namespace std::chrono; 


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
