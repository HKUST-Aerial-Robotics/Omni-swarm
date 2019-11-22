#include <loop_detector.h>
#include <swarm_msgs/swarm_lcm_converter.hpp>
#include <opencv2/opencv.hpp>
#include <chrono> 
using namespace std::chrono; 


void LoopDetector::on_image_recv(const ImageDescriptor_t & img_des, cv::Mat img) {
    auto start = high_resolution_clock::now(); 

    cv::Mat feature = cvfeatureFromByte((uint8_t*)img_des.feature_descriptor);
    // std::cout << "FEATUREX"<< featurex.size() << std::endl;

    ROS_INFO("Querying image descriptor ");

    int _id = db.add(feature);
    std::cout << "Add Cost " << duration_cast<microseconds>(high_resolution_clock::now() - start).count()/1000.0 <<"ms" << std::endl;

    if (db.size() > MATCH_INDEX_DIST) {

        DBoW3::QueryResults ret;
        db.query(feature, ret, 1, _id - MATCH_INDEX_DIST);
        auto stop = high_resolution_clock::now(); 

        if (ret.size() > 0 && ret[0].Score > LOOP_BOW_THRES) {
            std::cout << "Time Cost " << duration_cast<microseconds>(stop - start).count()/1000.0 <<"ms RES: " << ret << std::endl;
            if (ret.size() > 0 && !img.empty()) {
                int _old_id = ret[0].Id;
                if (id2imgs.find(_old_id)!= id2imgs.end()) {
                    cv::Mat _show;
                    cv::hconcat(img, id2imgs[_old_id], _show);
                    cv::imshow("Loop", _show);
                    cv::waitKey(3);
                }
            }
        }        

    }

    if (!img.empty() ) {
        id2imgs[_id] = img;
    }

    ROS_INFO("Adding image descriptor %d to database", _id);

}

void LoopDetector::on_loop_connection(const LoopConnection & loop_conn) {

}

LoopDetector::LoopDetector(const std::string & voc_path):
    voc(voc_path), db(voc, false, 0) {

}
