#include "swarm_localization/localization_DA_init.hpp"
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

bool LocalizationDAInit::try_data_association(std::map<int, int> &mapper) {
    //First we try to summarized all the UNIDENTIFIED detections
    std::set<int> unidentified;
    for (auto sf : sf_sld_win) {
        for (auto it: sf.id2nodeframe) {
            auto & _nf = it.second;
            for (auto it: _nf.detected_nodes) {
                if (it.first >= UNIDENTIFIED_MIN_ID) {
                    unidentified.insert(it.first);
                }
            }
        }
    }

    ROS_INFO("The sliding window contain %d unidentified drones", unidentified.size());

    //Secondly, we start give guess

    std::map<int, int> guess;
    std::map<int, DroneTraj> est_pathes;
    if (DFS(est_pathes, guess, unidentified)) {
        ROS_INFO("Initial guess is OK");
    }
}

bool LocalizationDAInit::verify(std::map<int, int> & guess) {
    //First we assume all static

}

void boundingbox(Eigen::Vector3d v, Eigen::Vector3d & min, Eigen::Vector3d & max);


bool LocalizationDAInit::DFS(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess, std::set<int> & unidentified) {
    if (unidentified.size() == 0) {
        return true;
    }

    for (auto _uniden : unidentified) {
        //Search _uniden
        if (guess.find(_uniden) == guess.end()) {
            for (auto new_id : available_nodes) {
                //This new id must not be the detector drone itself
                if (uniden_detector[_uniden] == new_id) {
                    //Than the unidentified is detected by this new id
                    continue;
                }

                //Here we start search this guess
                std::map<int, int> this_guess(guess);
                this_guess[_uniden] = new_id;
                std::set<int> this_unidentified(unidentified);
                this_unidentified.erase(_uniden);
                
                //We will try to estimate this position and verify it.
                DroneTraj this_path;
                estimate_path(this_path, new_id, this_guess, est_pathes);
                
                bool result = DFS(est_pathes, this_guess, this_unidentified);

                if (result) {
                    //Here we assume only one result
                    guess.insert(this_guess.begin(), this_guess.end());
                    unidentified.insert(this_unidentified.begin(), this_unidentified.end());

                    return true;
                }
            }
        }
    }

    //No good result, return false
    return false;
}

int LocalizationDAInit::estimate_path(DroneTraj & traj, int idj, map<int, int> & guess, const map<int, DroneTraj> est_pathes) {
    //Assume static now
    //Summarize Known constrains
    //Constrain may have 2 type: distance relative to position and unit vector relative to position
    vector<pair<Vector3d, double>> distance_constrain;
    vector<pair<Vector3d, Vector3d>> detection_constrain;

    Eigen::Vector3d max_bbx_dis(-100000,-100000,-100000);
    Eigen::Vector3d min_bbx_dis(1000000,100000,100000);

    Eigen::Vector3d max_bbx_det(-100000,-100000,-100000);
    Eigen::Vector3d min_bbx_det(1000000,100000,100000);

    for (auto & _sf : sf_sld_win) {
        for (auto it: _sf.id2nodeframe) {
            if (est_pathes.find(it.first) != est_pathes.end()) {
                //Then the traj of this node is known, can use to estimate others
                auto & nf = it.second;
                auto pose = nf.pose();

                if (nf.dis_map.find(idj) != nf.dis_map.end()) {
                    //Node idj can be observer distance to id
                    distance_constrain.push_back(make_pair(nf.position(), nf.dis_map[idj]));
                    boundingbox(nf.position(), min_bbx_dis, max_bbx_dis);
                }
                
                for (auto it : nf.detected_nodes) {
                    auto unidentify_id = it.first;
                    if (guess.find(unidentify_id)!= guess.end() && guess[unidentify_id] == idj) { //If this detection can be map to idj
                        // detection_constrain.push_back();
                        auto dir = pose.att() * it.second.p;
                        auto d = 1 / it.second.inv_dep;
                        auto det_mea = dir * d;
                        detection_constrain.push_back(make_pair(nf.position(), det_mea));
                        boundingbox(nf.position(), min_bbx_det, max_bbx_det);
                    }
                }
            }
        }
    }

    ROS_INFO("Node %d has %ld distance M baseline %f and %ld det M baseline %f", 
        idj, distance_constrain.size(), (max_bbx_dis - min_bbx_dis).norm(),
        detection_constrain.size(), (max_bbx_det - min_bbx_det).norm()
    );

    //We ignore distances first

    
}



void boundingbox(Eigen::Vector3d v, Eigen::Vector3d & min, Eigen::Vector3d & max) {
    if (v.x() > max.x()) {
        max.x() = v.x();
    }
    
    if (v.y() > max.y()) {
        max.y() = v.y();
    }
    
    if (v.z() > max.z()) {
        max.z() = v.z();
    }

    if (v.x() < min.x()) {
        min.x() = v.x();
    }
    
    if (v.y() < min.y()) {
        min.y() = v.y();
    }
    
    if (v.z() < min.z()) {
        min.z() = v.z();
    }
}