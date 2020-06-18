#include "swarm_localization/localization_DA_init.hpp"
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

#define MIN_DET_THRES 0.5
#define DET_BASELINE_THRES 0.3

double triangulatePoint3DPts(const vector<Pose> & _poses, const vector<Eigen::Vector3d> &points, Eigen::Vector3d &point_3d);
double triangulatePoint3DPts(const vector<pair<Pose, Vector3d>> & dets, Eigen::Vector3d &point_3d);


LocalizationDAInit::LocalizationDAInit(std::vector<SwarmFrame> & _sf_sld_win, double _triangulate_accept_thres):
    sf_sld_win(_sf_sld_win), triangulate_accept_thres(_triangulate_accept_thres) {
    for (auto & sf : sf_sld_win) { 
        available_nodes.insert(sf.node_id_list.begin(), sf.node_id_list.end());
    }

    
}

bool LocalizationDAInit::try_data_association(std::map<int, int> &mapper) {
    //First we try to summarized all the UNIDENTIFIED detections
    std::set<int> unidentified;
    
    DroneTraj traj;
    if(sf_sld_win.size() > 0) {
        self_id = sf_sld_win[0].self_id;
        for (auto & sf : sf_sld_win) {
            if (sf.has_node(self_id)) {
                traj.push_back(make_pair(sf.ts, sf.id2nodeframe[self_id].pose()));
            }
        }    
    }

    for (auto & sf : sf_sld_win) {
        // ROS_INFO("Scan sf %d", sf.ts);


        for (auto it: sf.id2nodeframe) {
            auto & _nf = it.second;
            // ROS_INFO("Scanning nf %d det %d", _nf.id, _nf.detected_nodes.size());

            for (auto it: _nf.detected_nodes) {
                // ROS_INFO("nf %d detect %d", _nf.id, it.first);
                if (it.first >= UNIDENTIFIED_MIN_ID) {
                    unidentified.insert(it.first);
                }
            }
        }
    }

    ROS_INFO("The sliding window contain %d unidentified drones", unidentified.size());

    if (unidentified.size() == 0) {
        return false;
    }

    //Secondly, we start give guess

    std::map<int, int> guess;
    std::map<int, DroneTraj> est_pathes;
    est_pathes[self_id] = traj;
    if (DFS(est_pathes, guess, unidentified)) {
        ROS_INFO("Initial guess is OK");
        return true;
    }

    return false;
}

bool LocalizationDAInit::verify(const std::map<int, DroneTraj> & est_pathes, const std::map<int, int> & guess) {
    //First we assume all static
    //Ignore verify now
    return true;
}

void boundingbox(Eigen::Vector3d v, Eigen::Vector3d & min, Eigen::Vector3d & max);

int LocalizationDAInit::estimate_pathes(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess) {
    printf("Estimate pathes with guess");
    for (auto it : guess) {
        printf("%d:%d ", it.first, it.second);
    }
    printf("\n");

    int count = 0;
    for (auto _id : available_nodes) {
        // Recalculate every time
        if (_id == self_id) {
            continue;
        }

        DroneTraj _path;
        int success = estimate_path(_path, _id, guess, est_pathes);
        if (success < 0) {
            return -1;
        } else {
            count += success;

            if(success) {
                est_pathes[_id] = _path;
            }
        }
    }
    return count;
}

bool LocalizationDAInit::DFS(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess, std::set<int> & unidentified) {

    ROS_INFO("DFS Unidentified num %ld guess ", unidentified.size());
    
    for (auto it : guess) {
        printf("%d:%d\n", it.first, it.second);
    }

    printf("\n");

    if (unidentified.size() == 0) {
        if (est_pathes.size() < available_nodes.size()) {
            //Now we should to estimate all known pathes
            printf("guess failed return\n");
            return false;
        } else if (verify(est_pathes, guess)) {
            for (auto it:est_pathes) {
                auto pos = it.second[0].second.pos();
                printf("id %d pos %f %f %f", it.first, pos.x(), pos.y(), pos.z());
            }
            
            printf("guess verified, this is final result, return\n");
            return true;
        }
    }

    //Search _uniden
    int _uniden = *unidentified.begin();
    ROS_INFO("Search unidentified %d", _uniden);
    if (guess.find(_uniden) == guess.end()) {
        for (auto new_id : available_nodes) {
            // ROS_INFO();
            //This new id must not be the detector drone itself
            if (uniden_detector[_uniden] == new_id) {
                //Than the unidentified is detected by this new id
                continue;
            }

            ROS_INFO("Try to use %d as %d\n", _uniden, new_id);

            //Here we start search this guess
            std::map<int, int> this_guess(guess);
            this_guess[_uniden] = new_id;
            std::set<int> this_unidentified(unidentified);
            this_unidentified.erase(_uniden);

            std::map<int, DroneTraj> this_pathes(est_pathes);
            
            //We will try to estimate this position and verify it.
            //Here we should estimate all unknow nodes
            int success = estimate_pathes(this_pathes, this_guess);
            if (success < 0) {
                return false;
            }

            bool _succ = verify(this_pathes, this_guess);
            if (!_succ) {
                continue;
            }
            bool result = DFS(this_pathes, this_guess, this_unidentified);

            if (result) {
                //Here we assume only one result
                guess.insert(this_guess.begin(), this_guess.end());
                unidentified.insert(this_unidentified.begin(), this_unidentified.end());

                return true;
            }
        }
    }

    //No good result, return false
    return false;
}

//return 0: not observable
//return 1: good
//return -1: estimate failed
int LocalizationDAInit::estimate_path(DroneTraj & traj, int idj, map<int, int> & guess, const map<int, DroneTraj> est_pathes) {
    //Assume static now
    //Summarize Known constrains
    //Constrain may have 2 type: distance relative to position and unit vector relative to position
    vector<pair<Vector3d, double>> distance_constrain;
    vector<pair<Pose, Vector3d>> detection_constrain;

    Eigen::Vector3d max_bbx_dis(-100000,-100000,-100000);
    Eigen::Vector3d min_bbx_dis(1000000,100000,100000);

    Eigen::Vector3d max_bbx_det(-100000,-100000,-100000);
    Eigen::Vector3d min_bbx_det(1000000,100000,100000);

    for (auto & _sf : sf_sld_win) {
        for (auto it: _sf.id2nodeframe) {
            if (est_pathes.find(it.first) != est_pathes.end()) {
                // ROS_INFO("Known object %d", it.first);

                //Then the traj of this node is known, can use to estimate others
                auto & nf = it.second;
                int _id = it.first;
                //Assmue static now
                auto pose = est_pathes.at(_id)[0].second;

                if (nf.dis_map.find(idj) != nf.dis_map.end()) {
                    //Node idj can be observer distance to id
                    distance_constrain.push_back(make_pair(pose.pos(), nf.dis_map[idj]));
                    boundingbox(nf.position(), min_bbx_dis, max_bbx_dis);
                }
                
                for (auto it : nf.detected_nodes) {
                    auto unidentify_id = it.first;
                    if (guess.find(unidentify_id)!= guess.end() && guess[unidentify_id] == idj) {
                        //If this detection can be map to idj
                        //detection_constrain.push_back();
                        auto dir = it.second.p.normalized();
                        auto d = 1 / it.second.inv_dep;
                        auto det_mea = dir * d;
                        detection_constrain.push_back(make_pair(pose, det_mea));
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

    double det_baseline = (max_bbx_det - min_bbx_det).norm();

    //We ignore distances first; use only triangulate to init

    if(detection_constrain.size() == 0) {
        return 0;
    }

    //Else processing triangulate or estimate with single detection

    if (detection_constrain.size() == 1 || (max_bbx_det - min_bbx_det).norm() < DET_BASELINE_THRES) {
        Pose pose = detection_constrain[0].first;
        Vector3d dir = detection_constrain[0].second;
        Vector3d estimated = pose * dir;

        ROS_INFO("Pose");
        pose.print();

        printf("dir %f %f %f\n", dir.x(), dir.y(), dir.z());
        
        for (auto & _sf : sf_sld_win) {
            if (_sf.id2nodeframe.find(idj) != _sf.id2nodeframe.end()) {
                Pose p(estimated, _sf.id2nodeframe[idj].yaw());
                traj.push_back(make_pair(_sf.ts, p));
            }
        }

        ROS_INFO("Estimate %d pos %f %f %f with one detection ", idj, estimated.x(), estimated.y(), estimated.z());
        return 1;
    }

    if (detection_constrain.size() > 1) {
        //Now we can detect it with triangulate
        Vector3d position;
        double error = triangulatePoint3DPts(detection_constrain, position);
        //Set trajectory here
        for (auto & _sf : sf_sld_win) {
            if (_sf.id2nodeframe.find(idj) != _sf.id2nodeframe.end()) {
                auto att =  _sf.id2nodeframe[idj].pose().att();
                traj.push_back(make_pair(_sf.ts, Pose(position, att)));
            }
        }

        ROS_INFO("Estimate %d pos %f %f %f with multiple detection error %f", idj, position.x(), position.y(), position.z(), error);
        
        if (error > triangulate_accept_thres) {
            return -1;
        } else {
            return 1;
        }
    }

    return 1;
}


double triangulatePoint3DPts(const vector<pair<Pose, Vector3d>> & dets, Eigen::Vector3d &point_3d) {
    vector<Pose> _poses; 
    vector<Eigen::Vector3d> pts;
    for (auto it: dets) {
        _poses.push_back(it.first);
        pts.push_back(it.second.normalized());
    }

    return triangulatePoint3DPts(_poses, pts, point_3d);
}

double triangulatePoint3DPts(const vector<Pose> & _poses, const vector<Eigen::Vector3d> &points, Eigen::Vector3d &point_3d)
{
    vector<Eigen::Matrix<double, 3, 4>> poses;
    for (auto p : _poses) {
        poses.push_back(p.to_isometry().affine());
    }

    Eigen::MatrixXd design_matrix(poses.size()*2, 4);
    assert(poses.size() > 0 && poses.size() == points.size() && "We at least have 2 poses and number of pts and poses must equal");
    for (unsigned int i = 0; i < poses.size(); i ++) {
        double p0x = points[i][0];
        double p0y = points[i][1];
        double p0z = points[i][2];
        design_matrix.row(i*2) = p0x * poses[i].row(2) - p0z*poses[i].row(0);
        design_matrix.row(i*2+1) = p0y * poses[i].row(2) - p0z*poses[i].row(1);

    }
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);

    Eigen::MatrixXd pts(4, 1);
    pts << point_3d.x(), point_3d.y(), point_3d.z(), 1;
    Eigen::MatrixXd errs = design_matrix*pts;
    // std::cout << "ERR" << errs.sum() << std::endl;
    return errs.norm()/ errs.rows(); 
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