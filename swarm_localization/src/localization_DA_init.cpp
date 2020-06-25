#include "swarm_localization/localization_DA_init.hpp"
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

#define MIN_DET_THRES 0.5
#define DET_BASELINE_THRES 0.3

//For visual initial, we limit all in 10 meter is OK
#define POSITION_LIM 30

// #define DFS_BEBUG_OUTPUT

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
            if(detected_set.find(_nf.id) == detected_set.end()) {
                detected_set[_nf.id] = std::set<int>();
            }
            // ROS_INFO("Scanning nf %d det %d", _nf.id, _nf.detected_nodes.size());

            for (auto it: _nf.detected_nodes) {
                // ROS_INFO("nf %d detect %d", _nf.id, it.first);
                if (it.first >= UNIDENTIFIED_MIN_ID) {
                    int unidentified_node = it.first;
                    unidentified.insert(unidentified_node);
                    if (detected_set[_nf.id].find(unidentified_node) == detected_set[_nf.id].end()) {
                        detected_set[_nf.id].insert(unidentified_node);
                        printf("Detector %d, unidentified %d\n", _nf.id, unidentified_node);
                        uniden_detector[unidentified_node] = _nf.id;
                    }
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
    auto ret = DFS(est_pathes, guess, unidentified);
    if (ret.first) {
        ROS_INFO("Initial guess is OK cost %f the assoication", ret.second);
        for (auto it : guess) {
            printf("%d:%d ", it.first, it.second);
        }
        printf("\n");
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

double LocalizationDAInit::estimate_pathes(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess) {

#ifdef DFS_BEBUG_OUTPUT
    printf("Estimate pathes with guess");
    for (auto it : guess) {
        printf("%d:%d ", it.first, it.second);
    }
    printf("\n");
#endif

    double count = 0;
    for (auto _id : available_nodes) {
        // Recalculate every time
        if (_id == self_id) {
            continue;
        }

        DroneTraj _path;
        auto ret = estimate_path(_path, _id, guess, est_pathes);
        if (ret.first < 0) {
            return -1;
        } else {
            count += ret.second;
            if(ret.first == 1) {
                est_pathes[_id] = _path;
            }
        }
    }
    return count;
}

bool LocalizationDAInit::check_guess_has_assign_id(std::map<int, int> & guess, int detector, int _new_id) {
    // printf("The detector is %d\n, the new assigned id is %d", detector, _new_id);
    std::set<int> detecteds = detected_set[detector];
    for (int _un: detecteds) {
        // printf("The unidentified %d is detected by the detector\n", _un);
        if (guess.find(_un)!=guess.end() && guess[_un] == _new_id) {
            return true;
        }
    }
    return false;
}

std::pair<bool, double> LocalizationDAInit::DFS(std::map<int, DroneTraj> & est_pathes, std::map<int, int> & guess, const std::set<int> & unidentified) {
#ifdef DFS_BEBUG_OUTPUT
    ROS_INFO("DFS Unidentified num %ld guess ", unidentified.size());
    
    for (auto it : guess) {
        printf("%d:%d\n", it.first, it.second);
    }

    printf("\n");
#endif

    if (unidentified.size() == 0) {
        if (est_pathes.size() < available_nodes.size()) {
            //Now we should to estimate all known pathes
#ifdef DFS_BEBUG_OUTPUT
            printf("guess failed return\n");
#endif
            return make_pair(false, -1);
        } else if (verify(est_pathes, guess)) {
            double cost = estimate_pathes(est_pathes, guess);

#ifdef DFS_BEBUG_OUTPUT
            for (auto it:est_pathes) {
                auto pos = it.second[0].second.pos();
                printf("id %d pos %f %f %f", it.first, pos.x(), pos.y(), pos.z());
            }

            printf("\n");
#endif
            if (cost < 0 ){
#ifdef DFS_BEBUG_OUTPUT
                printf("guess failed return\n");
#endif
                return make_pair(false, -1);
            } else {
#ifdef DFS_BEBUG_OUTPUT
                ROS_WARN("Guess verified, this is final result cost %f, return\n", cost);
#endif
                return make_pair(true, cost);
            }
            
        }
    }

    //Search _uniden
    int _uniden = *unidentified.begin();
#ifdef DFS_BEBUG_OUTPUT
    ROS_INFO("Search unidentified %d", _uniden);
#endif

    std::map<int, DroneTraj> best_pathes;
    std::map<int, int> best_guess;
    double best_cost = 1000000;

    if (guess.find(_uniden) == guess.end()) {
        for (auto new_id : available_nodes) {
            // ROS_INFO();
            //This new id must not be the detector drone itself
            if (uniden_detector[_uniden] == new_id) {
                //Than the unidentified is detected by this new id
                continue;
            }

            //We need to check if other drones detect by the detector has been guess with this new id
            int detector = uniden_detector[_uniden];
            if (check_guess_has_assign_id(guess, detector, new_id)) {
                // printf("New id %d has been assigned on same detector, jump\n", new_id);
                continue;
            }
#ifdef DFS_BEBUG_OUTPUT
            ROS_INFO("Try to use %d as %d\n", _uniden, new_id);
#endif

            //Here we start search this guess
            std::map<int, int> this_guess(guess);
            this_guess[_uniden] = new_id;
            std::set<int> this_unidentified(unidentified);
            this_unidentified.erase(_uniden);

            std::map<int, DroneTraj> this_pathes(est_pathes);
            
            //We will try to estimate this position and verify it.
            //Here we should estimate all unknow nodes
            double success = estimate_pathes(this_pathes, this_guess);
            if (success < 0) {
#ifdef DFS_BEBUG_OUTPUT
                ROS_WARN("Estimate path failed; The guess result wrong result.");
#endif
                continue;
            }

            bool _succ = verify(this_pathes, this_guess);
            if (!_succ) {
                continue;
            }

            auto result = DFS(this_pathes, this_guess, this_unidentified);

            if (result.first && result.second < best_cost) {
                //Here we assume only one result
                best_cost = result.second;
                best_guess = this_guess;
                best_pathes = this_pathes;
            }
        }
    }

    if(best_cost < 100) {
        guess = best_guess;
        est_pathes = best_pathes;
        return make_pair(true, best_cost);
    }

    //No good result, return false
    return make_pair(false, -1);
}

//return 0: not observable
//return 1: good
//return -1: estimate failed
std::pair<int, double>  LocalizationDAInit::estimate_path(DroneTraj & traj, int idj, map<int, int> & guess, const map<int, DroneTraj> est_pathes) {
    //Assume static now
    //Summarize Known constrains
    //Constrain may have 2 type: distance relative to position and unit vector relative to position
    vector<pair<Vector3d, double>> distance_constrain;
    vector<pair<Pose, Vector3d>> detection_constrain;
    vector<double> detection_correspond_distance;

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

                double distance = -1;
                if (nf.dis_map.find(idj) != nf.dis_map.end()) {
                    //Node idj can be observer distance to id
                    distance_constrain.push_back(make_pair(pose.pos(), nf.dis_map[idj]));
                    distance = nf.dis_map[idj];
                    boundingbox(pose.pos(), min_bbx_dis, max_bbx_dis);
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

                        detection_correspond_distance.push_back(distance);
                        boundingbox(pose.pos(), min_bbx_det, max_bbx_det);
                    }
                }
            }
        }
    }

#ifdef DFS_BEBUG_OUTPUT
    ROS_INFO("Node %d has %ld distance M baseline %f and %ld det M baseline %f", 
        idj, distance_constrain.size(), (max_bbx_dis - min_bbx_dis).norm(),
        detection_constrain.size(), (max_bbx_det - min_bbx_det).norm()
    );
#endif

    double det_baseline = (max_bbx_det - min_bbx_det).norm();

    //We ignore distances first; use only triangulate to init

    if(detection_constrain.size() == 0) {
        return std::make_pair(0, 0);
    }

    //Else processing triangulate or estimate with single detection

    if (detection_constrain.size() == 1 || (max_bbx_det - min_bbx_det).norm() < DET_BASELINE_THRES) {

        //Need to deal with multiple target from one drone is identify as same
        Pose pose = detection_constrain[0].first;

        Vector3d dir = detection_constrain[0].second;

        if (detection_correspond_distance[0] > 0) {
            dir = dir.normalized() * detection_correspond_distance[0];
        }

        Vector3d estimated = pose * dir;

#ifdef DFS_BEBUG_OUTPUT
        ROS_INFO("Pose");
        pose.print();

        printf("dir %f %f %f\n", dir.x(), dir.y(), dir.z());
#endif

        for (auto & _sf : sf_sld_win) {
            if (_sf.id2nodeframe.find(idj) != _sf.id2nodeframe.end()) {
                Pose p(estimated, _sf.id2nodeframe[idj].yaw());
                traj.push_back(make_pair(_sf.ts, p));
            }
        }

#ifdef DFS_BEBUG_OUTPUT
        ROS_INFO("Estimate %d pos %f %f %f with one detection ", idj, estimated.x(), estimated.y(), estimated.z());
#endif
        return std::make_pair(1, 1e-3);
    }

    if (detection_constrain.size() > 1) {
#ifdef DFS_BEBUG_OUTPUT
        ROS_INFO("Will apply triangulation, baseline %f", (max_bbx_det - min_bbx_det).norm());
#endif
        //Now we can detect it with triangulate
        Vector3d position;
        double error = triangulatePoint3DPts(detection_constrain, position);
        if (position.norm() > POSITION_LIM || isnan(error) || 
            isnan(position.x()) || isnan(position.y()) || isnan(position.z())) {
#ifdef DFS_BEBUG_OUTPUT
            ROS_WARN("Large initial position or nan detected %f %f %f, cost %f, give up", 
                position.x(), position.y(), position.z(), error);
#endif
            return make_pair(-1, 0);
        }

        //Set trajectory here
        for (auto & _sf : sf_sld_win) {
            if (_sf.id2nodeframe.find(idj) != _sf.id2nodeframe.end()) {
                auto att =  _sf.id2nodeframe[idj].pose().att();
                traj.push_back(make_pair(_sf.ts, Pose(position, att)));
            }
        }

#ifdef DFS_BEBUG_OUTPUT
        ROS_INFO("Estimate %d pos %f %f %f with multiple detection error %f", idj, position.x(), position.y(), position.z(), error);
#endif
        
        if (error > triangulate_accept_thres) {
            return std::make_pair(-1, 0);
        } else {
            return std::make_pair(1, error);
        }
    }

    return make_pair(-1, 0);
}


double triangulatePoint3DPts(const vector<pair<Pose, Vector3d>> & dets, Eigen::Vector3d &point_3d) {
    vector<Pose> _poses; 
    vector<Eigen::Vector3d> pts;
    for (auto it: dets) {
        _poses.push_back(it.first);
        pts.push_back(it.second.normalized());

#ifdef DFS_BEBUG_OUTPUT
        printf("Pose ");
        it.first.print();
        printf("Pts %f %f %f\n\n", 
            it.second.normalized().x(),
            it.second.normalized().y(),
            it.second.normalized().z());
#endif
    }

    return triangulatePoint3DPts(_poses, pts, point_3d);
}

double triangulatePoint3DPts(const vector<Pose> & _poses, const vector<Eigen::Vector3d> &_points, Eigen::Vector3d &point_3d)
{
    vector<Eigen::Matrix<double, 3, 4>> poses;
    vector<Vector3d> positions;
    vector<Eigen::Vector3d> points;
    
    for (unsigned int i = 0; i < _poses.size(); i++) {
        auto p = _poses[i];
        auto pos = p.pos();
        bool is_near_to_previous = false;
        for (auto p2 : positions) {
            //Need to speed up here
            if ((p2 - pos).norm() < DET_BASELINE_THRES / 2.0) {
                is_near_to_previous = true;
#ifdef DFS_BEBUG_OUTPUT
                ROS_INFO("Pos %f %f %f is near to %f %f %f", pos.x(), pos.y(), pos.z(), p2.x(), p2.y(), p2.z());
#endif
                break;
            }
        }

        if (!is_near_to_previous) {
            poses.push_back(p.to_isometry().affine());
            positions.push_back(pos);
            points.push_back(_points[i]);
        }
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

    point_3d = - point_3d;
    // Eigen::MatrixXd errs = design_matrix*pts;
    // std::cout << "ERR" << errs.norm()/ errs.rows() << std::endl;
    // return errs.norm()/ errs.rows(); 

    double error = 0;
    for (int i = 0; i < _poses.size(); i++) {
        //First we get the direction of the detected drones
        auto dir1 = (_poses[i].att() * _points[i]).normalized();

        //Second we get the direction of the point relative to this drone
        auto dir2 = (point_3d - _poses[i].pos()).normalized();

        //Then we try to get the angle
        double tmp = dir1.dot(dir2) / dir1.norm()/dir2.norm();
        double angle;
        if (tmp > 0.999) {
            angle = 0;
        } else if (tmp < - 0.999) {
            angle = M_PI;
        } else {
            angle = acos(tmp);
        }

        // std::cout << "Dir1" << dir1.transpose() << " Dir2" << dir2.transpose();
        // printf("acos %f angle %f\n", dir1.dot(dir2) / dir1.norm()/dir2.norm(), angle);
        error = error + angle;
    }

    // if(error/(double)(_poses.size()) > 0.5f ) {
        // printf("Error %f %f", error, error/(double)(_poses.size() > 0.5));
        // exit(-1);
    // }

    return error/(double)(_poses.size());
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