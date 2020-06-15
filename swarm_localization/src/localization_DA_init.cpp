#include "swarm_localization/localization_DA_init.hpp"
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

#define MIN_DET_THRES 0.5

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

int LocalizationDAInit::estimate_path(DroneTraj & traj, int idj, map<int, int> & guess, 
    const map<int, DroneTraj> est_pathes) {
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
                    if (guess.find(unidentify_id)!= guess.end() && guess[unidentify_id] == idj) {
                        //If this detection can be map to idj
                        //detection_constrain.push_back();
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

    //We ignore distances first; use only triangulate to init

    if((max_bbx_det - min_bbx_det).norm() < MIN_DET_THRES || detection_constrain.size() == 0) {
        return 0;
    }

    //Else processing triangulate or estimate with single detection

    if (detection_constrain.size() == 1) {
        Vector3d estimated = detection_constrain[0].first + detection_constrain[0].second;
        for (auto & _sf : sf_sld_win) {
            if (_sf.id2nodeframe.find(idj) != _sf.id2nodeframe.end()) {
                Pose p(estimated, _sf.id2nodeframe[idj].yaw());
                traj.push_back(make_pair(_sf.ts, p));
            }
        }
        return 1;
    }


    return 1;
}

double triangulatePoint3DPts(vector<Eigen::Matrix<double, 3, 4>> &poses, vector<Eigen::Vector3d> &points, Eigen::Vector3d &point_3d)
{
    //TODO:Rewrite this for 3d point
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