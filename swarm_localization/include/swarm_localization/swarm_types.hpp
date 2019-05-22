#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>
#include <swarm_detection/swarm_detect_types.h>
#include <ros/ros.h>


typedef std::map<int, double> DisMap;

using namespace Eigen;
namespace swarm {

    class Node {
    protected:
        int id = -1;
        std::vector<Camera *> camera;
        std::map<int, DroneMarker> markers;

        bool has_vo = false;
        bool has_uwb = false;
        bool has_global_pose = false;
        bool has_armarkers = false;
        bool has_global_velocity = false;

        Pose global_pose;
        Eigen::Vector3d global_velocity = Vector3d(0, 0, 0);
        Vector3d anntena_pos = Vector3d(0, 0, 0);



        void load_cameras(const std::string &path) {
            //TODO:
            //Load camera from path
        }

        void load_markers(const std::string &path) {
            //TODO:
            //Load markers from file path
        }

    public:

        bool HasCamera() const {
            return !camera.empty();
        }

        bool HasVO() const {
            return has_vo;
        }

        bool HasUWB() const {
            return has_uwb;
        }

        bool HasGlobalPose() const {
            return has_global_pose;
        }


        Node(int _id) :
                id(_id) {

        }

        static Node *createDroneNode(int _id,
                                     const std::string &node_config_path = "",
                                     const std::string &camera_config_path = "",
                                     const std::string &marker_config_path = ""
        ) {
            Node *node = new Node(_id);
            node->load_cameras(camera_config_path);
            node->has_vo = true;
            node->has_uwb = true;
            node->has_global_pose = false;

            //TODO:
            //Temp code
            node->anntena_pos = Vector3d(0, -0.083, 0.078);
            return node;
        }

        Pose get_global_pose() const {
            return global_pose;
        }

        Vector3d get_global_velocity() const {
            return global_velocity;
        }

        Vector3d get_anntena_pos() const {
            return anntena_pos;
        }
    };

    class NodeFrame {
    public:
        bool frame_available = false;
        bool vo_available = false;
        bool dists_available = false;
        bool corners_available = false;
        bool has_detect_relpose = false;
        Node *node = nullptr;
        int id = -1;

        DisMap dis_map;
        Pose self_pose;
        Eigen::Vector3d self_vel = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d global_vel = Eigen::Vector3d(0, 0, 0);
        std::vector<corner_array> corner_by_cams;
        std::map<int, Pose> detected_nodes;
        std::map<int, Eigen::Vector3d> detected_nodes_poscov;
        std::map<int, Eigen::Vector3d> detected_nodes_angcov;

        ros::Time stamp;
        int ts;

        NodeFrame(Node *_node) :
                node(_node) {

        }

        NodeFrame() {

        }

        Pose pose() const {
            //If has vo, return vo position
            assert(!(node->HasVO() && !vo_available) && "Try get position on VO failed node");

            if (vo_available) {
                return self_pose;
            } else {
                if (node->HasGlobalPose()) {
                    return node->get_global_pose();
                } else {
                    //Is unknown static node, using 0, 0, 0 position
                    Pose _pose;
                    return _pose;
                }
            }
        }

        Eigen::Vector3d position() const {
            return pose().position;
        }

        Eigen::Quaterniond attitude() const {
            return pose().attitude;
        }

        Eigen::Vector3d velocity() const {
            assert(!(node->HasVO() && !vo_available) && "Try get velocity on VO failed node");
            if (vo_available) {
                return self_vel;
            } else {
                if (node->HasGlobalPose()) {
                    return node->get_global_velocity();
                } else {
                    //Is unknown static node, using 0, 0, 0 position

                    return Vector3d(0, 0, 0);
                }
            }
        }
    };

    class SwarmFrame {
    public:
        std::map<int, NodeFrame> id2nodeframe;
        std::map<int, DisMap> dis_mat;
        std::vector<int> node_id_list;

        ros::Time stamp;
        int ts;

        int swarm_size() {
            return id2nodeframe.size();
        }

        bool HasUWB(const int id) const {
            return id2nodeframe.at(id).node->HasUWB();
        }

        bool HasDis(const int idj, const int idi) const {
            if (dis_mat.find(idj) == dis_mat.end()) {
                return false;
            }
            return dis_mat.at(idj).find(idi) != dis_mat.at(idj).end();
        }

        bool HasID(const int _id) const{
            return (id2nodeframe.find(_id) == id2nodeframe.end());
        }

        bool HasDetect(const int _id) const {
            //TODO:
        }
        double distance(const int idj, const int idi) const {
            assert(HasDis(idj, idi) && "Require distance not have");

            return dis_mat.at(idj).at(idi);
        }

        Vector3d position(const int id) const {
            return id2nodeframe.at(id).position();
        }

        Vector3d velocity(const int id) const {
            return id2nodeframe.at(id).velocity();
        }

        Quaterniond attitude(const int id) const {
            return id2nodeframe.at(id).attitude();
        }

    };
};