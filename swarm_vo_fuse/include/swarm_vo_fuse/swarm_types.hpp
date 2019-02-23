#include <iostream>
#include <eigen3/Eigen/Dense>
#include <map>
#include <vector>
#include <swarm_detection/swarm_detect_types.h>

typedef std::map<int, double> DisMap;

namespace Swarm {

    class Node {
    protected:
        int id = -1;
        std::vector<Camera *> camera = nullptr;
        std::map<int, DroneMarker> markers;

        bool has_vo = false;
        bool has_uwb = false;
        bool has_global_pose = false;
        bool has_armarkers = false;

        Pose global_pose;
        Eigen::Vector3d anntena_pos = Eigen::Vector3d(0, 0, 0);


        void load_cameras(std::string path) {
            //TODO:
            //Load camera from path
        }

        void load_markers(std::string path) {
            //TODO:
            //Load markers from file path
        }

    public:
        bool HasCamera() {
            return camera!=nullptr;
        }
        
        bool HasVO() {
            return has_vo;
        }

        bool HasUWB() {
            return has_uwb;
        }

        bool HasGlobalPose() {
            return has_global_pose;
        }

        Node(int id) {
            
        }

        static Node * createDroneNode(int _id, 
            std::string camera_config_path = "",
            std::string marker_config_path = ""
            ) {
            Node * node = new Node(_id);
            node->load_cameras(camera_config_path);
            node->has_vo = true;
            node->has_uwb = true;
            node->has_global_pose = false;
            return node;
        }

    };

    class NodeFrame {
    public:
        bool frame_available = false;
        bool vo_available = false;
        bool distances_available = false;
        bool corners_available = false;
        Node * node = nullptr;
        int id = -1;

        DisMap distance;
        Pose self_pose;
        Eigen::Vector3d self_vel;
        std::vector<corner_array> corner_by_cams;
        DroneFrame(Node * _node):
            node(_node)
        {

        }
    }

    class SwarmFrame {
    public:
        std::map<int, NodeFrame> node_map;
        std::map<int, DisMap> dis_mat;
        std::vector<int> node_id_list;
        int swarm_size() {
            return node_map.size();
        }
    };
};