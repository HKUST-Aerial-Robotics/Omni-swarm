#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from swarm_msgs.msg import swarm_fused_relative

import tf

class Converter:
    def __init__(self):        
        # self_odom_topic = "/vins_estimator/imu_propagate"
        self_odom_topic = "/swarm_mocap/SwarmNodeOdom0"
        self.vicon_odom = None

        self.vicon_sub = rospy.Subscriber(self_odom_topic, Odometry, self.self_vicon_cb, queue_size=1)

        self.vicon_sub = rospy.Subscriber(self_odom_topic, Odometry, self.self_vicon_cb, queue_size=1)

        self.relative_fused_sub = rospy.Subscriber("/swarm_drones/swarm_drone_fused_relative", swarm_fused_relative, self.relative_fused_cb, queue_size=1)

        self.publisher = {
            0:rospy.Publisher("/swarm_drones/est_odom_vicon_0", Odometry, queue_size=1),
            1:rospy.Publisher("/swarm_drones/est_odom_vicon_1", Odometry, queue_size=1),
            2:rospy.Publisher("/swarm_drones/est_odom_vicon_2", Odometry, queue_size=1),
            3:rospy.Publisher("/swarm_drones/est_odom_vicon_3", Odometry, queue_size=1),
            4:rospy.Publisher("/swarm_drones/est_odom_vicon_4", Odometry, queue_size=1),
            5:rospy.Publisher("/swarm_drones/est_odom_vicon_5", Odometry, queue_size=1)
        }

        self.trajpublisher = {
            0:rospy.Publisher("/swarm_drones/path_est_vicon_0", Path, queue_size=1),
            1:rospy.Publisher("/swarm_drones/path_est_vicon_1", Path, queue_size=1),
            2:rospy.Publisher("/swarm_drones/path_est_vicon_2", Path, queue_size=1),
            3:rospy.Publisher("/swarm_drones/path_est_vicon_3", Path, queue_size=1),
            4:rospy.Publisher("/swarm_drones/path_est_vicon_4", Path, queue_size=1),
            5:rospy.Publisher("/swarm_drones/path_est_vicon_5", Path, queue_size=1)
        }

        self.trajpublisher_vicon = {
            0:rospy.Publisher("/swarm_drones/path_vicon_0", Path, queue_size=1),
            1:rospy.Publisher("/swarm_drones/path_vicon_1", Path, queue_size=1),
            2:rospy.Publisher("/swarm_drones/path_vicon_2", Path, queue_size=1),
            3:rospy.Publisher("/swarm_drones/path_vicon_3", Path, queue_size=1),
            4:rospy.Publisher("/swarm_drones/path_vicon_4", Path, queue_size=1),
            5:rospy.Publisher("/swarm_drones/path_vicon_5", Path, queue_size=1)
        }

        self.vicon_path = {
            0:Path(),
            1:Path(),
            2:Path(),
            3:Path(),
            4:Path(),
            5:Path()
        }

        self.est_path = {
            0:Path(),
            1:Path(),
            2:Path(),
            3:Path(),
            4:Path(),
            5:Path()
        }
        
    def relative_fused_cb(self, sfr):
        # print("SFR stamp", sfr.header.stamp)
        # print("vicon stamp", self.vicon_odom.header.stamp)
        # print("DT", (sfr.header.stamp - self.vicon_odom.header.stamp).to_sec()*1000, "ms\n")

        # std_msgs/Header header
        # uint32[] ids
        # geometry_msgs/Point[] relative_drone_position
        # geometry_msgs/Vector3[] relative_drone_velocity
        # #Only yaw transformed
        # float64[] relative_drone_yaw
        for i in range(len(sfr.ids)):
            _id = sfr.ids[i]
            pos = sfr.relative_drone_position[i]
            vel = sfr.relative_drone_velocity[i]
            dyaw = sfr.relative_drone_yaw[i]
            qx, qy, qz, qw = tf.transformations.quaternion_from_euler(0, 0, dyaw)
            odomnew = Odometry()
            odomnew.header.stamp = sfr.header.stamp
            odomnew.header.frame_id = "odometry"
            odomnew.pose.pose.position.x = pos.x
            odomnew.pose.pose.position.y = pos.y
            odomnew.pose.pose.position.z = pos.z
            odomnew.pose.pose.orientation.w = qw
            odomnew.pose.pose.orientation.x = qx
            odomnew.pose.pose.orientation.y = qy
            odomnew.pose.pose.orientation.z = qz

            self.publisher[_id].publish(odomnew)
            
            """
            ps = PoseStamped()
            ps.pose = odomnew.pose.pose
            ps.header.stamp = odomnew.header.stamp
            ps.header.frame_id = "vicon0_drone"
            self.est_path[_id].poses.append(ps)
            self.est_path[_id].header.frame_id = "vicon0_drone"
            self.est_path[_id].header.stamp = odomnew.header.stamp
            self.trajpublisher[_id].publish(self.est_path[_id])
            """

    def self_vicon_cb(self, vicon_odom):
        self.vicon_odom = vicon_odom

        br = tf.TransformBroadcaster()
        br.sendTransform((vicon_odom.pose.pose.position.x, vicon_odom.pose.pose.position.y, vicon_odom.pose.pose.position.z),
                     (vicon_odom.pose.pose.orientation.x, vicon_odom.pose.pose.orientation.y, vicon_odom.pose.pose.orientation.z, vicon_odom.pose.pose.orientation.w ),
                     vicon_odom.header.stamp,
                     "odometry",
                     "world")

if __name__ == "__main__":
    rospy.init_node("relative_fused_vicon_convert")

    c = Converter()

    rospy.spin()