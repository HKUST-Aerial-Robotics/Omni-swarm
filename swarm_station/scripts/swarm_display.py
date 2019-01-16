#!/usr/bin/env python
import rospy
import visualization_msgs
from visualization_msgs.msg import Marker
import math
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path, Odometry

def gen_marker(pose, stamp=None):
    marker = Marker()
    marker.header.frame_id = "world"
    if stamp is None:
        marker.header.stamp = rospy.get_rostime()
    else:
        marker.header.stamp = stamp

    marker.ns = "swarm_drone_2"
    marker.id = 0
    marker.type = 10
    marker.action = 0
    marker.pose = pose
    marker.scale.x = 0.001
    marker.scale.y = 0.001
    marker.scale.z = 0.001
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.mesh_resource = "package://swarm_station/models/dddk.stl"
    return marker


class DroneVis:
    def __init__(self):
        self.path_pub = rospy.Publisher("/swarm_drone/drone_2_path", Path, queue_size=1, tcp_nodelay=True)
        self.vis_pub = rospy.Publisher("/swarm_drone/drone_2", Marker, queue_size=1, tcp_nodelay=True)
        self.odom_sub = rospy.Subscriber("/swarm_drone/odom_7", Odometry, self.odom_callback, tcp_nodelay=True)
        self.path = Path()
        self.path.header.frame_id = "world"
        self.path_last_ts = rospy.get_time()

    def odom_callback(self, odom):
        # print(odom)
        m = gen_marker(odom.pose.pose, odom.header.stamp)
        
        self.path.header.stamp = odom.header.stamp

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = odom.header.stamp
        pose_stamped.pose = odom.pose.pose
        
        if rospy.get_time() -self.path_last_ts>0.1:
            self.path_last_ts = rospy.get_time()
            self.path.poses.append(pose_stamped)
            if len(self.path.poses) > 500:
                self.path.poses = self.path.poses[1:-1]
            self.path_pub.publish(self.path)
        
        self.vis_pub.publish(m)

if __name__ == "__main__":
    print("Start swarm_display")
    rospy.init_node("swarm_display")
    dv = DroneVis()
    rospy.spin()