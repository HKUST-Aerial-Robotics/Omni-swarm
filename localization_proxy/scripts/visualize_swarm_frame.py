#!/usr/bin/python

import rospy
from swarm_msgs.msg import swarm_frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose

class visuallizer:
    def __init__(self):
        self.predict_sub = rospy.Subscriber("/swarm_drones/swarm_frame", swarm_frame, self.predict_frame_callback, tcp_nodelay=True, queue_size=100)
        self.node_vo_predict_pub = {
            0: rospy.Publisher("/vo0", Odometry,queue_size=1),
            2: rospy.Publisher("/vo2", Odometry,queue_size=1),
            3: rospy.Publisher("/vo3", Odometry,queue_size=1),
            4: rospy.Publisher("/vo4", Odometry,queue_size=1)
        }

        self.node_detect_predict_pub = {
            2: rospy.Publisher("/pose2", PoseStamped, queue_size=1),
            3: rospy.Publisher("/pose3", PoseStamped, queue_size=1),
            4: rospy.Publisher("/pose4", PoseStamped, queue_size=1),
        }


    
    def predict_frame_callback(self, sf):
        detected = []
        for _nf in sf.node_frames:
            if _nf.vo_available:
                odom = Odometry()
                odom.header.stamp = _nf.header.stamp
                odom.pose.pose.position = _nf.position
                odom.twist.twist.linear = _nf.velocity
                odom.header.frame_id = "world"
                self.node_vo_predict_pub[_nf.drone_id].publish(odom)
                # if _nf.drone_id == 3:
                    # print(_nf.position.z)
                for d in _nf.detected:
                    if d.self_drone_id == 3 or d.remote_drone_id == 3:
                        # detected.append("Z {} CAM {} MAR {} DZ {}".format(_nf.position.z, d.self_drone_id, d.remote_drone_id, d.dpos.z))
                        detected.append(sf)
                        if _nf.drone_id ==0 :
                            p = Pose()
                            p.position.x = odom.pose.pose.position.x + d.dpos.x
                            p.position.y = odom.pose.pose.position.y + d.dpos.y
                            p.position.z = odom.pose.pose.position.z + d.dpos.z
                            ps = PoseStamped()
                            ps.pose = p
                            ps.header.stamp = odom.header.stamp
                            ps.header.frame_id = "world"
                            self.node_detect_predict_pub[d.remote_drone_id].publish(ps)

        if len(detected) > 0:
            print(detected)

if __name__ == "__main__":
    rospy.init_node("visual_swame_frame")    
    vis = visuallizer()
    rospy.spin()