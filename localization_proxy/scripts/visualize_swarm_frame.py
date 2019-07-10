#!/usr/bin/python

import rospy
from swarm_msgs.msg import swarm_frame
from nav_msgs.msg import Odometry

class visuallizer:
    def __init__(self):
        self.predict_sub = rospy.Subscriber("/swarm_drones/swarm_frame", swarm_frame, self.predict_frame_callback, tcp_nodelay=True, queue_size=1)
        self.node_vo_predict_pub = {
            0: rospy.Publisher("/vo0", Odometry,queue_size=1),
            2: rospy.Publisher("/vo2", Odometry,queue_size=1),
            4: rospy.Publisher("/vo4", Odometry,queue_size=1),
            3: rospy.Publisher("/vo3", Odometry,queue_size=1)}

    
    def predict_frame_callback(self, sf):
        detected = []
        for _nf in sf.node_frames:
            if _nf.vo_available:
                odom = Odometry()
                odom.header.stamp = _nf.header.stamp
                odom.pose.pose.position = _nf.position
                odom.twist.twist.linear = _nf.velocity
                odom.header.frame_id = "world"
                self.node_vo_predict_pub[_nf.id].publish(odom)
                for d in _nf.detected.detected_nodes:
                    detected.append("CAM {} MAR {}".format(d.self_drone_id, d.remote_drone_id))
        if len(detected) > 0:
            print(detected)

if __name__ == "__main__":
    rospy.init_node("visual_swame_frame")    
    vis = visuallizer()
    rospy.spin()