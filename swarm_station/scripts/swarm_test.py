#!/usr/bin/env python
import rospy
from swarm_msgs.msg import data_buffer, swarm_drone_source_data, remote_uwb_info

def on_remote_node_info(info):
    print(info)

rospy.init_node('swarm_station')

uwb_recv_data = rospy.Subscriber("/uwb_node/remote_nodes", remote_uwb_info, on_remote_node_info, queue_size=1)

rospy.spin()