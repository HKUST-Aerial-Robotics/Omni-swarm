#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
import time
from inf_uwb_ros.msg import *
import time
import math

class SwarmCommander():
    def __init__(self):
        rospy.init_node("count_recv")
        self.node_msg_count = {}
        self.incoming_data_sub = rospy.Subscriber("/uwb_node/incoming_broadcast_data", incoming_broadcast_data, self.on_remote_data, queue_size=100)
        self.dis_data_sub = rospy.Subscriber("/uwb_node/remote_nodes", remote_uwb_info, self.on_node_info, queue_size=100)
        self.recv_max = 0
        self.ts = None
        self.t = rospy.Timer(rospy.Duration(1.0), self.on_report)
        self.node_dis_count = {}
        self.tsInfo = None

    def on_report(self, e):
        if self.ts is None:
            return
        tc = 0
        dt = rospy.get_time() - self.ts
        for _id in self.node_msg_count:
            c = self.node_msg_count[_id]
            tc += c
            print("ID:{} FRE:{:3.1f}hz ".format(
                _id, float(c)/dt
            ), end="")
        
        print("TOTAL C {} FRE:{:3.1f}\n DIS:".format(tc, float(tc)/dt))
        
        tc = 0
        for _id in self.node_dis_count:
            c = self.node_dis_count[_id]
            tc += c
            print("ID:{} FRE:{:3.1f}hz ".format(
                _id, float(c)/dt
            ), end="")
        
        print("TOTAL C {} FRE:{:3.1f}\n".format(tc, float(tc)/dt))
    
    def on_node_info(self, data):
        if self.tsInfo is None:
            self.tsInfo = rospy.get_time()
        for i in range(len(data.node_ids)):
            _id = data.node_ids[i]
            avail = data.active[i]
            if avail:
                if not (_id in self.node_dis_count):
                    self.node_dis_count[_id] = 0
                self.node_dis_count[_id] += 1

    def on_remote_data(self, data):
        if self.ts is None:
            self.ts = rospy.get_time()
        """
        Header header
        uint32 remote_id
        uint32 remote_recv_time
        uint32 lps_time
        uint8[] data """
        # self.parse_data(data.data, data.remote_id, data.lps_time)
        if not (data.remote_id in self.node_msg_count):
            self.node_msg_count[data.remote_id] = 0
        self.node_msg_count[data.remote_id] += 1

        if self.node_msg_count[data.remote_id] > self.recv_max:
            self.recv_max = self.node_msg_count[data.remote_id]



if __name__ == "__main__":
    sw_cmd = SwarmCommander()
    rospy.loginfo("start swarm commander")
    rospy.spin()
