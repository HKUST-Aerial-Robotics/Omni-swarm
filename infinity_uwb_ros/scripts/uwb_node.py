#!/usr/bin/env python
from __future__ import print_function

import rospy
from uwb_helper import UWBHelperNode
from infinity_uwb_ros.msg import remote_uwb_info, incoming_broadcast_data
from infinity_uwb_ros.srv import broadcast_msg
from std_msgs.msg import String

class UWBNodeROSNode:
    def __init__(self):
        serial_name = rospy.get_param('~serial_name', "/dev/ttyS6")
        baudrate = rospy.get_param('~baudrate', 921600)
        print("Starting uwb with", serial_name,":", baudrate)
        self.info_pub = rospy.Publisher('~remote_nodes', remote_uwb_info, queue_size=1)
        self.data_pub = rospy.Publisher('~incoming_broadcast_data', incoming_broadcast_data, queue_size=1)
        self.count = 0

        self.uwbhelper = UWBHelperNode(serial_name, baud_rate=baudrate, 
            update_cb=self.on_data_updated, data_cb=self.on_bddata_recevied)

        self.srv = rospy.Subscriber('~send_broadcast_data', String, self.handle_broadcast_msg)
        self.tm = rospy.Timer(rospy.Duration(0.001), self.update)

        self.broadcast_pending_data = ""
        self.broadcast_max_send_size = 250
        self.tm_broadcast = rospy.Timer(rospy.Duration(0.02), self.send_broadcast_data)

    def send_broadcast_data(self, dt):
        if self.broadcast_pending_data == "":
            # print("Not send")
            return
        
        if len(self.broadcast_pending_data) < self.broadcast_max_send_size:
            # print("Will send", self.broadcast_pending_data)
            self.uwbhelper.broadcast_data(self.broadcast_pending_data)

            self.broadcast_pending_data = ""
        else:
            # print("Send", self.broadcast_pending_data[0:self.broadcast_max_send_size])

            self.uwbhelper.broadcast_data(self.broadcast_pending_data[0:self.broadcast_max_send_size])
            self.broadcast_pending_data = self.broadcast_pending_data[self.broadcast_max_send_size:]


    def on_data_updated(self, uwbhelper):
        infos = remote_uwb_info()
        infos.sys_time = uwbhelper.sys_time
        infos.self_id = uwbhelper.self_id
        infos.remote_node_num = len(uwbhelper.nodes_info)
        for _id in uwbhelper.nodes_info:
            infos.node_ids.append(_id)
            inf = uwbhelper.nodes_info[_id]
            infos.node_dis.append(inf["distance"])
            infos.recv_distance_time.append(inf["distance_time"])
            infos.active.append(inf['active'])
            infos.datas.append(inf['data'])
            infos.data_available.append(len(inf['data']) > 0)
            infos.rssi.append(inf['rssi'])

        infos.header.stamp = rospy.Time.now()
        self.info_pub.publish(infos)
        self.count = self.count + 1

        if self.count % 50 == 1:
            print("Count {} ID {} SYSTIME {} nodes {} avail {}".format(
                self.count,uwbhelper.self_id, uwbhelper.sys_time, infos.remote_node_num, uwbhelper.available_node_num))

    def on_bddata_recevied(self, _id, _msg, _lps_time, _recv_time):
        bd = incoming_broadcast_data()
        bd.header.stamp = rospy.Time.now()
        bd.remote_id = _id
        bd.remote_recv_time = _recv_time
        bd.lps_time = _lps_time
        bd.data = _msg
        # print(bd.data, end="")
        self.data_pub.publish(bd)

    def update(self, t):
        self.uwbhelper.update_from_buf(read_all=True)

    def handle_broadcast_msg(self, msg):
        msg = msg.data
        self.broadcast_pending_data = self.broadcast_pending_data + msg
        return 1
        
    

       
if __name__ == "__main__":
    try:
        rospy.init_node('uwb', disable_signals=False)
        uwbn = UWBNodeROSNode()
        rospy.spin()

    except KeyboardInterrupt, IOError as inst:
        uwbn.uwbhelper.close()
        print("Exit by keyboard")
        exit(0)
    except:
        # uwbn.uwbhelper.close()
        raise
        exit(0)
