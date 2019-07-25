#!/usr/bin/env python
import copy
import math
import matplotlib.pyplot as plt
import random
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from swarm_msgs.msg import swarm_fused
# import rospy as loginfo
from visualization_msgs.msg import Marker


class SwarmVOVisual:
    def draw_arrow_marker(self, pos, vel, color, _id, r=1):
        # print(type(pos))
        # print(type(vel))
        pos_tgt = copy.deepcopy(pos)

        pos_tgt.x = vel.x*r
        pos_tgt.y = vel.y*r
        pos_tgt.z = vel.z*r

        total_len = math.sqrt(pos_tgt.x**2 + pos_tgt.y ** 2 + pos_tgt.z **2)

        vec_start = Point(0,0,0)

        mark = Marker()
        mark.type = mark
        mark.ns = "SwarmDrone{}".format(_id)
        mark.type = 0
        mark.action = 0
        mark.pose.position = pos
        mark.color.a = 1.0
        mark.color.r = color[0]
        mark.color.g = color[1]
        mark.color.b = color[2]
        mark.points = [vec_start, pos_tgt]
        mark.scale.x = 0.05
        mark.scale.y = total_len * 0.2
        mark.scale.z = total_len * 0.3
        mark.header.frame_id = "world_ned"

        return mark

    def on_fused_data(self, fused):
        for i in range(len(fused.ids)):
            id = fused.ids[i]
            pos = fused.remote_drone_position[i]
            vel = fused.remote_drone_velocity[i]
            mark = self.draw_arrow_marker(pos, vel, self.colors[id], id)
            self.pub.publish(mark)

    def on_cost_recv(self, cost):
        self.cost_array.append(cost.data)



    def __init__(self):
        self.pub = rospy.Publisher('/swarm_drones/drones_mark', Marker, queue_size=1)
        self.sub = rospy.Subscriber("/swarm_drones/swarm_drone_fused", swarm_fused, self.on_fused_data)
        self.sub2 = rospy.Subscriber("/swarm_drones/solving_cost", Float32, self.on_cost_recv)
        self.colors = [[
            random.uniform(0,1),
            random.uniform(0,1),
            random.uniform(0,1)
            ] for i in range(100)]

        self.cost_array = []

if __name__ == "__main__":
    plt.ion()
    plt.figure("Solving cost")
    print("Start swarm vo visualize")
    rospy.init_node("swarm_vo_visualize")
    visual = SwarmVOVisual()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # rate.sleep()
        plt.clf()
        plt.plot(visual.cost_array)
        plt.grid(which="both")
        plt.pause(0.1)
    rospy.spin()