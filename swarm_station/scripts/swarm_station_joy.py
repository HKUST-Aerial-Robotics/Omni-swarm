#!/usr/bin/env python
from __future__ import print_function
import rospy
# import pygame
import pymavlink
import sys
from pymavlink4swarm import MAVLink
import pymavlink4swarm as pymavlink
import time
from swarm_msgs.msg import data_buffer, swarm_drone_source_data, remote_uwb_info
import time
from geometry_msgs.msg import Pose, PoseStamped
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)


class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def print(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10


# pygame.init()
# pygame.joystick.init()

# size = [300, 200]
# screen = pygame.display.set_mode(size)
# pygame.display.set_caption("SwarmCMD")
# textPrint = TextPrint()

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

class SwarmCommander():

    def __init__(self):
        f = fifo()
        self.mav = MAVLink(f)
        rospy.init_node('swarm_station')
        # self.pub = rospy.Publisher('/uwb_node/send_broadcast_data', data_buffer, queue_size=10)
        self.reference_node = 7
        self.remote_node_list = []
        self.remote_node_pub = {}

        self.remote_node_vo_pose_pubs = {}

        self.vicon_pose = Pose()
        self.vicon_pose_sub = rospy.Subscriber("/SwarmNode7/pose", PoseStamped, self.on_vicon_pose_recv, queue_size=1);

        self.self_vo_pose = Pose()
        self.self_vo_yaw = 0
        self.uwb_recv_data = rospy.Subscriber("/uwb_node/remote_nodes", remote_uwb_info, self.on_remote_node_info, queue_size=1)
        
    def on_remote_node_info(self, info):
        ptr = 0
        for i in range(len(info.node_ids)):
            if info.node_ids[i] == self.reference_node:
                ptr = i
                break
        for i in range(len(info.node_ids)):
            _id = info.node_ids[i]
            if info.data_available[i]:
                self.parse_data(info.datas[i].data, _id)
            # print("Finish parse data")

    def on_vicon_pose_recv(self, pose):
        self.vicon_pose = pose.pose

    def toggle_arm_disarm(self, target):
        msg = self.mav.command_long_encode(target, 0, pymavlink.MAV_CMD_DO_SET_MODE, 0, 192, 0, 0, 0, 0, 0, 0)
        self.send_mavlink_msg(msg)
    
    def takeoff_cmd(self, target):
        yawAngle = float('nan')
        msg = self.mav.command_long_encode(target, 0, pymavlink.MAV_CMD_NAV_TAKEOFF_LOCAL, 0, 0, 0, 0, yawAngle, 0, 0, 0.5)
        self.send_mavlink_msg(msg)

    def landing_cmd(self, target):
        yawAngle = float('nan')
        msg = self.mav.command_long_encode(target, 0, pymavlink.MAV_CMD_NAV_TAKEOFF_LOCAL, 0, 0, 0, 0.3, yawAngle, 0, 0, 0)
        self.send_mavlink_msg(msg)
    
    def pathplaning_cmd(self, target):
        msg = self.mav.command_long_encode(target, 0, pymavlink.MAV_CMD_NAV_PATHPLANNING, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_mavlink_msg(msg)
        #Random fly to test our fuse algorithms

    def send_manual_control(self, rc_state):
        target = rc_state.target_id
        x = int(rc_state.rcA * 1000)
        y = int(rc_state.rcE * 1000)
        z = int(rc_state.rcT * 1000)
        r = int(rc_state.rcR * 1000) 
        buttons = 4*rc_state.sw1 + 2*rc_state.sw2 + rc_state.sw3
        msg = self.mav.manual_control_encode(target, x, y, z, r, buttons)
        self.send_mavlink_msg(msg)

    def send_mavlink_msg(self, msg):
        buf = msg.pack(self.mav, force_mavlink1=False)
        _buf = data_buffer
        _buf.data = buf
        # self.pub.publish(_buf)
        #print(buf)
    
    def swarm_relative_fused_recv(self, msg):
        # print(msg)
        source_id = msg.source_id
        target_id = msg.target_id
        if source_id == 7 and target_id !=7:
            self.remote_node_list.append(target_id)
            if target_id not in self.remote_node_pub:
                self.remote_node_pub[target_id] = rospy.Publisher("/swarm_drone/estimate_pose_{}".format(target_id), PoseStamped);

            _pose = PoseStamped()
            remote_pose = _pose.pose
            _pose.header.frame_id = "world"
            _pose.header.stamp = rospy.Time.now()
            remote_pose.position.x = self.vicon_pose.position.x + msg.rel_x
            remote_pose.position.y = self.vicon_pose.position.y + msg.rel_y
            remote_pose.position.z = self.vicon_pose.position.z + msg.rel_z

            remote_pose.orientation.w = 1
            self.remote_node_pub[target_id].publish(_pose)
    
    def swarm_vo_info_recv(self, msg, is_self_node=False):
        if is_self_node:
            self.self_vo_pose.position.x = msg.x
            self.self_vo_pose.position.y = msg.y
            self.self_vo_pose.position.z = msg.z

            self.self_vo_pose.orientation.w = msg.q0
            self.self_vo_pose.orientation.x = msg.q1
            self.self_vo_pose.orientation.y = msg.q2
            self.self_vo_pose.orientation.z = msg.q3

            quaternion = (msg.q1, msg.q2, msg.q3, msg.q0)
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            self.self_vo_yaw = yaw
        # print(self.self_vo_pose)


    def parse_data(self, s, _id):
        try:
            is_ref_node = _id == self.reference_node
            msg = self.mav.parse_char(s)
            if msg is not None:
                if msg.get_type() == "SWARM_RELATIVE_FUSED":
                    print("Rel",msg)
                    if is_ref_node:
                        self.swarm_relative_fused_recv(msg)
                if msg.get_type() == "SWARM_INFO":
                    # print(" {}: {:3.2f} {:3.2f} {:3.2f}".format(_id, msg.x, msg.y, msg.z))
                    self.swarm_vo_info_recv(msg, is_ref_node)

        except Exception as inst:
            print(inst)
            pass
class RCState():
    def __init__(self):
        self.target_id = 255
        self.rcA = 0
        self.rcE = 0
        self.rcR = 0
        self.rcT = 0
        self.rcAUX1 = 0
        self.rcAUX2 = 0
        self.rcAUX3 = 0
        self.sw1 = 0
        self.sw2 = 0
        self.sw3 = 0

        self.sw_cmd = None

        self.joystick = pygame.joystick.Joystick(0)

        self.button_pressed_time = {}
    
    def pack_mavlink(self):
        return

    def release_button(self, button, hold_time):
        print("Button {} hold {}".format(button, hold_time))
        if button == 11:
            #Hold A
            if hold_time > 1.0:
                self.sw_cmd.toggle_arm_disarm(self.target_id)
                print("Try arm disarm drone {}".format(self.target_id))


    def update(self):

        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                # done=True # Flag that we are done so we exit this loop
                exit(0)
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                # print(event.button)
                # print("Joystick button pressed.")
                self.button_pressed_time[event.button] = time.time()

            if event.type == pygame.JOYBUTTONUP:
                # print(event.button)
                # print("Joystick button released.")
                self.release_button(event.button, time.time() - self.button_pressed_time[event.button])
        
        joystick = self.joystick
        joystick.init()
        self.rcA = joystick.get_axis(2)
        self.rcE = - joystick.get_axis(3)
        self.rcR = joystick.get_axis(0)
        self.rcT = -joystick.get_axis(1)
        self.rcAUX1 = joystick.get_axis(4)
        self.rcAUX2 = joystick.get_axis(5)

        #Right banji 5
        #Left banji 4

        # Left btn 8
        # Right btn 9

 

        self.btnA = joystick.get_button(11)

        textPrint.print(screen, "rcA {:3.2f} rcE {:3.2f} rcR {:3.2f} rcT {:3.2f}".format(
            self.rcA, self.rcE, self.rcR, self.rcT, self.rcAUX1, self.rcAUX2))
        textPrint.print(screen, "btn {} {} {}".format(self.sw1, self.sw2, self.sw3))
        # print("rcA {:3.2f} rcE {:3.2f} rcR {:3.2f} rcT {:3.2f} rcAUX1 {:3.2f} rc AUX2 {:3.2f}".format(
            # self.rcA, self.rcE, self.rcR, self.rcT, self.rcAUX1, self.rcAUX2))
        # A 11
        # B 12
        # Y 14
        # X 13
        self.sw1 = joystick.get_button(13)
        self.sw2 = joystick.get_button(14)

     
if __name__ == "__main__":
    # rcs = RCState()
    sw_cmd = SwarmCommander()
    # rcs.sw_cmd = sw_cmd
    # t = threading.Thread(target = rospy.spin)
    # t.start()
    rospy.spin()
    """
    while True:
        try:
            screen.fill(WHITE)
            textPrint.reset()
            rcs.update()
            sw_cmd.send_manual_control(rcs)
            time.sleep(0.02)
            pygame.display.flip()
        except KeyboardInterrupt:
            exit(0)
    
    """