#/usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry

rate = rospy.Rate(1) # 10hz


class drone_state:
    def __init__(self):
        rospy.init_node("~drone_status")
        self.fc_quat = None
        self.battery_state = None
        self.odom = None

if __name__ == "__main__":



