import rospy
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import euler_from_quaternion
import numpy as np

def on_quat(msg):
    q = np.array([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
    r, p, y = euler_from_quaternion(q)
    print("Yaw {:3.2f} {:3.2f} {:3.2f}".format(y*57.3, p*57.3, r*57.3))

if __name__ == "__main__":
    rospy.init_node("YAW_WATCH")
    sub = rospy.Subscriber("/dji_sdk_1/dji_sdk/attitude", QuaternionStamped, on_quat, queue_size=1)
    rospy.spin()
