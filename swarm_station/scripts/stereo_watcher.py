#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

class StereoWatcher:
    def __init__(self):
        self.lsub = rospy.Subscriber("/stereo/left/image", Image, self.on_image_l)
        self.rsub = rospy.Subscriber("/stereo/right/image", Image, self.on_image_r)
        self.last_img_ts_l = rospy.get_time()
        self.last_img_ts_r = rospy.get_time()
    
    def on_image_l(self, img):
        self.last_img_ts_l = rospy.get_time()

    def on_image_r(self, img):
        self.last_img_ts_r = rospy.get_time()
    
    def update(self):
        dt = rospy.get_time() - self.last_img_ts_l 
        if dt > 0.06:
            print("Loss L {:3.1f}ms".format(dt*1000))

        dt = rospy.get_time() - self.last_img_ts_r
        if dt > 0.06:
            print("Loss R {:3.1f}ms".format(dt*1000))

if __name__ == "__main__":
    rospy.init_node("stereo_watchdog")
    sw = StereoWatcher()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        sw.update()
        rate.sleep()