#!/usr/bin/env python2
from __future__ import print_function
import rospy
import rosbag
import sys
from rosgraph_msgs.msg import Clock
import time

class SyncBagPlayer:
    def __init__(self, bag_path, play_t0_sys=None):
        self.bag = rosbag.Bag(bag_path)
        self.prepare_publishers()
        self.play_t0_sys = play_t0_sys
        self.play_t0_bag = None
        self.r = rospy.Rate(10000)
        self.total_time = self.bag.get_end_time() - self.bag.get_start_time()
        print("Start sys time", play_t0_sys, "Now time", time.time())
        
    def prepare_publishers(self):
        bag = self.bag
        topics = bag.get_type_and_topic_info()[1]
        publishers = {}
        #Prepare publishers for bag
        for topic, msg, t in bag.read_messages():
            if topic not in publishers:
                publishers[topic] = rospy.Publisher(topic, type(msg), queue_size=10)
            if len(topics) == len(publishers):
                break
        print("Pubs", publishers.keys())
        self.publishers = publishers
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
    
    def play(self):
        count = 0
        for topic, msg, t in self.bag.read_messages():
            try:
                if rospy.is_shutdown():
                    print("Rospy shut down")
                    break
                if self.play_t0_sys is None:
                    self.play_t0_sys = time.time()
                if self.play_t0_bag is None:
                    self.play_t0_bag = t
                while time.time() - self.play_t0_sys < (t - self.play_t0_bag).to_sec():
                    #print(time.time() - self.play_t0_sys, (t - self.play_t0_bag).to_sec())
                    time.sleep(0.0001)
                self.publishers[topic].publish(msg)
                sim_clock = Clock()
                sim_clock.clock = t
                self.clock_pub.publish(sim_clock)
                if count % 100 == 0:
                    t_ = (time.time()- self.play_t0_sys)
                    progress = t_ / self.total_time * 100
                    print("Time {:5.2f}/{:5.2f} Progress {:3.2f}% Count {}".format(t_, self.total_time, progress, count), end="\r")
                    sys.stdout.flush()
                count += 1
            except KeyboardInterrupt:
                print("User break")
                break

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Play bags synchronously from dockers')
    parser.add_argument('--path', type=str,
                    help='bagpath')
    parser.add_argument('--syst', type=float,
                    help='bagpath')
    args = parser.parse_args()
    print("Will play bag from", args.path, ", start at system time", args.syst)
    rospy.init_node("player")
    player = SyncBagPlayer(args.path, args.syst)
    player.play()