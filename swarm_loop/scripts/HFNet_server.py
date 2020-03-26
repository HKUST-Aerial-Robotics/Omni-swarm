#!/usr/bin/env python
# Tensorflow
from __future__ import print_function

import rospy
import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
tf.contrib.resampler
from pathlib import Path
import cv2
import numpy as np
from swarm_loop.srv import HFNetSrv, HFNetSrvResponse

def imgmsg_to_cv2( msg ):
    assert msg.encoding == "8UC3" or msg.encoding == "8UC1" or msg.encoding == "bgr8" or msg.encoding == "mono8", \
        "Expecting the msg to have encoding as 8UC3 or 8UC1, received"+ str( msg.encoding )
    if msg.encoding == "8UC3" or msg.encoding=='bgr8':
        X = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        return X

    if msg.encoding == "8UC1" or msg.encoding=='mono8':
        X = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return X

class HFNet:
    def __init__(self, model_path, outputs):
        self.session = tf.Session()
        self.image_ph = tf.placeholder(tf.float32, shape=(None, None, 3))

        net_input = tf.image.rgb_to_grayscale(self.image_ph[None])
        tf.saved_model.loader.load(
            self.session, [tag_constants.SERVING], str(model_path),
            clear_devices=True,
            input_map={'image:0': net_input})

        graph = tf.get_default_graph()
        self.outputs = {n: graph.get_tensor_by_name(n+':0')[0] for n in outputs}
        self.nms_radius_op = graph.get_tensor_by_name('pred/simple_nms/radius:0')
        self.num_keypoints_op = graph.get_tensor_by_name('pred/top_k_keypoints/k:0')
        
    def inference(self, image, nms_radius=4, num_keypoints=1000):
        inputs = {
            self.image_ph: image[..., ::-1].astype(np.float),
            self.nms_radius_op: nms_radius,
            self.num_keypoints_op: num_keypoints,
        }
        return self.session.run(self.outputs, feed_dict=inputs)


class HFNetServer:
    def __init__(self, model_path, num_kpts=200, nms_radius = 4 ):
        outputs = ['global_descriptor', 'keypoints', 'local_descriptors']
        self.hfnet = HFNet(model_path, outputs)
        self.num_kpts = num_kpts
        self.nms_radius = nms_radius

        print("NFNet ready")
    
    def inference_network_on_image(self, img):
        ret = self.hfnet.inference(img, self.nms_radius, self.num_kpts)
        return ret["global_descriptor"], ret["keypoints"], ret["local_descriptors"]
    
    def handle_req(self, req):
        cv_image = imgmsg_to_cv2( req.image )
        global_desc, kpts, kp_descs = self.inference_network_on_image(cv_image)
        ret = HFNetSrvResponse()
        ret.global_desc = global_desc
        ret.keypoints = kpts
        ret.local_descriptors = kp_descs.flatten()
        return ret

if __name__ == "__main__":
    print("Initializing HFNet...")
    rospy.init_node( 'whole_image_descriptor_compute_server' )
    nms_radius = rospy.get_param('~nms_radius')
    num_keypoints = rospy.get_param('~num_keypoints')
    model_path = rospy.get_param('~model_path')

    hfserver = HFNetServer(model_path, num_keypoints, nms_radius)
    s = rospy.Service( 'whole_image_descriptor_compute_ts', HFNetSrv, hfserver.handle_req)
    rospy.spin()