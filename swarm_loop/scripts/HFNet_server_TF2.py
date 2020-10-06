#!/usr/bin/env python3
# Tensorflow
from __future__ import print_function

import rospy
import tensorflow as tf
from tensorflow.python.saved_model import tag_constants
from tensorflow.python.saved_model import signature_constants
import tensorflow_addons as tfa
from pathlib import Path
import cv2
import numpy as np
from swarm_loop.srv import HFNetSrv, HFNetSrvResponse
from geometry_msgs.msg import Point32
import time

tfa.register.register_all()

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
    def __init__(self, model_path):
        self.func = self.load(model_path)

    def load(self, output_saved_model_dir):
        saved_model_loaded = tf.saved_model.load(
            output_saved_model_dir, tags=[tag_constants.SERVING])

        frozen_func = graph_func = saved_model_loaded.signatures[
        signature_constants.DEFAULT_SERVING_SIGNATURE_DEF_KEY]
        return frozen_func
    
    def inference(self, img):
        _img = np.expand_dims(img, axis=2)
        _img = np.array([_img]).astype(np.float)
        _img = tf.convert_to_tensor(_img, dtype=tf.float32)
        start_time = time.time()
        output = self.func(_img)
        print( f'Inference hfnet {img.shape} in {( 1000. *(time.time() - start_time) ) }fms')

        return output


class HFNetServer:
    def __init__(self, model_path):
        self.hfnet = HFNet(model_path)

        tmp_zer = np.random.randn(208, 400)
        self.inference_network_on_image(tmp_zer)
        print("NFNet ready")
    
    def inference_network_on_image(self, img):
        ret = self.hfnet.inference(img)
        return ret["global_descriptor"][0].numpy(), ret["keypoints"][0].numpy(), ret["local_descriptors"][0].numpy()
    
    def handle_req(self, req):
        start_time = time.time()

        cv_image = imgmsg_to_cv2( req.image )
        global_desc, kpts, kp_descs = self.inference_network_on_image(cv_image)
        ret = HFNetSrvResponse()
        ret.global_desc = global_desc
        _kpts = []
        for pt in kpts:
            kp = Point32(pt[0], pt[1], 0)
            _kpts.append(kp)
        ret.keypoints = _kpts
        print(kp_descs.shape)
        ret.local_descriptors = kp_descs.flatten()

        print( 'HFNet return req in %4.4fms' %( 1000. *(time.time() - start_time) ) )
        return ret

def set_memory_limit(memory_limit):   
    gpus = tf.config.experimental.list_physical_devices('GPU')
    if gpus:
        # Restrict TensorFlow to only allocate 1GB of memory on the first GPU
        try:
            tf.config.experimental.set_virtual_device_configuration(
                gpus[0],
                [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=memory_limit)])
            logical_gpus = tf.config.experimental.list_logical_devices('GPU')
            print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
        except RuntimeError as e:
            # Virtual devices must be set before GPUs have been initialized
            print(e)





if __name__ == "__main__":
    print("Initializing HFNet... with tensorflow {}".format(tf.__version__))
    rospy.init_node( 'hfnet_server' )
    model_path = rospy.get_param('~model_path')
    memory_limit = rospy.get_param('~memory_limit')
    
    set_memory_limit(memory_limit)
    
    hfserver = HFNetServer(model_path)
    s = rospy.Service( '/swarm_loop/hfnet', HFNetSrv, hfserver.handle_req)
    rospy.spin()