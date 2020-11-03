#!/usr/bin/env python3
import gym_duckietown
from gym_duckietown.simulator import Simulator
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
import cv2
import sys
import numpy as np
from cv_bridge import CvBridge


class DuckietownSimulator(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DuckietownSimulator, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.cap = cv2.VideoCapture(2)
        self.sub = rospy.Subscriber('wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.callback)
        self.pub = rospy.Publisher('camera_node/image/compressed', CompressedImage, queue_size=10)
        self._cvbr = CvBridge()
        rospy.loginfo("Started Duckietown Simulator ROS node...")

    def callback(self, msg):
        left_wheel = msg.vel_left
        right_wheel = msg.vel_right

        action = [left_wheel, right_wheel]
        observation, reward, done, misc = self._env.step(action)
        # self.env.render()
        if done:
            self._env.reset()
        observation = cv2.cvtColor(observation, cv2.COLOR_RGB2BGR)
        image_cmpr_msg = self._cvbr.cv2_to_compressed_imgmsg(observation)
        self.pub.publish(image_cmpr_msg)
    
    _env = Simulator(seed=123, # random seed
            map_name="loop_empty",
            max_steps=500001, # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=4, # start close to straight
            full_transparency=True,
            distortion=True
            )  

if __name__ == '__main__':
    # create the node
    
    node = DuckietownSimulator(node_name='duckietown_simulator')
    # run node
    # keep spinning
    rospy.spin()