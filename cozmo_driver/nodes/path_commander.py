# system
import sys
import numpy as np
from copy import deepcopy

# ROS
import rospy
from tf import TransformListener
from nav_msgs.msg import Path
from mrdrrt.srv import PrmSrv

class PathCommander:
    def __init__(self, *args):
        self.tf = TransformListener()
        rospy.Subscriber(... etc

    def some_method(self):
        if self.tf.frameExists("/base_link") and self.tf.frameExists("/map"):
            t = self.tf.getLatestCommonTime("/base_link", "/map")
            position, quaternion = self.tf.lookupTransform("/base_link", "/map", t)
            print position, quaternion
