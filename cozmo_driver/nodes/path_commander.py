# system
import sys
import numpy as np
from copy import deepcopy
import math

# ROS
import rospy
from tf import TransformListener
from nav_msgs.msg import Path
from geometry_msgs.msg import (
    Twist,
    TransformStamped
)

#others
from mrdrrt.srv import PrmSrv
from transformations import euler_from_quaternion

class PathCommander:
    def __init__(self, *args):
        self.tf = TransformListener()
        # self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback, queue_size=1)
        self.path = rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        self.twist_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.waypoints = []
        

    def get_pose(self):
        if self.tf.frameExists("base_link") and self.tf.frameExists("world"):
            t = self.tf.getLatestCommonTime("base_link", "world")
            position, quaternion = self.tf.lookupTransform("base_link", "world", t)
            theta = euler_from_quaternion(quaternion)
            x = position[0]
            y = position[1]
            return x, y, theta

        else:
            ROS_WARN("No Valid Transform Available")
            return None



    def path_callback(self, path):
        self.waypoints = path.poses



    def goToWaypoint(self, waypoint):
        curr_x, curr_y , curr_th = self.get_pose()

        goal_x = waypoint.pose.position.x
        goal_y = waypoint.pose.position.y
        goal_th = euler_from_quaternion(waypoint.pose.orientation)

        if not (self.goalReached()):
            d_theta = theta - math.atan2((goal_y-curr_y),(goal_x-curr)x);
            cozmo.turnInPlace(d_theta, 1); #turn towards goal, anglesinrad



    def goalReached(self):