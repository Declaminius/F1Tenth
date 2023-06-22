#!/usr/bin/env python3

#ROS Imports
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


class PointWrapper:
    def __init__(self, point):
        assert isinstance(point, Point)

        self.point = point
        self.min_speed = 0.5
        self.ttc = 0
        self.margin = 0.3
        self.velocity=0