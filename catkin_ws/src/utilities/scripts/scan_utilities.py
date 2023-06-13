#!/usr/bin/env python3
import math

#ROS Imports
import rospy

def myScanAngle(self, scan_msg, index):
    # returns angle in radians

    return scan_msg.angle_min + index * scan_msg.angle_increment

def myScanIndex(self, scan_msg, angle):
    # expects an angle in degrees and outputs the respective index in the scan_ranges array.
    # angle: between -135 to 135 degrees, where 0 degrees is directly to the front

    rad_angle = angle * math.pi / 180.0

    if not (scan_msg.angle_min <= rad_angle <= scan_msg.angle_max):
        rospy.loginfo("ANGLE out of range")
        return 540

    index = int((rad_angle - scan_msg.angle_min) / scan_msg.angle_increment)

    return index

def myGetRange(self, scan_msg, angle):
    # scan_msg: single message from topic /scan
    # angle: between -135 to 135 degrees, where 0 degrees is directly to the front
    # Outputs length in meters to object with angle in lidar scan field of view

    index = self.myScanIndex(scan_msg, angle)

    if scan_msg.ranges[index] == math.inf or scan_msg.ranges[index] == math.nan: 
        rospy.loginfo("No LIDAR data for angle")
        return 100.

    range = scan_msg.ranges[index]
    return range