#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np
import copy
from itertools import groupby

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker


#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class FollowTheGap:
    """ Implements the follow the gap algorithm
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.fast_speed = 0.75
        self.medium_speed = 0.5
        self.slow_speed = 0.25

        self.actual_speed = 0

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.actual_speed_pub = rospy.Publisher("/actual_speed_topic", Float32, queue_size = 1000)
        self.steering_angle_pub = rospy.Publisher("/steering_angle_topic", Float32, queue_size = 1000)
        self.desired_dist_pub = rospy.Publisher("/desired_dist_topic", Float32, queue_size = 1000)
        self.desired_speed_pub = rospy.Publisher("/desired_speed_topic", Float32, queue_size = 1000)
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 1000)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        self.gap_pub = rospy.Publisher("/gap_viz", LaserScan, queue_size = 1)

    def get_scan_index(self, scan_msg, angle):

        angle_degrees = angle * 180 / np.pi
        position = (angle_degrees + 45) / 270
        length = len(scan_msg.ranges)
        index = int(length * position)
        return index        
    
    def get_scan_angle(self, scan_msg, index):
        length = len(scan_msg.ranges)
        angle = ((index / length * 270) - 45)/180*np.pi
        return angle


    def lidar_callback(self, scan_msg):
        self.follow_the_gap(scan_msg)
        
    
    def follow_the_gap(self, scan_msg):
        threshold = 2
        safety_bubble = 100
        speed = 0.5
        n = len(scan_msg.ranges)
        min_angle = 0
        max_angle = 180


        forward_ranges = np.array(scan_msg.ranges)

        # forward_ranges[:self.get_scan_index(scan_msg, min_angle)] = float("inf")
        # forward_ranges[self.get_scan_index(scan_msg, max_angle):] = float("inf")

        min_index = np.argmin(scan_msg.ranges)

        forward_ranges[:self.get_scan_index(scan_msg, min_angle)] = 0
        forward_ranges[self.get_scan_index(scan_msg, max_angle):] = 0
    
        forward_ranges[forward_ranges < threshold] = 0

        # find maximal subarray of consecutive non-zeroes
        max_gap_start = 0
        max_gap_length = 0
        gap_start = 0
        gap_length = 0


        for (i, scan_range) in enumerate(forward_ranges):
            if scan_range > 0:
                gap_length += 1
            else:
                if gap_length > max_gap_length:
                    max_gap_length = gap_length
                    max_gap_start = gap_start
                gap_start = i + 1
                gap_length = 0
        if gap_length > max_gap_length:
            max_gap_length = gap_length
            max_gap_start = gap_start
        
        max_gap_end = max_gap_start + max_gap_length

        best_point = np.argmax(forward_ranges[max_gap_start:max_gap_end]) + max_gap_start
        print(best_point)
        
        #best_point = np.argmax(scan_msg.ranges)

        self.best_distance = forward_ranges[best_point]



        steering_angle = self.compute_steering_angle(scan_msg, best_point)
        #print(f"{steering_angle=}")
        #print(f"{max_gap_start=}")

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.desired_speed_pub.publish(speed)
        self.drive_pub.publish(drive_msg)
        self.steering_angle_pub.publish(steering_angle*180/np.pi)


        scan_msg.intensities = forward_ranges
        scan_msg.intensities[max_gap_start:max_gap_end] = 5
        self.gap_pub.publish(scan_msg)

        self.visualize_point([self.best_distance*np.cos(steering_angle), self.best_distance*np.sin(steering_angle)])


        
        self.marker_viz(speed, steering_angle)


    def compute_steering_angle(self, scan_msg, best_point):
        steering_angle = self.get_scan_angle(scan_msg, best_point) - np.pi/2
        return steering_angle

    def odom_callback(self, odom_msg):
        self.actual_speed = odom_msg.twist.twist.linear.x
        self.actual_speed_pub.publish(self.actual_speed)


    def visualize_point(self,pt,frame='laser',r=1.0,g=0.0,b=1.0):

        x = float(pt[0])
        y = float(pt[1])
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.id = 150
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.lifetime = rospy.Duration(0.1)
        self.marker_pub.publish(marker)

    def marker_viz(self, speed, steering_angle):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 0
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.0

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = np.cos(steering_angle)
        marker.pose.orientation.y = np.sin(steering_angle)
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0

        self.marker_pub.publish(marker)

class DisparityExtender:
    """ Implements the disparity extender algorithm
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.fast_speed = 0.75
        self.medium_speed = 0.5
        self.slow_speed = 0.25

        self.actual_speed = 0

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.actual_speed_pub = rospy.Publisher("/actual_speed_topic", Float32, queue_size = 1000)
        self.steering_angle_pub = rospy.Publisher("/steering_angle_topic", Float32, queue_size = 1000)
        self.desired_dist_pub = rospy.Publisher("/desired_dist_topic", Float32, queue_size = 1000)
        self.desired_speed_pub = rospy.Publisher("/desired_speed_topic", Float32, queue_size = 1000)
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 1000)

    def getRange(self, scan_msg, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view

        angle_degrees = angle * 180 / np.pi
        position = (angle_degrees + 45) / 270
        length = len(scan_msg.ranges)
        index = int(length * position)
        return scan_msg.ranges[index]        
       

    def lidar_callback(self, scan_msg):
        n = len(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(n)*scan_msg.angle_increment
        self.follow_the_gap(angles)
        

    def odom_callback(self, odom_msg):
        self.actual_speed = odom_msg.twist.twist.linear.x
        self.actual_speed_pub.publish(self.actual_speed)


def main(args):
    rospy.init_node("reactive", anonymous=True)
    follow_the_gap = FollowTheGap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
