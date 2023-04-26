#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class Reactive:
    """ Implements a reactive algorithm like Follow The Gap
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
        self.left_dist_pub = rospy.Publisher("/left_dist_topic", Float32, queue_size = 1000)
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
        """ 
        """
        global DESIRED_DISTANCE_LEFT
        error = self.followLeft(scan_msg, DESIRED_DISTANCE_LEFT)
        time = scan_msg.header.stamp.secs + scan_msg.header.stamp.nsecs*1e-9
        #send error to pid_control
        self.pid_control(scan_msg, error, time)
    
    def odom_callback(self, odom_msg):
        self.actual_speed = odom_msg.twist.twist.linear.x
        self.actual_speed_pub.publish(self.actual_speed)
    


def main(args):
    rospy.init_node("reactive", anonymous=True)
    reactive = Reactive()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
