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
from std_msgs.msg import Float32, Bool


class SlowCrash:
    """ Implement slowly crashing into the wall in front of you
    """
    def __init__(self):
        #Topics & Subs, Pubs
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 1)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.led_pub = rospy.Publisher('/led/blue', Bool, queue_size=1)

    def lidar_callback(self, scan_msg):

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = 0
        drive_msg.drive.speed = 0.2
        self.drive_pub.publish(drive_msg)

        # set blue led
        led_msg = Bool()
        led_msg.data = True
        self.led_pub.publish(led_msg)



def main(args):
    rospy.init_node("slow_crash", anonymous=True)
    slow_crash = SlowCrash()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
