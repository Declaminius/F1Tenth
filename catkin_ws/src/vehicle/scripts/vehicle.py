#!/usr/bin/env python3
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped


class Vehicle:
    def __init__(self) -> None:
        pure_pursuit_topic = '/pure_pursuit_nav'
        reactive_topic = '/reactive_nav'
        drive_topic = '/nav'
        lidarscan_topic = '/scan'
        odom_topic = '/odom'

        self.pure_pursuit_drive = rospy.Subscriber(pure_pursuit_topic, AckermannDriveStamped, self.pure_pursuit_callback, queue_size=1)
        self.reactive_drive = rospy.Subscriber(reactive_topic, AckermannDriveStamped, self.reactive_callback, queue_size=1)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        self.pp_drive_msg = AckermannDriveStamped()
        self.reactive_drive_msg = AckermannDriveStamped()

        self.ttc = 1000
        self.ttc_threshold = 0.5
        self.velocity = 0.
        self.steering_angle = 0.

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

    def pure_pursuit_callback(self, drive):
        self.pp_drive_msg = drive

    def reactive_callback(self, drive):
        self.reactive_drive_msg = drive

    def lidar_callback(self, data):
        self.scan_msg = data
        scan_ranges = np.array(data.ranges)
        scan_angles = np.linspace(data.angle_min, data.angle_max, len(scan_ranges))
        projected_speed_array = self.velocity * np.cos(scan_angles)
        projected_speed_array[projected_speed_array < 0.1] = 0.1
        self.ttc_array = (np.maximum(0,scan_ranges - 0.3)) / projected_speed_array
        self.ttc = np.amin(self.ttc_array[self.myScanIndex(data, math.degrees(self.steering_angle) - 30):self.myScanIndex(data, math.degrees(self.steering_angle) + 30)])
        # self.ttc = self.ttc_array[self.ttc_index]
        self.ttc_front = self.ttc_array[self.myScanIndex(data, math.degrees(self.steering_angle))]

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        # drive_msg.header.frame_id = "laser"
       
        self.drive_pub.publish(drive_msg)
        if self.ttc > 0.5:
            drive_msg.drive.steering_angle = self.pp_drive_msg.drive.steering_angle
            drive_msg.drive.speed = self.pp_drive_msg.drive.speed
        else:
            drive_msg.drive.steering_angle = self.reactive_drive_msg.drive.steering_angle
            drive_msg.drive.speed = self.reactive_drive_msg.drive.speed
        self.drive_pub.publish(drive_msg)
        
        self.steering_angle = drive_msg.drive.steering_angle

    def odom_callback(self, data):
        self.velocity = data.twist.twist.linear.x

    def myScanIndex(self, scan_msg, angle):
        # expects an angle in degrees and outputs the respective index in the scan_ranges array.
        # angle: between -135 to 135 degrees, where 0 degrees is directly to the front

        rad_angle = angle * math.pi / 180.0

        if not (scan_msg.angle_min <= rad_angle <= scan_msg.angle_max):
            rospy.loginfo("ANGLE out of range: " + str(angle))
            return 540

        index = int((rad_angle - scan_msg.angle_min) / scan_msg.angle_increment)

        return index

def main(args):
    rospy.init_node("vehicle_node", anonymous=True)
    rfgs = Vehicle()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
