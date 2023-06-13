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

        self.lookahead_dist = 2
        self.margin = 0.3
        self.velocity = 0

        self.speed = 0
        self.steering_angle = 0
        self.ttc = 0
        self.ttc_index = 0

        self.min_speed = 0.5

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

        self.velocity_pub = rospy.Publisher("/velocity_topic", Float32, queue_size = 1000)
        self.steering_angle_pub = rospy.Publisher("/steering_angle_topic", Float32, queue_size = 1000)
        self.desired_speed_pub = rospy.Publisher("/desired_speed_topic", Float32, queue_size = 1000)
        self.drive_pub = rospy.Publisher("/reactive_nav", AckermannDriveStamped, queue_size = 1000)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 1000)
        self.gap_pub = rospy.Publisher("/gap_viz", LaserScan, queue_size = 1000)   
    
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

    def compute_speed(self, scan_msg, ttc_front):
        # choose front beam ttc if minimum ttc is not in fov [-45, 45]
        if not -45 < math.degrees(self.myScanAngle(scan_msg, self.ttc_index)) < 45:
            # speed = 5 * ttc_array[self.myScanIndex(scan_msg, math.degrees(self.steering_angle))]
            speed = min(7, max(0.5, 7 * (1 - math.exp(-0.5*ttc_front))))
        else:
            speed = min(7, max(0.5, 7 * (1 - math.exp(-0.75*self.ttc))))
        
        # clip speed by steering angle
        clip = math.exp(8*abs(self.steering_angle) - 2)
        diff = speed - clip
        if diff > 0.5:
            speed = diff
        else:
            speed = 0.5
        
        return speed
    
    def compute_largest_gap(self, scan_msg, scan_ranges):
        ### Improvement: instead of simply maximizing the length of the gap;
        ### we also include the distance of the largest scan within the gap
        ### by maximizing gap_length*gap_distance

        max_gap_start = 0
        max_gap_length = 0
        max_gap_distance = 0
        gap_start = 0
        gap_length = 0
        gap_distance = 0
        scan_msg.intensities = np.array(scan_ranges)

        for (i, scan_range) in enumerate(scan_ranges):
            if scan_range > 0.:
                gap_length += 1
                if scan_range > gap_distance:
                    gap_distance = scan_range
            else:
                if gap_length*gap_distance > max_gap_length*max_gap_distance:
                    max_gap_length = gap_length
                    max_gap_start = gap_start
                    max_gap_distance = gap_distance
                scan_msg.intensities[gap_start:gap_start+gap_length] = 5
                gap_start = i + 1
                gap_length = 0
                gap_distance = 0
        if gap_start < len(scan_ranges):
            scan_msg.intensities[gap_start:gap_start+gap_length] = 5
        if gap_length > max_gap_length:
            max_gap_length = gap_length
            max_gap_start = gap_start
            max_gap_distance = gap_distance

        return max_gap_start, max_gap_length

    def compute_steering_angle(self, scan_msg, best_point):
        # some manual collision avoidance if we are too close to the wall
        right_dist = scan_msg.ranges[self.myScanIndex(scan_msg, -90)]
        left_dist = scan_msg.ranges[self.myScanIndex(scan_msg, 90)]
        front_dist = scan_msg.ranges[self.myScanIndex(scan_msg, 0)]
        if right_dist < self.margin and right_dist < left_dist:
            steering_angle = np.pi/4 # 45 degrees left
            # print(f"DANGER! Turning left because {right_dist=}")
        elif left_dist < self.margin:
            steering_angle = -np.pi/4 # 45 degrees right
            # print(f"DANGER! Turning right because {left_dist=}")
        elif front_dist < self.margin:
            if right_dist < left_dist:
                steering_angle = np.pi/3 # 60 degrees left
            else:
                steering_angle = -np.pi/3 # 60 degrees right
        else:
            steering_angle = self.myScanAngle(scan_msg, best_point)
        
        return steering_angle
    
    def choose_waypoint(self, scan_ranges, option = 1, num_points = 3):
        # Returns the waypoint to which our car turns based on three possible options
        # The waypoint is chosen within the largest gap
        # options: 0, 1, 2

        if option == 0:
            ### Option 0: Choose furthest point within the largest gap
            best_point = np.argmax(scan_ranges[self.max_gap_start: self.max_gap_end]) + self.max_gap_start
            return best_point
        elif option == 1:
            ### Option 1: Choose middle point of the largest gap
            best_point = self.max_gap_start + int(self.max_gap_length / 2)
            return best_point
        elif option == 2:
            ### Option 3: Use an evenly distributed number of candidate waypoints within the largest gap
            ### and choose the waypoint with is the farthest away
            candidate_points = np.linspace(self.max_gap_start, self.max_gap_end, num_points+2, dtype = int)[1:-1]

            best_point = None
            best_dist = 0

            for point in candidate_points:
                if scan_ranges[point] > best_dist:
                    best_dist = scan_ranges[point]
                    best_point = point
            return best_point

        else:
            print("Invalid option!")
    
    def publish_drive_message(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = self.steering_angle
        drive_msg.drive.speed = self.speed
        self.desired_speed_pub.publish(self.speed)
        self.drive_pub.publish(drive_msg)
        self.steering_angle_pub.publish(self.steering_angle*180/np.pi)


    def follow_the_gap(self, scan_msg):
        dist_threshold = 0.6
        safety_radius = 0.5
        ttc_threshold = 1.5

        min_angle = -100
        max_angle = 100
        min_angle_index = self.myScanIndex(scan_msg, min_angle)
        max_angle_index = self.myScanIndex(scan_msg, max_angle)

        scan_ranges = np.array(scan_msg.ranges)
        scan_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_ranges))

        self.right_dist = np.min(scan_ranges[:self.myScanIndex(scan_msg, 0)])
        self.left_dist = np.min(scan_ranges[self.myScanIndex(scan_msg, 0):])

        ### New version: Set all points with a projected ttc below a certain threshold to 0

        projected_speed_array = self.velocity * np.cos(scan_angles)
        projected_speed_array[projected_speed_array < 0.1] = 0.1
        ttc_array = (np.maximum(0,scan_ranges - self.margin)) / projected_speed_array
        self.ttc_index = np.argmin(ttc_array)
        self.ttc = ttc_array[self.ttc_index]
        ttc_front = ttc_array[self.myScanIndex(scan_msg, math.degrees(self.steering_angle))]
        scan_ranges[ttc_array < np.quantile(scan_ranges, 0.35)] = 0
        scan_ranges[scan_ranges < dist_threshold] = 0
        
        ### Old version: Put a safety bubble around the closest distance returned by the LiDar

        # self.min_dist_index = np.argmin(scan_ranges[min_angle_index:max_angle_index]) + min_angle_index
        # self.min_dist = scan_ranges[self.min_dist_index]

        # if self.min_dist - self.lookahead_dist > safety_radius:
        #     safety_angle = np.arcsin(safety_radius/self.min_dist)
        # else:
        #     safety_angle = 0.4

        # safety_left = self.myScanIndex(scan_msg, math.degrees(self.myScanAngle(scan_msg, self.min_dist_index) - safety_angle))
        # safety_right = self.myScanIndex(scan_msg, math.degrees(self.myScanAngle(scan_msg, self.min_dist_index) + safety_angle))
        # scan_ranges[safety_left: safety_right] = 0

        # min_dist_angle = self.myScanAngle(scan_msg, self.min_dist_index)
        # for i in range(len(scan_ranges)):
        #     if self.min_dist * (abs(self.myScanAngle(scan_msg, i) - self.myScanAngle(scan_msg, self.min_dist_index))) < safety_radius:
        #         scan_ranges[i] = 0.

        ### Put a safety bubble around the car
        
        scan_ranges[scan_ranges < dist_threshold] = 0

        ### Set backward ranges to zero to prevent the car turning around in sharp corners

        scan_ranges[:min_angle_index] = 0
        scan_ranges[max_angle_index:] = 0


        ### Compute the gap maximizing length*distance

        self.max_gap_start, self.max_gap_length = self.compute_largest_gap(scan_msg, scan_ranges)
        self.max_gap_end = self.max_gap_start + self.max_gap_length

        ### Choose the best point within the largest gap

        self.best_point = self.choose_waypoint(scan_ranges)
        self.best_distance = scan_ranges[self.best_point]

        ### Compute the steering angle based on the best point and our distance towards it
        ### In addition, perform emergency steering in case we are too close to the wall

        self.steering_angle = self.compute_steering_angle(scan_msg, self.best_point)
        self.steering_angle *= 2/self.best_distance

        ### Compute the desired driving speed based on our current time-to-collision and steering angle

        self.speed = self.compute_speed(scan_msg, ttc_front)

        ### Publish speed and steering angle

        self.publish_drive_message()

        ### Visualize the largest gap

        scan_msg.intensities[ttc_array < 1] = 1
        scan_msg.intensities[self.max_gap_start:self.max_gap_end] = 2
        self.gap_pub.publish(scan_msg)

        ### Visualize the waypoint we're driving towards

        best_angle = self.myScanAngle(scan_msg, self.best_point)
        # self.visualize_point(self.best_distance*np.cos(best_angle), self.best_distance*np.sin(best_angle), r = 0, g = 1, b = 0)

        # Visualize the car's speed and steering angle with an arrow

        self.marker_viz(self.speed, self.steering_angle)


    def lidar_callback(self, scan_msg):
        self.follow_the_gap(scan_msg)

    def odom_callback(self, odom_msg):
        self.velocity = odom_msg.twist.twist.linear.x
        self.velocity_pub.publish(self.velocity)


    def visualize_point(self,x,y,frame='laser',r=1.0,g=0.0,b=1.0):
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
        marker.scale.x = self.speed
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


def main(args):
    rospy.init_node("reactive", anonymous=True)
    follow_the_gap = FollowTheGap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)