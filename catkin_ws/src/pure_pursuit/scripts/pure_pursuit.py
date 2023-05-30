#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry, Path

# from skimage import io, morphology, img_as_ubyte TODO: Uncomment
from scipy import spatial


class PointWrapper:
    def __init__(self, point):
        assert isinstance(point, Point)

        self.point = point

class pure_pursuit:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        map_topic = '/map'
        odom_topic = '/odom'
        path_topic = '/path'

        self.path_pub = rospy.Subscriber(path_topic, Path, self.path_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        #self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1) # optional

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("/marker_goal", Marker, queue_size = 1000)

        self.ground_pose = Pose()
        self.L = 4
        self.odom_frame = ""
        self.odom_stamp = rospy.Time()

        self.path = Path() 

        self.prev_ground_path_index = 0

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def odom_callback(self, data):
        """ Process each position update using the Pure Pursuit algorithm & publish an AckermannDriveStamped Message
        """
        self.ground_pose = data.pose.pose
        self.odom_frame = data.header.frame_id
        self.odom_stamp = data.header.stamp

        poses_stamped = self.path.poses
        if len(poses_stamped) == 0:
            return
        
        poses =  np.array([(p.pose.position.x, p.pose.position.y) for p in poses_stamped])
        tree = spatial.KDTree(poses)

        # find closest path point to current pose
        ground_pose = (self.ground_pose.position.x, self.ground_pose.position.y)
        prev_dist = np.linalg.norm(poses[self.prev_ground_path_index] - ground_pose)
        for i in range (self.prev_ground_path_index + 1, len(poses)):
            dist = np.linalg.norm(poses[i] - ground_pose)
            if dist >= prev_dist:
                break
            prev_dist = dist
        start_index = i - 1
        start_pose = poses[start_index]
        self.prev_ground_path_index = start_index
        # start_index = tree.query((self.ground_pose.position.x, self.ground_pose.position.y))[1]
        # start_pose = poses[start_index

        # find goal point on path at lookahead_distance L
        dist = 0
        i = start_index + 1
        while i < len(poses):
            dist += np.linalg.norm(poses[i] - start_pose)
            if dist > self.L:
                break
            i += 1
                
        goal = poses[i - 1]
        
        # path_points_in_range = tree.query_ball_point(start, self.L, return_sorted=True)
        # goal = poses[path_points_in_range[len(path_points_in_range) - 1]]


        # transform goal point to vehicle coordinate frame
        transform = self.tf_buffer.lookup_transform("base_link", self.path.header.frame_id, rospy.Time())
        goal_transformed = tf2_geometry_msgs.do_transform_point(PointWrapper(Point(goal[0], goal[1], 1)), transform).point
        curvature = 2 * goal_transformed.y / pow(self.L, 2)
        R = 1 / curvature

        steering_angle = 1 / np.tan(curvature * 0.3302)
        # steering_angle = np.arctan(0.3302 * R)
        # steering_angle = np.arctan(1 / R)

        speed = 0.5
        
        rospy.loginfo_throttle(1, "goal_transformed.x: " + str(goal_transformed.y))

        self.publish_drive(speed, steering_angle)
        self.visualize_point(goal[0], goal[1])
    
    def path_callback(self, data):
        self.path = data
     

    def publish_drive(self, speed, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        # drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def visualize_point(self,x,y,frame='map',r=0.0,g=1.0,b=0.0):
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


def main(args):
    rospy.init_node("pure_pursuit_node", anonymous=True)
    rfgs = pure_pursuit()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
