#!/usr/bin/env python3
import sys
import numpy as np
import json
from io import BytesIO
from scipy import spatial
import math

#ROS Imports
import rospy
import rospkg
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32, Float32

#Local imports
from rviz_functions import visualize_point


class PointWrapper:
    def __init__(self, point):
        assert isinstance(point, Point)

        self.point = point
        self.min_speed = 0.5
        self.ttc = 0
        self.margin = 0.3
        self.velocity = 0

class PurePursuit:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/pure_pursuit_nav'
        map_topic = '/map'
        odom_topic = '/odom'
        path_topic = '/path'
        path_flying_lap_topic = '/path_flying_lap'

        # Tuneable parameters
        self.max_speed = 7
        self.max_decel = 8.26
        self.max_steering_angle = 0.4189
        self.wheelbase = 0.3302
        self.speed_percentage = 1
        self.speed_reduction_curvature = 3
        # self.speed_reduction_steering_angle = 10
        self.speed_reduction_path_error = 0.

        self.lookahead_speed_factor = 0.25
        self.lookahead_dist_min = 0.5

        self.n_log = 50
        self.multilap = True

        # Initialization parameters
        self.steering_angle = 0
        self.velocity = 0.

        self.timestamp = 0
        self.laps = 0
        self.first_lap = True
        self.checkpoint_reached = False
        self.prev_lap_finish_time = rospy.get_time()
        self.path = Path()

        # ROS Transformations
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS Publishers
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.speed_command_pub = rospy.Publisher('speed_command', Float32, queue_size=1)
        self.speed_curvature_pub = rospy.Publisher('speed_curvature', Float32, queue_size=1)
        self.speed_path_error_pub = rospy.Publisher('speed_path_error', Float32, queue_size=1)
        self.lookahead_dist_pub = rospy.Publisher('lookahead_dist', Float32, queue_size=1)
        self.marker_pub = rospy.Publisher("/marker_goal", Marker, queue_size = 1000)

        # ROS Subscribers
        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback, queue_size=1)
        self.path_flying_lap_sub = rospy.Subscriber(path_flying_lap_topic, Path, self.path_flying_lap_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)


    
    def myScanIndex(self, scan_msg, angle):
        # expects an angle in degrees and outputs the respective index in the scan_ranges array.
        # angle: between -135 to 135 degrees, where 0 degrees is directly to the front

        rad_angle = angle * np.pi / 180.0

        if not (scan_msg.angle_min <= rad_angle <= scan_msg.angle_max):
            # rospy.loginfo("ANGLE out of range: " + str(angle))
            return 540

        index = int((rad_angle - scan_msg.angle_min) / scan_msg.angle_increment)

        return index
    
    def check_multilap(self, poses, pose_index):
        # Reset the pose index to the start of the array, once a lap has been completed

        if pose_index == self.checkpoint_index:
            if self.checkpoint_reached == False:
                rospy.loginfo("Checkpoint Charlie!")
                self.checkpoint_reached = True
        # Start a new lap
        if pose_index == len(poses):
            if self.checkpoint_reached == True:
                finish_time = rospy.get_time()
                lap_time = finish_time - self.prev_lap_finish_time
                self.prev_lap_finish_time = finish_time
                rospy.loginfo(f"New lap. Previous lap time: {lap_time:.2f} seconds")
                self.laps += 1
                self.checkpoint_reached = False
            pose_index = 1
        
        return pose_index

    def calculate_closest_waypoint(self, poses, ground_pose):
        # find closest path point to current pose

        prev_dist = np.linalg.norm(poses[self.closest_waypoint_index] - ground_pose)
        pose_index = self.closest_waypoint_index
        while pose_index < len(poses):
            dist = np.linalg.norm(poses[pose_index] - ground_pose)
            if dist > prev_dist:
                break
            prev_dist = dist
            pose_index += 1
            if self.multilap:
                pose_index = self.check_multilap(poses, pose_index)

        start_index = pose_index - 1
        return start_index
    
    def calculate_goalpoint(self, poses, ground_pose, lookahead_dist):
        # find goal point on path at lookahead_distance L

        dist = 0
        i = self.closest_waypoint_index + 1
        while i < len(poses):
            dist = np.linalg.norm(poses[i] - ground_pose)
            if dist > lookahead_dist:
                break
            i += 1
            # Start a new lap
            if self.multilap and i == len(poses):
                self.first_lap = False
                i = 1
        return poses[i - 1]

    def lidar_callback(self, scan_msg):
        scan_ranges = np.array(scan_msg.ranges)
        scan_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_ranges))
        projected_speed_array = self.velocity * np.cos(scan_angles)
        projected_speed_array[projected_speed_array < 0.1] = 0.1

        self.min_dist = np.min(scan_ranges)
        self.front_dist = scan_ranges[self.myScanIndex(scan_msg, 0)]
        self.ttc_array = (np.maximum(0,scan_ranges - 0.3)) / projected_speed_array
        self.ttc = np.amin(self.ttc_array[self.myScanIndex(scan_msg, np.degrees(self.steering_angle) - 30):self.myScanIndex(scan_msg, np.degrees(self.steering_angle) + 30)])
        self.ttc = max(0.1, self.ttc)
        self.ttc_front = self.ttc_array[self.myScanIndex(scan_msg, np.degrees(self.steering_angle))]

    def odom_callback(self, odom_msg):
        """ Process each position update using the Pure Pursuit algorithm & publish an AckermannDriveStamped Message
        """
        self.velocity = odom_msg.twist.twist.linear.x

        if self.velocity < sys.float_info.min:
            self.closest_waypoint_index = None

        self.lookahead_dist = self.lookahead_speed_factor * self.velocity + self.lookahead_dist_min
        self.lookahead_dist_pub.publish(Float32(self.lookahead_dist))

        self.lookahead_dist_brake = 1.4*self.velocity/self.max_decel + 1

        if self.first_lap:
            poses_stamped = self.path.poses
        else:
            poses_stamped = self.path_flying_lap.poses
        if len(poses_stamped) == 0:
            return
        
        # Setting a checkpoint to check whether the car has completed a real lap and not just driven backwards and forwards over the finish line!
        self.checkpoint_index = len(poses_stamped)//2

        poses =  np.array([(p.pose.position.x, p.pose.position.y) for p in poses_stamped])
        ground_pose = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y)

        if self.closest_waypoint_index is None:
            dists = np.linalg.norm(poses - ground_pose, axis=1)
            self.closest_waypoint_index = np.argmin(dists)
            # rospy.loginfo_throttle(1, "start index.x: " + str(self.closest_waypoint_index))

        self.closest_waypoint_index = self.calculate_closest_waypoint(poses, ground_pose)
        goal = self.calculate_goalpoint(poses, ground_pose, self.lookahead_dist)
        brake_goal = self.calculate_goalpoint(poses, ground_pose, self.lookahead_dist_brake)

        # transform goal point to vehicle coordinate frame
        transform = self.tf_buffer.lookup_transform("base_link", self.path.header.frame_id, rospy.Time())
        goal_transformed = tf2_geometry_msgs.do_transform_point(PointWrapper(Point(goal[0], goal[1], 1)), transform).point
        self.path_error = goal_transformed.y

        brake_goal_transformed = tf2_geometry_msgs.do_transform_point(PointWrapper(Point(brake_goal[0], brake_goal[1], 1)), transform).point


        self.curvature = 2 * goal_transformed.y / self.lookahead_dist**2
        self.curvature_brake = 2 * brake_goal_transformed.y / self.lookahead_dist_brake**2

        self.steering_angle = np.arctan(self.wheelbase * self.curvature)
        self.steering_angle = np.clip(self.steering_angle, -self.max_steering_angle, self.max_steering_angle)

        self.speed = self.compute_speed()*self.speed_percentage
        self.speed_command_pub.publish(self.speed)

        self.publish_drive(self.speed, self.steering_angle)
        
        marker = visualize_point(goal[0], goal[1])
        self.marker_pub.publish(marker)
    
    def path_callback(self, path_msg):
        self.path = path_msg

    def path_flying_lap_callback(self, path_msg):
        self.path_flying_lap = path_msg

    def publish_drive(self, speed, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)
    
    def compute_projected_path_error(self):
        alpha = self.steering_angle
        b = self.path_error
        dt = b * np.cos(alpha)
        projected_path_error = dt + self.lookahead_dist * np.sin(alpha)
        return projected_path_error


    def compute_speed(self):
        projected_path_error = self.compute_projected_path_error()

        speed_curvature = self.max_speed / (1 + self.speed_reduction_curvature * self.curvature_brake**2)
        # speed_steering_angle = self.max_speed / (1 + self.speed_reduction_steering_angle *self.steering_angle**2)
        speed_path_error = self.max_speed / (1 + self.speed_reduction_path_error * (self.path_error/self.ttc)**2)

        self.speed_curvature_pub.publish(speed_curvature)
        self.speed_path_error_pub.publish(speed_path_error)

        speed = min(speed_curvature, speed_path_error)
        return speed

def main(args):
    rospy.init_node("pure_pursuit")
    pure_pursuit = PurePursuit()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)