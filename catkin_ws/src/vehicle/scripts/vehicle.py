#!/usr/bin/env python3
import sys
import math
import numpy as np
import skimage
import json
import yaml
from io import BytesIO
import traceback
from scipy.spatial.distance import pdist, squareform
from scipy.stats import norm
import cv2

#ROS Imports
import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, ColorRGBA, Int32, Float32
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseWithCovariance, Point, PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PointWrapper:
    def __init__(self, point):
        assert isinstance(point, Point)

        self.point = point
        self.min_speed = 0.5
        self.ttc = 0
        self.margin = 0.3
        self.velocity=0

def preprocess_map(map_msg):
        """
        Converts the map_msg.data array into a 2D numpy array.
        Current assumption: The map is centered at (0,0), which is also the robot's initial pose.
        WARNING: map_msg.info.origin represents the lower-right pixel of the map, not the starting position of the robot!
        Shouldn't we use /gt_pose messages to determine the inital pose of the robot, or can we assume it to always be at (0,0)?
        """

        # TODO: Calculate shortest path without downscaling the map
        map_width = int(np.ceil(map_msg.info.width))
        map_height = int(np.ceil(map_msg.info.height))
        map_res = map_msg.info.resolution
        
        map_data = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)).T
        map_data = skimage.measure.block_reduce(map_data, (1, 1), np.min)

        # Set unknown values to be occupied
        map_binary = (map_data > 20).astype(int)

        return map_binary

def get_rotation (orientation):
    global roll, pitch, yaw
    orientation_q = orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

def sensor_to_map_transform(p, rx, ry, rt, s):
    transformed = np.array([[math.cos(rt), -math.sin(rt), rx], [math.sin(rt), math.cos(rt), ry], [0, 0, 1]]) * np.array([p.x, p.y, 1]).transpose()
    # rospy.loginfo_throttle(1, "TRANSFORMED POINT: " + str(transformed))
    return Point(transformed[0][0], transformed[1][0], 1)

class Vehicle:
    
    map_file = None

    def __init__(self) -> None:
        pure_pursuit_topic = '/pure_pursuit_nav'
        reactive_topic = '/reactive_nav'
        drive_topic = '/nav'
        lidarscan_topic = '/scan'
        odom_topic = '/odom'
        path_topic = '/path'
        obstacles_topic = '/costmap_node/costmap/costmap_updates'

        self.pure_pursuit_drive = rospy.Subscriber(pure_pursuit_topic, AckermannDriveStamped, self.pure_pursuit_callback, queue_size=1)
        self.reactive_drive = rospy.Subscriber(reactive_topic, AckermannDriveStamped, self.reactive_callback, queue_size=1)
        self.pure_pursuit_start_index_sub = rospy.Subscriber('/start_pose_index', Int32, self.pure_pursuit_start_index_callback, queue_size=1)
        # self.pp_l_sub = rospy.Subscriber('/lookahead_dist', Float32, self.pure_pursuit_L_callback, queue_size=1)

        self.scans_pub = rospy.Publisher("/cartesian_scans", Marker, queue_size = 1)
        self.obstacle_marker_pub = rospy.Publisher("/obstacle", Marker, queue_size = 1000)
        self.closest_path_point_pub = rospy.Publisher("/closest_path_point", Marker, queue_size = 1000)

        self.obstacles_pub = rospy.Publisher('/obstacles_map', OccupancyGrid, queue_size=1)
        rospy.Timer(rospy.Duration(2.0/1.0), self.publish_obstacles_map)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        self.pp_drive_msg = AckermannDriveStamped()
        self.reactive_drive_msg = AckermannDriveStamped()
        self.path = Path()
        self.goal = None
        self.multilap = True

        self.obstacle_detected = False

        self.ttc = 1000
        self.velocity = 0.
        self.yaw = 0.
        self.steering_angle = 0.
        self.map = None
        self.map_matrix = np.matrix([[]])
        self.drive_mode = "PURE PURSUIT"
        rospy.Timer(rospy.Duration(1.0/2.0), self.log_drive_mode)

        self.pp_start_index = Int32(0)
        self.safe_path_pose = (0., 0.)

        self.obstacles_update_sub = rospy.Subscriber(obstacles_topic, OccupancyGridUpdate, self.obstacles_update_callback, queue_size=1)
        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pure_pursuit_L = 1.


        # Tuning parameters
        self.reactive_L = 0.2 # maybe ignora
        self.pure_pursuit_alpha = 0.75
        self.pure_pursuit_beta = 0.5
        self.map_lethal_thres = 65 # ignora
        self.reactive_front_fow = 125
        self.reactive_safety_factor = 2.5

    def convert_position_to_grid_cell(self, pos_x, pos_y):
        "Takes a position in meters and converts it to the corresponding grid cell in the OccupancyGrid"

        origin = self.map.info.origin.position
        index_x = int((pos_x - origin.x)/self.map.info.resolution)
        index_y = int((pos_y - origin.y)/self.map.info.resolution)

        return index_x, index_y
    
    def convert_grid_cell_to_position(self, index_x, index_y):
        "Takes a tuple (i,j) of indices on the grid and converts it to its coordinates in meters."
        
        origin = self.map.info.origin.position
        pos_x = (index_x )*self.map.info.resolution + origin.x
        pos_y = (index_y )*self.map.info.resolution + origin.y

        return pos_x, pos_y
    
    def publish_obstacles_map(self, event=None):
        if self.map is None:
            rospy.loginfo("WE HAVE NO MAP")
            return
        self.obstacles_pub.publish(self.map)

    def log_drive_mode(self, event=None):
        # rospy.loginfo("DRIVE MODE: " + self.drive_mode)
        rospy.loginfo("OBSTACLE DETECTED: " + str(self.obstacle_detected))

    def pure_pursuit_start_index_callback(self, index):
        self.pp_start_index = index

        # find goal point on path at lookahead_distance L
        dist = 0
        i = self.pp_start_index.data + 1
        poses = np.array([(p.pose.position.x, p.pose.position.y) for p in self.path.poses])
        ground_pos = self.path.poses[self.pp_start_index.data]
        ground_pose = (ground_pos.pose.position.x, ground_pos.pose.position.y)
        max_L = max(self.pure_pursuit_L, self.reactive_L)
        goal_index = i
        safety_index = i
        goal_found = False
        safety_found = False
        while i < len(poses):
            dist = np.linalg.norm(poses[i] - ground_pose)
            if dist > self.pure_pursuit_L and not goal_found:
                goal_found = True
                goal_index = i - 1
            if dist > self.reactive_L and not safety_found:
                safety_found = True
                safety_index = i - 1
            if dist > max_L:
                break
            i += 1
            # Start a new lap
            if self.multilap and i == len(poses):
                i = 0
        self.goal = poses[goal_index]    

        self.safe_path_pose = poses[safety_index]
        marker = self.visualize_obstacle(poses[safety_index][0], poses[safety_index][1], g=0.0, b=1.0)
        self.closest_path_point_pub.publish(marker)


    def pure_pursuit_L_callback(self, L):
        self.pure_pursuit_L = L.data * 0.5 + 0.8

    def pure_pursuit_callback(self, drive):
        self.pp_drive_msg = drive
        if not self.obstacle_detected:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.pp_drive_msg.header.stamp
            drive_msg.drive.steering_angle = self.pp_drive_msg.drive.steering_angle
            drive_msg.drive.speed = self.pp_drive_msg.drive.speed
            # drive_msg.drive.speed = 1.
            self.drive_mode = "PURE PURSUIT"
            self.drive_pub.publish(drive_msg)
            self.steering_angle = drive_msg.drive.steering_angle

    def reactive_callback(self, drive):
        self.reactive_drive_msg = drive
        if self.obstacle_detected:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.reactive_drive_msg.header.stamp
            drive_msg.drive.steering_angle = self.reactive_drive_msg.drive.steering_angle
            drive_msg.drive.speed = self.reactive_drive_msg.drive.speed
            self.steering_angle = drive_msg.drive.steering_angle

            self.drive_mode = 'FOLLOW THE GAP'
            self.drive_pub.publish(drive_msg)

    def map_callback(self, data):
        self.map = data
        self.map_stamp = data.header.stamp
        self.map_matrix = np.array(data.data).reshape((data.info.height, data.info.width))

    def obstacles_update_callback(self, data):
        if self.map is None:
            # wait for initial obstacles map
            self.map = rospy.wait_for_message('/costmap_node/costmap/costmap', OccupancyGrid)
            self.map_matrix = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))

        # cast occupancy update data to numpy matrix
        occupancy_window = np.array(data.data).reshape((data.height, data.width))
        x = data.y 
        y = data.x
        self.map_matrix[x:x+data.height, y:y+data.width] = occupancy_window
        self.map.data = self.map_matrix.flatten().tolist()

    def odom_callback(self, data):
        self.velocity = data.twist.twist.linear.x
        self.yaw = get_rotation(data.pose.pose.orientation)
        self.pose = data.pose
        self.map_frame = data.header.frame_id
        self.car_frame = data.child_frame_id
        self.car_stamp = data.header.stamp
        self.pure_pursuit_L = self.pure_pursuit_alpha * self.velocity + self.pure_pursuit_beta if self.velocity > sys.float_info.min else self.pure_pursuit_L

        if self.map is None or self.goal is None:
            return
        
        goal_in_map = self.convert_position_to_grid_cell(self.goal[1], self.goal[0])
        goal = self.convert_grid_cell_to_position(goal_in_map[1], goal_in_map[0])
        marker = self.visualize_obstacle(goal[0], goal[1])
        self.obstacle_marker_pub.publish(marker)
        
        if not self.obstacle_detected:
            
            # rospy.loginfo("OCCUPANCY VALUE FOR L: " + str(self.map_matrix[goal_in_map[0], goal_in_map[1]]))
            if self.map_matrix[goal_in_map[0], goal_in_map[1]] >= self.map_lethal_thres:
                # occupied!
                self.obstacle_detected = True


    def lidar_callback(self, data):
        if self.map is None or self.map.info.width == 0:
            return
        
        self.scan_msg = data
        self.sensor_frame = data.header.frame_id
        self.sensor_stamp = data.header.stamp
        
        # --- switch drive mode back to PP based on ttc threshold -------------------
        if self.obstacle_detected:
            ranges = np.array(data.ranges)
            front = ranges[self.myScanIndex(data, -self.reactive_front_fow):self.myScanIndex(data, self.reactive_front_fow)]
            # front = ranges
            goal_in_map = self.convert_position_to_grid_cell(self.safe_path_pose[1], self.safe_path_pose[0])
            if not np.any(front < self.pure_pursuit_L * self.reactive_safety_factor) and self.map_matrix[goal_in_map[0], goal_in_map[1]] < self.map_lethal_thres:
                self.obstacle_detected = False

    def visualize_obstacle(self,x,y,frame='map',r=0.0,g=1.0,b=0.0):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.id = 150
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.lifetime = rospy.Duration()
        return marker

    def myScanIndex(self, scan_msg, angle):
        # expects an angle in degrees and outputs the respective index in the scan_ranges array.
        # angle: between -135 to 135 degrees, where 0 degrees is directly to the front

        rad_angle = angle * math.pi / 180.0
        rad_angle = np.clip(rad_angle, -0.4189, 0.4189)

        if not (scan_msg.angle_min <= rad_angle <= scan_msg.angle_max):
            rospy.loginfo("ANGLE out of range: " + str(angle))
            return 540

        index = int((rad_angle - scan_msg.angle_min) / scan_msg.angle_increment)

        return index
    
    def path_callback(self, data):
        self.path = data


def main(args):
    rospy.init_node("vehicle_node", anonymous=True)
    rfgs = Vehicle()
    rospy.sleep(0.1)
    rfgs.map = rospy.wait_for_message('/costmap_node/costmap/costmap', OccupancyGrid)
    rfgs.map_matrix = np.array(rfgs.map.data).reshape((rfgs.map.info.height, rfgs.map.info.width))
    rospy.sleep(0.1)
    rospy.spin()  

if __name__ == '__main__':
    main(sys.argv)
