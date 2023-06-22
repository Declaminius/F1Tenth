#!/usr/bin/env python3
import sys
import numpy as np
import json
from io import BytesIO
from scipy import spatial

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
        self.velocity=0

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
        self.L_factor = 2.
        self.speed_percentage = 1
        self.max_steering_angle = 0.4189

        # Multilap settings
        self.multilap = True
        self.laps = 0
        self.first_lap = True
        self.checkpoint_reached = False
        self.prev_lap_finish_time = rospy.get_time()

        self.path = Path() 
        self.actual_path = Path()
        self.actual_path.header.frame_id="map"

        self.ground_pose = Pose()
        
        self.odom_frame = ""
        self.odom_stamp = rospy.Time()
        self.velocity = 0.

        self.ttc_array = []
        self.ttc = 1000
        self.ttc_front = 1000

        self.speed = 0.
        self.steering_angle = 0.

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        self.speed_command_pub = rospy.Publisher('speed_command', Float32, queue_size=1)
        self.L_pub = rospy.Publisher('lookahead_dist', Float32, queue_size=1)
        self.marker_pub = rospy.Publisher("/marker_goal", Marker, queue_size = 1000)
        self.actual_path_pub = rospy.Publisher("/actual_path", Path, latch=True, queue_size=1)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length (in seconds)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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
    
    def myScanAngle(self, scan_msg, index):
        # returns angle in radians

        return scan_msg.angle_min + index * scan_msg.angle_increment
    

    def calculate_closest_waypoint(self, poses, ground_pose):
        # find closest path point to current pose

        prev_dist = np.linalg.norm(poses[self.closest_waypoint_index] - ground_pose)
        i = self.closest_waypoint_index
        while i < len(poses):
            dist = np.linalg.norm(poses[i] - ground_pose)
            if dist > prev_dist:
                break
            prev_dist = dist
            i += 1
            if i == self.checkpoint_index:
                rospy.loginfo("Checkpoint Charlie!")
                self.checkpoint_reached = True
            # Start a new lap
            if self.multilap and i == len(poses):
                if self.checkpoint_reached == True:
                    finish_time = rospy.get_time()
                    lap_time = finish_time - self.prev_lap_finish_time
                    self.prev_lap_finish_time = finish_time
                    rospy.loginfo(f"New lap. Previous lap time: {lap_time:.2f} seconds")
                    self.laps += 1
                    self.checkpoint_reached = False
                i = 1
        start_index = i - 1
        return start_index
    
    def calculate_goalpoint(self, poses, ground_pose):
        # find goal point on path at lookahead_distance L

        dist = 0
        i = self.closest_waypoint_index + 1
        while i < len(poses):
            dist = np.linalg.norm(poses[i] - ground_pose)
            if dist > self.L:
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
        self.ttc_array = (np.maximum(0,scan_ranges - 0.3)) / projected_speed_array
        self.ttc = np.amin(self.ttc_array[self.myScanIndex(scan_msg, np.degrees(self.steering_angle) - 30):self.myScanIndex(scan_msg, np.degrees(self.steering_angle) + 30)])
        # self.ttc = self.ttc_array[self.ttc_index]
        self.ttc_front = self.ttc_array[self.myScanIndex(scan_msg, np.degrees(self.steering_angle))]

    def odom_callback(self, odom_msg):
        """ Process each position update using the Pure Pursuit algorithm & publish an AckermannDriveStamped Message
        """
        self.odom_frame = odom_msg.header.frame_id
        self.odom_stamp = odom_msg.header.stamp
        self.velocity = odom_msg.twist.twist.linear.x

        if self.velocity < sys.float_info.min:
            self.closest_waypoint_index = None

        # self.L = 0.59259259259259 * self.velocity + 1.8518518518519 if self.velocity > sys.float_info.min else 2.
        # self.L = 0.81481481481482 * self.speed + 0.2962962962963 if self.speed > sys.float_info.min else 1.5
        self.L = 0.35 * self.velocity + 1
        self.L_pub.publish(Float32(self.L))
        # self.L *= self.L_factor

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
        goal = self.calculate_goalpoint(poses, ground_pose)

        # transform goal point to vehicle coordinate frame
        transform = self.tf_buffer.lookup_transform("base_link", self.path.header.frame_id, rospy.Time())
        goal_transformed = tf2_geometry_msgs.do_transform_point(PointWrapper(Point(goal[0], goal[1], 1)), transform).point

        self.goal_pose = goal_transformed
        self.path_error = goal_transformed.y

        curvature = 2 * goal_transformed.y / pow(self.L, 2)
        self.steering_angle = np.arctan(0.3302 * curvature)
        self.steering_angle = np.clip(self.steering_angle, -0.4189, 0.4189)

        self.speed = self.compute_speed()*self.speed_percentage
        self.speed_command_pub.publish(self.speed)

        self.publish_drive(self.speed, self.steering_angle)
        
        self.actual_path_pub.publish(self.actual_path)
        marker = visualize_point(goal[0], goal[1])
        self.marker_pub.publish(marker)
    
    def path_callback(self, path_msg):
        self.path = path_msg

    def path_flying_lap_callback(self, path_msg):
        self.path_flying_lap = path_msg

    def publish_drive(self, speed, angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        # drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


    def compute_speed(self):
        # choose front beam ttc if minimum ttc is not in fov [-45, 45]
        # if not -45 < np.degrees(self.myScanAngle(self.scan_msg, self.ttc_index)) < 45:
        #     # speed = 5 * ttc_array[self.myScanIndex(scan_msg, np.degrees(self.steering_angle))]
        #     speed = min(7, max(0.5, 7 * (1 - np.exp(-0.5*self.ttc_front))))
        # else:
        #     speed = min(7, max(0.5, 7 * (1 - np.exp(-0.75*self.ttc))))

        # alpha = np.arcsin(self.goal_pose.y / self.L)

        if self.goal_pose.x < 0:
            return 0.1

        alpha = self.steering_angle
        # rospy.loginfo_throttle(1, "steering angle: " + str(alpha))

        b = self.path_error
        dt = b * np.cos(alpha)

        projected_path_error = dt + self.L * np.sin(alpha)

        # ttc = self.ttc_array[self.myScanIndex(self.scan_msg, np.degrees(alpha))]
        ttc = self.ttc
        speed = min(7, max(0.25, 7 * (1 - np.exp(-0.75*ttc))))
        
        # clip speed by steering angle
        speed /= 10. * pow(self.steering_angle, 2) + 1

        
        # speed_multiplier = max(1, 1 / (100*projected_path_error**2)) if projected_path_error > sys.float_info.min else 1
        # speed *= speed_multiplier
        # rospy.loginfo(f"Speed multiplier (path error): {speed_multiplier}")

        # rospy.loginfo_throttle(1, "path error: " + str(projected_path_error))
        # rospy.loginfo_throttle(1, "velocity: " + str(self.velocity))
        return speed
    
    def append_pose(self, pos_x, pos_y):
        cur_pose = PoseStamped()
        cur_pose.header.stamp = rospy.Time.now()
        cur_pose.header.frame_id = self.odom_frame
        cur_pose.pose.position.x = pos_x
        cur_pose.pose.position.y = pos_y
        cur_pose.pose.position.z = 0
        self.actual_path.poses.append(cur_pose)

def main(args):
    rospy.init_node("pure_pursuit_node", anonymous=True)
    rfgs = PurePursuit()
    rospy.sleep(0.1)
    # rospack = rospkg.RosPack()
    # with open(f"{rospack.get_path('vehicle')}/path.bin", "rb") as path_file:
    #     rfgs.path_file = path_file
    #     buf = BytesIO(path_file.read())
    #     rfgs.path.deserialize(buf.getvalue())
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
