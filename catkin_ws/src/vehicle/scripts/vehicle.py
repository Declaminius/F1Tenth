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
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
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
        odom_topic = '/odometry/filtered'
        ground_truth_topic = '/odom'
        path_topic = '/path2'

        self.pure_pursuit_drive = rospy.Subscriber(pure_pursuit_topic, AckermannDriveStamped, self.pure_pursuit_callback, queue_size=1)
        self.reactive_drive = rospy.Subscriber(reactive_topic, AckermannDriveStamped, self.reactive_callback, queue_size=1)
        self.scans_pub = rospy.Publisher("/cartesian_scans", Marker, queue_size = 1)
        self.obstacle_marker_pub = rospy.Publisher("/obstacle", Marker, queue_size = 1000)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        self.pp_drive_msg = AckermannDriveStamped()
        self.reactive_drive_msg = AckermannDriveStamped()

        self.ttc = 1000
        self.ttc_threshold = 0.5
        self.velocity = 0.
        self.pose = PoseWithCovariance()
        self.steering_angle = 0.
        self.map = OccupancyGrid()
        self.map_matrix = np.matrix([[]])
        self.likelihood_field = np.matrix([[]])
        self.drive_mode = "PURE PURSUIT"

        self.sensor_frame = 'laser'
        self.car_frame = 'base_link'
        self.map_frame = 'map'

        self.sensor_stamp = rospy.Time.now()
        self.car_stamp = rospy.Time.now()
        self.map_stamp = rospy.Time.now()

        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.ground_truth_sub = rospy.Subscriber(ground_truth_topic, Odometry, self.gt_callback, queue_size=1)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def pure_pursuit_callback(self, drive):
        self.pp_drive_msg = drive
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.pp_drive_msg.header.stamp
        drive_msg.drive.steering_angle = self.pp_drive_msg.drive.steering_angle
        drive_msg.drive.speed = self.pp_drive_msg.drive.speed
        self.drive_mode = "PURE PURSUIT"
        self.drive_pub.publish(drive_msg)
        self.steering_angle = drive_msg.drive.steering_angle

    def reactive_callback(self, drive):
        self.reactive_drive_msg = drive

    def map_callback(self, data):
        self.map = data
        self.map_stamp = data.header.stamp
        buff = BytesIO()
        data.serialize(buff)
        serialized_bytes = buff.getvalue()

        # if self.map_file is not None:
        rospack = rospkg.RosPack()
        with open(f"{rospack.get_path('vehicle')}/map.bin", "wb") as f:
            rospy.loginfo_throttle(1, "MAP: " + str(buff.getbuffer()))
            f.write(buff.getbuffer())

    def set_likelihood_field(self, lf):
        self.likelihood_field = lf

    def gt_callback(self, data):
        self.map_stamp = data.header.stamp

    def lidar_callback(self, data):
        if self.map is None or self.map.info.width == 0:
            return
        
        self.scan_msg = data
        self.sensor_frame = data.header.frame_id
        self.sensor_stamp = data.header.stamp
        # --- switch drive mode based on ttc threshold -------------------
        # scan_ranges = np.array(data.ranges)
        # scan_angles = np.linspace(data.angle_min, data.angle_max, len(scan_ranges))
        # projected_speed_array = self.velocity * np.cos(scan_angles)
        # projected_speed_array[projected_speed_array < 0.1] = 0.1
        # self.ttc_array = (np.maximum(0,scan_ranges - 0.3)) / projected_speed_array
        # angle = np.clip(self.steering_angle, -0.4189, 0.4189)
        # self.ttc = np.amin(self.ttc_array[self.myScanIndex(data, math.degrees(angle) - 10):self.myScanIndex(data, math.degrees(angle) + 10)])
        # # self.ttc = self.ttc_array[self.ttc_index]
        # self.ttc_front = self.ttc_array[self.myScanIndex(data, math.degrees(self.steering_angle))]
        # ----------------------------------------------------------------

        # transform laser scans from polar (m) to cartesian (pixel) coordinates in sensor frame
        polar_ranges = np.array(data.ranges)
        polar_to_cartesian = lambda rho, alpha: Point((rho) * math.cos(alpha), (rho) * math.sin(alpha), 1)
        cartesian_scans = np.array([polar_to_cartesian(r, data.angle_min + i * data.angle_increment) for i, r in enumerate(polar_ranges)])
        
        # filter out scans out of range
        cartesian_scans = cartesian_scans[(data.range_min <= polar_ranges) & (polar_ranges <= data.range_max)]

        colors = [ColorRGBA(0.0, 1.0, 0.0, 1.0)] * len(cartesian_scans)

        backup = cartesian_scans.copy()

        try:
            # transform scans from sensor frame to map frame
            sensor_to_car = self.tf_buffer.lookup_transform(self.map_frame, self.sensor_frame, data.header.stamp) 
            # map_to_car = self.tf_buffer.lookup_transform(self.car_frame, self.map_frame, self.sensor_stamp) 
            car_to_map = self.tf_buffer.lookup_transform(self.map_frame, self.car_frame, data.header.stamp) 
            transformed_cartesian_scans = [tf2_geometry_msgs.do_transform_point(PointStamped(data.header, p), sensor_to_car) for p in cartesian_scans]
            transformed_cartesian_scans = [tf2_geometry_msgs.do_transform_point(p, car_to_map) for p in transformed_cartesian_scans]
            transformed_cartesian_scans = [p.point  for p in transformed_cartesian_scans]
            
            # OLD: manual transformation ------------------
            # rx = self.pose.pose.position.x
            # ry = self.pose.pose.position.y
            # rt = get_rotation(self.pose.pose.orientation)
            # s = self.map.info.resolution
            # cartesian_scans = [sensor_to_map_transform(p, rx, ry, rt, s) for p in cartesian_scans]
            # cartesian_scans = [Point(p.x // self.map.info.resolution - 50, p.y // self.map.info.resolution - 50, 1) for p in cartesian_scans]
            # ------------------------------------------------

            # cond = lambda window: np.any(window > 0)
            lf_cond = lambda scan: self.likelihood_field[int(scan.x)][int(scan.y)] > 0.1

            # def window_around_scan(scan):
            #     center_row = int(scan.x)
            #     center_col = int(scan.y)
            #     window_size = 500
            #     start_row = max(center_row - window_size // 2, 0)
            #     end_row = min(center_row + window_size // 2 + 1, self.map.info.width)
            #     start_col = max(center_col - window_size // 2, 0)
            #     end_col = min(center_col + window_size // 2 + 1, self.map.info.height)
            #     # Adjust the boundaries to matrix bounds if necessary
            #     adjusted_start_row = max(start_row, 0)
            #     adjusted_end_row = min(end_row, self.map.info.width)
            #     adjusted_start_col = max(start_col, 0)
            #     adjusted_end_col = min(end_col, self.map.info.height)
            #     # Create the window
            #     return self.map_matrix[adjusted_start_row:adjusted_end_row, adjusted_start_col:adjusted_end_col]

            # scans_idxs = np.argwhere([cond(window_around_scan(scan)) for scan in transformed_cartesian_scans])
            scans_idxs = np.argwhere([lf_cond(scan) for scan in transformed_cartesian_scans])
            if len(scans_idxs) > 0:
                 # obstacle detected
                idx = scans_idxs[0]
                colors[idx].r = 1.0
                colors[idx].g = 0.0
                self.visualize_obstacle(transformed_cartesian_scans[idx].x, transformed_cartesian_scans[idx].y)
                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = self.reactive_drive_msg.header.stamp
                drive_msg.drive.steering_angle = self.reactive_drive_msg.drive.steering_angle
                drive_msg.drive.speed = self.reactive_drive_msg.drive.speed
                self.steering_angle = drive_msg.drive.steering_angle

                self.drive_mode = 'FOLLOW THE GAP'
                self.drive_pub.publish(drive_msg)


            # for idx, scan in enumerate(cartesian_scans):
            #     margin = 50
            #     # center = int(scan.x * self.map.info.width + scan.y)
            #     # window = np.array(self.map.data[center - margin * self.map.info.width - margin: center - margin * self.map.info.width + margin])
            #     # for i in range(1, margin):
            #         # window = np.append(window, np.array(self.map.data[center - (margin + i) * self.map.info.width - margin: center - (margin + i) * self.map.info.width + margin]))

                
            #     center_point = Point(scan.x , scan.y, 1)
            #     # center_points.append(center_point)
            #     # window = map_matrix[max(0, scan.point.x - margin)  * self.map.info.resolution - 50 : min(self.map.info.height - 1, scan.point.x + margin)  * self.map.info.resolution- 50][max(0, scan.point.y - margin)  * self.map.info.resolution  - 50: min(self.map.info.width - 1, scan.point.y + margin)  * self.map.info.resolution- 50]
            #     window = map_matrix[max(0, int(center_point.x) - margin // 2): min(self.map.info.height - 1, int(center_point.x) + margin // 2)][max(0, int(center_point.y) - margin // 2): min(self.map.info.width - 1, int(center_point.y) + margin // 2)]

            #     # if map_matrix[int(scan.point.x)][int(scan.point.y)] > 0:
            #     # if np.count_nonzero(window) > 0.1 * math.pow(margin*2, 2):
            #     dist_threshold = np.linalg.norm(np.array([scan.x, scan.y]) - np.array([self.pose.pose.position.x, self.pose.pose.position.y]))
            #     if window.any() and dist_threshold < 5:
            #         # obstacle detected
            #         colors[idx].r = 1.0
            #         colors[idx].g = 0.0
            #         self.visualize_obstacle(scan.x, scan.y)
            #         drive_msg = AckermannDriveStamped()
            #         drive_msg.header.stamp = self.reactive_drive_msg.header.stamp
            #         drive_msg.drive.steering_angle = self.reactive_drive_msg.drive.steering_angle
            #         drive_msg.drive.speed = self.reactive_drive_msg.drive.speed
            #         self.steering_angle = drive_msg.drive.steering_angle

            #         self.drive_mode = 'FOLLOW THE GAP'
            #         self.drive_pub.publish(drive_msg)
            #         break


                # if map_matrix[int(scan.point.x)][int(scan.point.y)] != 0:
                #     map_viz = np.append(map_viz, scan.point)

            
            # colors = [ColorRGBA(0.0, 1.0, 0.0, 1.0)] * len(map_viz)
            # self.visualize_cartesian_scans(map_viz, colors)

            # colors = [ColorRGBA(0.0, 1.0, 0.0, 1.0)] * len(center_points)
            # self.visualize_cartesian_scans(center_points, colors)

            cartesian_scans = transformed_cartesian_scans

        except Exception as e:
            ok = True
            cartesian_scans = backup
            # traceback.print_exc()
            # rospy.loginfo_throttle(1, "EXCEPTION: " + str(e))

        self.visualize_cartesian_scans(cartesian_scans, colors, data.header.stamp)

        # if self.ttc > 2.:
        #     drive_msg.drive.steering_angle = self.pp_drive_msg.drive.steering_angle
        #     drive_msg.drive.speed = self.pp_drive_msg.drive.speed
        #     drive_mode = "PURE PURSUIT"
        # else:
        #     drive_msg.drive.steering_angle = self.reactive_drive_msg.drive.steering_angle
        #     drive_msg.drive.speed = self.reactive_drive_msg.drive.speed
        #     drive_mode = 'FOLLOW THE GAP'

        rospy.loginfo_throttle(1, self.drive_mode)
        

    def visualize_cartesian_scans(self, scans, colors, stamp):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.
        marker.colors = colors
        marker.pose.orientation.w = 1
        marker.pose.position.x = -0.5
        marker.pose.position.y = -0.5
        marker.pose.position.z = 0
        marker.lifetime = rospy.Duration(0.1)

        marker.points = scans
        # rospy.loginfo_throttle(1, marker.points)
        self.scans_pub.publish(marker)

    def visualize_obstacle(self,x,y,frame='map',r=0.0,g=1.0,b=0.0):
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
        self.obstacle_marker_pub.publish(marker)

    def odom_callback(self, data):
        self.velocity = data.twist.twist.linear.x
        self.pose = data.pose
        self.map_frame = data.header.frame_id
        self.car_frame = data.child_frame_id
        self.car_stamp = data.header.stamp

    def compute_likelihood_field(self):
        map = self.map_matrix
        width = self.map.info.width
        height = self.map.info.height
        scale = self.map.info.resolution
        try:
            self.likelihood_field = np.zeros((width, height))
            map = map.astype(np.uint8)
            dists = cv2.distanceTransform(map, cv2.DIST_L2, 5)
            dists = dists / scale
            distribution = norm(0.0, 0.6)
            for i in range (0, height):
                for j in range (0, width):
                    prob = distribution.pdf(dists[i, j])
                    self.likelihood_field[i, j] = prob
            
        except:
            traceback.print_exc()

    def myScanIndex(self, scan_msg, angle):
        # expects an angle in degrees and outputs the respective index in the scan_ranges array.
        # angle: between -135 to 135 degrees, where 0 degrees is directly to the front

        rad_angle = angle * math.pi / 180.0

        if not (scan_msg.angle_min <= rad_angle <= scan_msg.angle_max):
            rospy.loginfo("ANGLE out of range: " + str(angle))
            return 540

        index = int((rad_angle - scan_msg.angle_min) / scan_msg.angle_increment)

        return index
    
    def path_callback(self, data):
        # y = yaml.load(str(data))
        # json_object = json.dumps(y, indent=4)
        # self.path_file.write(json_object)
        
        buff = BytesIO()
        data.serialize(buff)
        # serialized_bytes = buff.getvalue()

        rospack = rospkg.RosPack()
        with open(f"{rospack.get_path('vehicle')}/path.bin", "wb") as f:
            f.write(buff.getbuffer())

def main(args):
    rospy.init_node("vehicle_node", anonymous=True)
    rfgs = Vehicle()
    rospy.sleep(0.1)
    rospack = rospkg.RosPack()
    with open(f"{rospack.get_path('vehicle')}/map.bin", "rb") as f:
        buf = BytesIO(f.read())
        rfgs.map.deserialize(buf.getvalue())
        rfgs.map_matrix = np.matrix(preprocess_map(rfgs.map))
        rfgs.compute_likelihood_field()
        rospy.sleep(0.1)
    rospy.spin()  

    # with open(f"{rospack.get_path('vehicle')}/src/map.bin", "wb") as f:
        # rfgs.map_file = f
    rospy.spin()  

if __name__ == '__main__':
    main(sys.argv)
