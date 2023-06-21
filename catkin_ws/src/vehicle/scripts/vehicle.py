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
        odom_topic = '/odometry/filtered'
        ground_truth_topic = '/odom'
        path_topic = '/path2'
        costmap_topic = '/costmap_node/costmap/costmap'
        obstacles_topic = '/costmap_node/costmap/costmap_updates'

        self.pure_pursuit_drive = rospy.Subscriber(pure_pursuit_topic, AckermannDriveStamped, self.pure_pursuit_callback, queue_size=1)
        self.reactive_drive = rospy.Subscriber(reactive_topic, AckermannDriveStamped, self.reactive_callback, queue_size=1)
        self.pure_pursuit_start_index_sub = rospy.Subscriber('/start_pose_index', Int32, self.pure_pursuit_start_index_callback, queue_size=1)
        self.pp_l_sub = rospy.Subscriber('/lookahead_dist', Float32, self.pure_pursuit_L_callback, queue_size=1)


        self.scans_pub = rospy.Publisher("/cartesian_scans", Marker, queue_size = 1)
        self.obstacle_marker_pub = rospy.Publisher("/obstacle", Marker, queue_size = 1000)
        self.gap_pub = rospy.Publisher("/received_scans", LaserScan, queue_size = 1000)   
        self.lf_pub = rospy.Publisher('/lf', OccupancyGrid, queue_size=1, latch=True)

        self.obstacles_pub = rospy.Publisher('/ocstacles_map', OccupancyGrid, queue_size=1)
        rospy.Timer(rospy.Duration(2.0/1.0), self.publish_obstacles_map)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        self.pp_drive_msg = AckermannDriveStamped()
        self.reactive_drive_msg = AckermannDriveStamped()
        self.path = Path()
        self.goal = None
        self.multilap = True

        self.obstacle_detected = False

        self.ttc = 1000
        self.ttc_threshold = 0.5
        self.velocity = 0.
        self.pose = PoseWithCovariance()
        self.steering_angle = 0.
        self.map = None
        self.map_matrix = np.matrix([[]])
        self.likelihood_field = np.matrix([[]])
        self.drive_mode = "PURE PURSUIT"
        rospy.Timer(rospy.Duration(1.0/2.0), self.log_drive_mode)

        self.pp_start_index = Int32(0)
        self.L = 3

        self.sensor_frame = 'laser'
        self.car_frame = 'base_link'
        self.map_frame = 'map'

        self.sensor_stamp = rospy.Time.now()
        self.car_stamp = rospy.Time.now()
        self.map_stamp = rospy.Time.now()

        self.lf = OccupancyGrid()

        # self.costmap_sub = rospy.Subscriber(costmap_topic, OccupancyGrid, self.costmap_callback, queue_size=1)
        # self.obstacles_sub = rospy.Subscriber('/costmap_node/costmap/costmap', OccupancyGrid, self.obstacles_callback, queue_size=1)
        self.obstacles_update_sub = rospy.Subscriber(obstacles_topic, OccupancyGridUpdate, self.obstacles_update_callback, queue_size=1)

        self.path_sub = rospy.Subscriber(path_topic, Path, self.path_callback, queue_size=1)
        # self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.ground_truth_sub = rospy.Subscriber(ground_truth_topic, Odometry, self.gt_callback, queue_size=1)
        # self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def set_lf(self, lf):
        self.lf = lf
        self.lf_pub.publish(lf)

    def convert_position_to_grid_cell(self, pos_x, pos_y):
        "Takes a position in meters and converts it to the corresponding grid cell in the OccupancyGrid"

        origin = self.map.info.origin.position
        # index_x = int((pos_x - origin.x)/self.map.info.resolution + self.map.info.height/2)
        # index_y = int((pos_y - origin.y)/self.map.info.resolution + self.map.info.width/2)
        index_x = int((pos_x - origin.x)/self.map.info.resolution)
        index_y = int((pos_y - origin.y)/self.map.info.resolution)

        return index_x, index_y
    
    def convert_grid_cell_to_position(self, index_x, index_y):
        "Takes a tuple (i,j) of indices on the grid and converts it to its coordinates in meters."
        
        origin = self.map.info.origin.position
        # pos_x = (index_x - self.map.info.height/2)*self.map.info.resolution + origin.x
        # pos_y = (index_y - self.map.info.width/2)*self.map.info.resolution + origin.y
        pos_x = (index_x )*self.map.info.resolution + origin.x
        pos_y = (index_y )*self.map.info.resolution + origin.y

        return pos_x, pos_y
    
    def publish_obstacles_map(self, event=None):
        if self.map is None:
            rospy.loginfo("WE HAVE NO MAP")
            return
        # self.map.header.stamp = rospy.Time.now()
        # self.map.header.seq = self.map.header.seq + 1
        self.obstacles_pub.publish(self.map)

    def log_drive_mode(self, event=None):
        rospy.loginfo("DRIVE MODE: " + self.drive_mode)


    def pure_pursuit_start_index_callback(self, index):
        self.pp_start_index = index

    def pure_pursuit_L_callback(self, L):
        self.L = L.data * 2

    def pure_pursuit_callback(self, drive):
        self.pp_drive_msg = drive
        if not self.obstacle_detected:
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
        self.map_matrix = np.array(data.data).reshape((data.info.width, data.info.height)).T
        # buff = BytesIO()
        # data.serialize(buff)
        # serialized_bytes = buff.getvalue()

        # # if self.map_file is not None:
        # rospack = rospkg.RosPack()
        # with open(f"{rospack.get_path('vehicle')}/map.bin", "wb") as f:
        #     rospy.loginfo_throttle(1, "MAP: " + str(buff.getbuffer()))
        #     f.write(buff.getbuffer())

    def set_likelihood_field(self, lf):
        self.likelihood_field = lf

    def gt_callback(self, data):
        self.map_stamp = data.header.stamp

    def has_elements_in_circle(matrix, center, radius):
        # # Calculate the indices of all elements in the matrix
        # indices = np.indices(matrix.shape)
        
        # Calculate the Euclidean distances from the center to all elements
        distances = np.sqrt((matrix[0] - center[0])**2 + (matrix[1] - center[1])**2)
        
        # Check if any element is within the specified radius
        return np.any(distances <= radius)
    
    def obstacles_callback(self, data):
        self.map = data
        self.map_matrix = np.array(self.map.data).reshape((self.map.info.height, self.map.info.width))

    def obstacles_update_callback(self, data):
        if self.map is None:
            # wait for initial obstacles map
            self.map = rospy.wait_for_message('/costmap_node/costmap/costmap', OccupancyGrid)
            self.map_matrix = np.array(self.map.data).reshape((self.map.info.width, self.map.info.height))

        # cast occupancy update data to numpy matrix
        occupancy_window = np.array(data.data).reshape((data.height, data.width))
        self.map_matrix[data.x:data.x+data.height, data.y:data.y+data.width] = occupancy_window
        self.map.data = self.map_matrix.flatten().tolist()

        # x, y = self.convert_grid_cell_to_position(data.x, data.y)
        # if self.map_matrix[data.x, data.y] >= 65:
        #     self.visualize_obstacle(x, y)

        # self.publish_obstacles_map()

        # occupancy_window = np.mgrid(data.x:data.x+data.height, data.y:data.y+data.width)


    def lidar_callback(self, data):
        if self.map is None or self.map.info.width == 0:
            return
        
        self.gap_pub.publish(data)

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

        # colors = [ColorRGBA(0.0, 1.0, 0.0, 1.0)] * len(cartesian_scans)

        # backup = cartesian_scans.copy()

        # try:
        #     # transform scans from sensor frame to map frame
        #     sensor_to_car = self.tf_buffer.lookup_transform(self.map_frame, self.sensor_frame, rospy.Time(0)) 
        #     # map_to_car = self.tf_buffer.lookup_transform(self.car_frame, self.map_frame, self.sensor_stamp) 
        #     car_to_map = self.tf_buffer.lookup_transform(self.map_frame, self.car_frame, rospy.Time(0)) 
        #     transformed_cartesian_scans = [tf2_geometry_msgs.do_transform_point(PointStamped(data.header, p), sensor_to_car) for p in cartesian_scans]
        #     # transformed_cartesian_scans = [tf2_geometry_msgs.do_transform_point(p, car_to_map) for p in transformed_cartesian_scans]
        #     transformed_cartesian_scans = [p.point  for p in transformed_cartesian_scans]
            
        #     # OLD: manual transformation ------------------
        #     # rx = self.pose.pose.position.x
        #     # ry = self.pose.pose.position.y
        #     # rt = get_rotation(self.pose.pose.orientation)
        #     # s = self.map.info.resolution
        #     # cartesian_scans = [sensor_to_map_transform(p, rx, ry, rt, s) for p in cartesian_scans]
        #     # cartesian_scans = [Point(p.x // self.map.info.resolution - 50, p.y // self.map.info.resolution - 50, 1) for p in cartesian_scans]
        #     # ------------------------------------------------

        #     # cond = lambda window: np.any(window > 0)
        #     def lf_cond(scan):
        #         cell_x, cell_y = self.convert_position_to_grid_cell(scan.x, scan.y)
        #         return self.likelihood_field[cell_x][cell_y] < 0.2
        #     rospy.loginfo_throttle(1, "MAX LIKELIHOOD FIELD VALUE: " + str(np.amax(self.likelihood_field)))

        #     scans_idxs = np.argwhere([lf_cond(scan) for scan in transformed_cartesian_scans])
        #     if len(scans_idxs) > 0:
        #          # obstacle detected
        #         idx = scans_idxs[0]
        #         colors[idx].r = 1.0
        #         colors[idx].g = 0.0
        #         self.visualize_obstacle(transformed_cartesian_scans[idx].x, transformed_cartesian_scans[idx].y)
        #         drive_msg = AckermannDriveStamped()
        #         drive_msg.header.stamp = self.reactive_drive_msg.header.stamp
        #         drive_msg.drive.steering_angle = self.reactive_drive_msg.drive.steering_angle
        #         drive_msg.drive.speed = self.reactive_drive_msg.drive.speed
        #         self.steering_angle = drive_msg.drive.steering_angle

        #         self.drive_mode = 'FOLLOW THE GAP'
        #         self.drive_pub.publish(drive_msg)

        #     cartesian_scans = transformed_cartesian_scans

        # except Exception as e:
        #     ok = True
        #     cartesian_scans = backup

        # self.visualize_cartesian_scans(cartesian_scans, colors, rospy.Time.now())

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
        marker.colors = colors[::100]
        marker.pose.orientation.w = 1
        marker.pose.position.x = -0.5
        marker.pose.position.y = -0.5
        marker.pose.position.z = 0
        marker.lifetime = rospy.Duration()

        marker.points = scans[::100]
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
        marker.lifetime = rospy.Duration()
        self.obstacle_marker_pub.publish(marker)

    def odom_callback(self, data):
        self.velocity = data.twist.twist.linear.x
        self.pose = data.pose
        self.map_frame = data.header.frame_id
        self.car_frame = data.child_frame_id
        self.car_stamp = data.header.stamp

        if self.map is None or self.goal is None:
            return

        goal_in_map = self.convert_position_to_grid_cell(self.goal[0], self.goal[1])
        goal = self.convert_grid_cell_to_position(goal_in_map[0], goal_in_map[1])
        self.visualize_obstacle(goal[0], goal[1])
        # rospy.loginfo("OCCUPANCY VALUE FOR L: " + str(self.map_matrix[goal_in_map[0], goal_in_map[1]]))
        if self.map_matrix[goal_in_map[0], goal_in_map[1]] >= 65:
            # occupied!
            # self.visualize_obstacle(self.goal[0], self.goal[1])
            self.obstacle_detected = True
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.reactive_drive_msg.header.stamp
            drive_msg.drive.steering_angle = self.reactive_drive_msg.drive.steering_angle
            drive_msg.drive.speed = self.reactive_drive_msg.drive.speed
            self.steering_angle = drive_msg.drive.steering_angle

            self.drive_mode = 'FOLLOW THE GAP'
            self.drive_pub.publish(drive_msg)
        else:
            self.obstacle_detected = False

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
            distribution = norm(0.0, 0.9)
            for i in range (0, height):
                for j in range (0, width):
                    prob = distribution.pdf(dists[i, j])
                    self.likelihood_field[i, j] = prob

            lf = OccupancyGrid()
            lf.info = MapMetaData()
            lf.info = self.map.info
            normalized_lf = self.likelihood_field.copy()
            normalized_lf *= 100. / np.max(normalized_lf) 
            lf.data = normalized_lf.flatten().to_list()

            return lf
            
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
        self.path = data
        # find goal point on path at lookahead_distance L
        dist = 0
        i = self.pp_start_index.data + 1
        poses = np.array([(p.pose.position.x, p.pose.position.y) for p in self.path.poses])
        ground_pos = self.path.poses[self.pp_start_index.data]
        ground_pose = (ground_pos.pose.position.x, ground_pos.pose.position.y)
        while i < len(poses):
            dist = np.linalg.norm(poses[i] - ground_pose)
            if dist > self.L:
                break
            i += 1
            # Start a new lap
            if self.multilap and i == len(poses):
                i = 0
        self.goal = poses[i - 1]    
    
        # buff = BytesIO()
        # data.serialize(buff)
        # # serialized_bytes = buff.getvalue()

        # rospack = rospkg.RosPack()
        # with open(f"{rospack.get_path('vehicle')}/path.bin", "wb") as f:
        #     f.write(buff.getbuffer())

def main(args):
    rospy.init_node("vehicle_node", anonymous=True)
    rfgs = Vehicle()
    rospy.sleep(0.1)
    rfgs.map = rospy.wait_for_message('/costmap_node/costmap/costmap', OccupancyGrid)
    rfgs.map_matrix = np.array(rfgs.map.data).reshape((rfgs.map.info.height, rfgs.map.info.width))
    rospy.sleep(0.1)
    rospack = rospkg.RosPack()
    # with open(f"{rospack.get_path('vehicle')}/map.bin", "rb") as f:
    #     buf = BytesIO(f.read())
    #     rfgs.map.deserialize(buf.getvalue())
    #     rfgs.map_matrix = np.matrix(preprocess_map(rfgs.map))
    #     # rfgs.set_likelihood_field(rfgs.compute_likelihood_field())
    #     rospy.sleep(0.1)
    rospy.spin()  

    # with open(f"{rospack.get_path('vehicle')}/src/map.bin", "wb") as f:
        # rfgs.map_file = f

if __name__ == '__main__':
    main(sys.argv)
