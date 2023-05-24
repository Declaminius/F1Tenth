#!/usr/bin/env python3
import sys
import numpy as np
import copy

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, OccupancyGrid



class PathGenerator:
    """ Creating a path for the car to drive using only data from the /map topic.
    """

    def __init__(self):
        # Subscribers

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

        # Publishers

        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 1000)
        self.driveable_area_pub = rospy.Publisher("/driveable_area", OccupancyGrid, queue_size = 1000)

        # Racecar state
        self.velocity = 0

        # Controller parameters
        self.driving_speed = 0
        self.steering_angle = 0
    
    def get_scan_angle(self, scan_msg, index):
        # converts index to angle in radians

        return scan_msg.angle_min + index * scan_msg.angle_increment
    
    def get_scan_index(self, scan_msg, angle):
        # expects an angle in degrees and outputs the respective index in the scan_ranges array.
        # angle: between -135 to 135 degrees, where 0 degrees is directly to the front

        rad_angle = angle * np.pi / 180.0

        if not (scan_msg.angle_min <= rad_angle <= scan_msg.angle_max):
            rospy.loginfo("ANGLE out of range")
            return 100.

        index = int((rad_angle - scan_msg.angle_min) / scan_msg.angle_increment)

        return index

    def get_scan_range(self, scan_msg, angle):
        # scan_msg: single message from topic /scan
        # angle: between -135 to 135 degrees, where 0 degrees is directly to the front
        # Outputs length in meters to object with angle in lidar scan field of view

        index = self.get_scan_index(scan_msg, angle)

        if scan_msg.ranges[index] == np.inf or scan_msg.ranges[index] == np.nan: 
            rospy.loginfo("No LIDAR data for angle")
            return 100.

        return scan_msg.ranges[index]
    
    def generate_path(self):
        pass

    def publish_drive_message(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = self.steering_angle
        drive_msg.drive.speed = self.driving_speed
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, scan_msg):
        self.generate_path()

    def odom_callback(self, odom_msg):
        self.velocity = odom_msg.twist.twist.linear.x
    
    def preprocess_map(self, map_msg):
        """
        Converts the map_msg.data array into a 2D numpy array.
        Current assumption: The map is centered at (0,0)
        WARNING: map_msg.info.origin represents the lower-left pixel of the map, not the starting position of the robot!
        Shouldn't we use /gt_pose messages to determine the inital pose of the robot, or can we assume it to always be at (0,0)?
        """

        map_width = map_msg.info.width
        map_height = map_msg.info.height
        map_resolution = map_msg.info.resolution
        
        map_binary = np.array(map_msg.data).reshape((map_height, map_width))

        # TODO: Maybe replace hardcoded initial position?
        starting_point = self.convert_position_to_grid_cell(map_msg, 0, 0)

        return map_binary, starting_point
    
    def convert_position_to_grid_cell(self, map_msg, x, y):
        "Takes a position in meters and converts it to the correspdonging grid cell in the OccupancyGrid"

        index_x = int(x/self.map_res + self.map_height/2)
        index_y = int(y/self.map_res + self.map_width/2)

        return index_x, index_y

    
    def map_callback(self, map_msg):
        rospy.loginfo(f"Map Header: {map_msg.header}")
        rospy.loginfo(f"Map info: {map_msg.info}")

        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.map_res = map_msg.info.resolution


        # I think map_binary is a misnomer since the values of this array are probabilities and definitely not binary!
        self.map_binary, starting_point = self.preprocess_map(map_msg)

        self.driveable_area = self.fill4(starting_point[0], starting_point[1])
        driveable_area_msg = copy.deepcopy(map_msg)
        driveable_area_msg.data = self.driveable_area.flatten()

        print("Starting to publish")
        self.driveable_area_pub.publish(driveable_area_msg)
        print("Finished publishing")

    def fill4(self, x, y, occupancy_treshhold = 10):
        "Source: https://wikipedia.org/wiki/Floodfill"

        stack = []
        stack.append((x, y))
        while stack != []:
            (x, y) = stack.pop()
            if self.map_binary[x,y] != -1 and self.map_binary[x,y] <= occupancy_treshhold:
                self.map_binary[x,y] = 1000
                if y + 1 < self.map_width:
                    stack.append((x, y + 1))
                if y - 1 >= 0:
                    stack.append((x, y - 1))
                if x + 1 < self.map_height:
                    stack.append((x + 1, y))
                if x - 1 >= 0:
                    stack.append((x - 1, y))
        
        return np.where(self.map_binary == 1000, 0, 100)

        



def main(args):
    rospy.init_node("planner", anonymous=True)
    follow_the_gap = PathGenerator()
    # rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
