#!/usr/bin/env python3
import sys
import numpy as np
import copy
import skimage
import time
import networkx

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped



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
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 1000)

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

    def publish_drive_message(self):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = self.steering_angle
        drive_msg.drive.speed = self.driving_speed
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, scan_msg):
        pass

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
        starting_point = self.convert_position_to_grid_cell(0, 0)

        return map_binary, starting_point
    
    def calculate_finish_line(self, driveable_area, starting_point):
        x = starting_point[0]
        y = starting_point[1]
        left_end = y
        right_end = y

        while driveable_area[x,right_end] == 1:
            right_end += 1

        while driveable_area[x,left_end] == 1:
            left_end -= 1  

        rospy.loginfo(f"{right_end=}, {left_end=}")
        return (x, left_end), (x, right_end)

    
    def erode_map(self, map_binary):

        map_image = np.rot90(np.flip(map_binary, 0), 1)
        skimage.io.imsave('~/F1Tenth/maps/map_binary.png', skimage.img_as_ubyte(255*map_image), check_contrast=False)

        radius = 10

        tic = time.time()
        eroded_map = skimage.morphology.binary_erosion(map_binary, footprint = np.ones((2*radius,2*radius)))
        toc = time.time()

        rospy.loginfo(f"Time for binary erosion: {toc - tic}")

        map_image = np.rot90(np.flip(eroded_map, 0), 1)

        skimage.io.imsave('~/F1Tenth/maps/eroded_map.png', skimage.img_as_ubyte(map_image), check_contrast=False)
        print("Saved map")

        return eroded_map
    
    def shortest_path(self, safe_area, finish_line_start, finish_line_end):
        "Use a simple breadth-first search"
        x = finish_line_start[0]
        visited = []
        queue = [(x-1,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)]
        previous_node = {}
        path = []
        pos_x, pos_y = self.convert_grid_cell_to_position(finish_line_start[0], finish_line_start[1])
        self.visualize_point(pos_x, pos_y)

        i = 0
        while queue != []:
            i += 1
            x,y = queue.pop()
            if i % 10 == 0:
                pos_x, pos_y = self.convert_grid_cell_to_position(x,y)
                
            if x == finish_line_start[0] and finish_line_start[1] <= y <= finish_line_end[1]:
                path.append((x,y))
                break
    
            visited.append((x,y))
            for new_x, new_y in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                if (new_x, new_y) not in visited and safe_area[new_x, new_y] == 1:
                    queue.append((new_x,new_y))
                    visited.append((x,y))
                    previous_node[(new_x,new_y)] = (x,y)
        
        print(len(previous_node.keys()))
        node = previous_node[path[0]]
        while node not in finish_line:
            node = previous_node[node]
            path.append(node)

        return path
            

    
    def convert_position_to_grid_cell(self, pos_x, pos_y):
        "Takes a position in meters and converts it to the correspdonging grid cell in the OccupancyGrid"

        index_x = int(pos_x/self.map_res + self.map_height/2)
        index_y = int(pos_y/self.map_res + self.map_width/2)

        return index_x, index_y
    
    def convert_grid_cell_to_position(self, index_x, index_y):
        
        pos_x = (index_x - self.map_height/2)*self.map_res
        pos_y = (index_y - self.map_width/2)*self.map_res

        return pos_x, pos_y



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
        
        return np.where(self.map_binary == 1000, 1, 0)

    
    def map_callback(self, map_msg):
        rospy.loginfo(f"Map Header: {map_msg.header}")
        rospy.loginfo(f"Map info: {map_msg.info}")

        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.map_res = map_msg.info.resolution

        self.map_binary, starting_point = self.preprocess_map(map_msg)

        tic = time.time()
        driveable_area = self.fill4(starting_point[0], starting_point[1])
        toc = time.time()
        rospy.loginfo(f"Time for FloodFill: {toc - tic}")

        finish_line_start, finish_line_end = self.calculate_finish_line(driveable_area, starting_point)

        safe_area = self.erode_map(driveable_area)
        shortest_path = self.shortest_path(safe_area, finish_line_start, finish_line_end)

        for i in range(10):
            rospy.loginfo(shortest_path[i])


        



def main(args):
    rospy.init_node("planner", anonymous=True)
    follow_the_gap = PathGenerator()
    # rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
