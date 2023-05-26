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
from nav_msgs.msg import Odometry, OccupancyGrid, Path
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
        self.path_pub = rospy.Publisher('/path', Path, latch=True, queue_size=10)

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
    
    def preprocess_map(self, map_msg, occupancy_treshhold = 10, scale = 3):
        """
        Converts the map_msg.data array into a 2D numpy array.
        Current assumption: The map is centered at (0,0)
        WARNING: map_msg.info.origin represents the lower-right pixel of the map, not the starting position of the robot!
        Shouldn't we use /gt_pose messages to determine the inital pose of the robot, or can we assume it to always be at (0,0)?
        """

        self.map_width = int(np.ceil(map_msg.info.width/scale))
        self.map_height = int(np.ceil(map_msg.info.height/scale))
        self.map_res = map_msg.info.resolution*scale
        
        map_data = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)).T
        map_data = skimage.measure.block_reduce(map_data, (scale,scale), np.min)
        # map_data = np.rot90(np.flip(map_data, 0), 1)
        # Set unknown values to be occupied
        map_data[map_data == - 1] = 100
        map_binary = (map_data < occupancy_treshhold).astype(int)

        # TODO: Maybe replace hardcoded initial position?
        starting_point = self.convert_position_to_grid_cell(0, 0)
        print(starting_point)

        return map_binary, starting_point
    
    def calculate_finish_line(self, driveable_area, starting_point):
        x = starting_point[0]
        y = starting_point[1]
        left_end = y
        right_end = y

        while driveable_area[x, right_end] == 1:
            right_end += 1

        while driveable_area[x, left_end] == 1:
            left_end -= 1  

        rospy.loginfo(f"{right_end=}, {left_end=}")
        return (x, left_end), (x, right_end)

    
    def erode_map(self, driveable_area):
        skimage.io.imsave('~/F1Tenth/maps/driveable_area.png', skimage.img_as_ubyte(255*driveable_area), check_contrast=False)

        radius = 3

        tic = time.time()
        eroded_map = skimage.morphology.binary_erosion(driveable_area, footprint = np.ones((2*radius,2*radius)))
        toc = time.time()

        rospy.loginfo(f"Time for binary erosion: {toc - tic}")

        map_image = np.flip(np.flip(eroded_map, 1), 0)

        skimage.io.imsave('~/F1Tenth/maps/eroded_map.png', skimage.img_as_ubyte(map_image), check_contrast=False)
        print("Saved map")

        return eroded_map
    
    def shortest_path(self, map_msg, safe_area, finish_line_start, finish_line_end):
        "Use a simple breadth-first search"
        x = finish_line_start[0]
        visited = []
        finish_line = [(x,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)]
        queue = [(x-1,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)]
        previous_node = {(x-1,y): (x,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)}
        path = []
        pos_x, pos_y = self.convert_grid_cell_to_position(finish_line_start[0], finish_line_start[1])
        self.visualize_point(pos_x, pos_y)

        i = 0
        while queue != []:
            i += 1
            x,y = queue.pop(0)
            visited.append((x,y))

            if i % 100 == 0:
                print(i)
                pos_x, pos_y = self.convert_grid_cell_to_position(x,y)
                self.visualize_point(pos_x,pos_y)
            if (x,y) in finish_line:
                path.append((x,y))
                print("Finish line")
                break

            if (x+1,y) in finish_line:
                new_x = x - 1
                new_y = y
                if ((new_x, new_y) not in visited) and safe_area[new_x, new_y] == 1:
                    queue.append((new_x,new_y))
                    visited.append((new_x,new_y))
                    previous_node[(new_x,new_y)] = (x,y)
            else:
                for new_x, new_y in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                    if ((new_x, new_y) not in visited) and safe_area[new_x, new_y] == 1:
                        queue.append((new_x,new_y))
                        visited.append((new_x,new_y))
                        previous_node[(new_x,new_y)] = (x,y)
        
        node = previous_node[path[0]]
        while node not in finish_line:
            node = previous_node[node]
            pos_x, pos_y = self.convert_grid_cell_to_position(node[0],node[1])
            cur_pose = PoseStamped()
            cur_pose.header = map_msg.header
            cur_pose.pose.position.x = pos_x
            cur_pose.pose.position.y = pos_y
            cur_pose.pose.position.z = 0
            self.path.poses.append(cur_pose)

            

    
    def convert_position_to_grid_cell(self, pos_x, pos_y):
        "Takes a position in meters and converts it to the correspdonging grid cell in the OccupancyGrid"

        index_x = int(pos_x/self.map_res + self.map_height/2)
        index_y = int(pos_y/self.map_res + self.map_width/2)

        return index_x, index_y
    
    def convert_grid_cell_to_position(self, index_x, index_y):
        
        pos_x = (index_x - self.map_height/2)*self.map_res
        pos_y = (index_y - self.map_width/2)*self.map_res

        return pos_x, pos_y



    def fill4(self, map_binary, x, y):
        """Source: https://wikipedia.org/wiki/Floodfill
        0 is occupied
        1 is free space
        2 is driveable area
        """

        stack = []
        stack.append((x, y))
        while stack != []:
            (x, y) = stack.pop()
            if map_binary[x,y] == 1:
                map_binary[x,y] = 2
                if y + 1 < self.map_height:
                    stack.append((x, y + 1))
                if y - 1 >= 0:
                    stack.append((x, y - 1))
                if x + 1 < self.map_width:
                    stack.append((x + 1, y))
                if x - 1 >= 0:
                    stack.append((x - 1, y))
        
        return map_binary == 2

    
    def map_callback(self, map_msg):
        rospy.loginfo(f"Map Header: {map_msg.header}")
        rospy.loginfo(f"Map info: {map_msg.info}")

        self.path = Path()

        map_binary, starting_point = self.preprocess_map(map_msg)
        rospy.loginfo(f"number of free grid cells: {np.sum(map_binary)}")

        tic = time.time()
        driveable_area = self.fill4(map_binary, starting_point[0], starting_point[1])
        toc = time.time()
        rospy.loginfo(f"Time for FloodFill: {toc - tic}")
        rospy.loginfo(f"number of driveable grid cells: {np.sum(driveable_area)}")

    
        finish_line_start, finish_line_end = self.calculate_finish_line(driveable_area, starting_point)

        
        safe_area = self.erode_map(driveable_area)

        rospy.loginfo(f"number of safe grid cells: {np.sum(safe_area)}")
        tic = time.time()
        shortest_path = self.shortest_path(map_msg, safe_area, finish_line_start, finish_line_end)
        toc = time.time()
        rospy.loginfo(f"Time for shortest path: {toc - tic}")

        
        self.path.header = map_msg.header
        self.path_pub.publish(self.path)



def main(args):
    rospy.init_node("planner", anonymous=True)
    follow_the_gap = PathGenerator()
    # rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
