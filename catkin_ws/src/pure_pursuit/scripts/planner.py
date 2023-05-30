#!/usr/bin/env python3
import sys
import numpy as np
import copy
import skimage
import time
import networkx
import queue

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

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

        # Publishers

        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 1000)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 1000)
        self.path_pub = rospy.Publisher('/path', Path, latch=True, queue_size=10)


        # Controller parameters
        self.sparsity = 5
        self.scale = 1
        self.occupancy_treshhold = 10
        self.driving_speed = 0
        self.steering_angle = 0

    
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
        marker.lifetime = rospy.Duration(0.25)
        self.marker_pub.publish(marker)

    
    def preprocess_map(self, map_msg):
        """
        Converts the map_msg.data array into a 2D numpy array.
        Current assumption: The map is centered at (0,0)
        WARNING: map_msg.info.origin represents the lower-right pixel of the map, not the starting position of the robot!
        Shouldn't we use /gt_pose messages to determine the inital pose of the robot, or can we assume it to always be at (0,0)?
        """

        # TODO: Calculate shortest path without downscaling the map
        self.map_width = int(np.ceil(map_msg.info.width/self.scale))
        self.map_height = int(np.ceil(map_msg.info.height/self.scale))
        self.map_res = map_msg.info.resolution*self.scale
        
        map_data = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)).T
        map_data = skimage.measure.block_reduce(map_data, (self.scale,self.scale), np.min)

        # Set unknown values to be occupied
        map_data[map_data == - 1] = 100
        map_binary = (map_data < self.occupancy_treshhold).astype(int)

        # TODO: Maybe replace hardcoded initial position? /gt_pose?
        starting_point = self.convert_position_to_grid_cell(0, 0)
        print(starting_point)

        return map_binary, starting_point
    
    def calculate_finish_line(self, driveable_area, starting_point):
        # TODO: Currently, we assume the car is always facing straight forward at the start
        # Maybe adjust to calculate the finish line perpendicular to the inital orientation of the car?
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
        map_image = np.flip(np.flip(driveable_area, 1), 0)

        skimage.io.imsave('~/Documents/F1Tenth/maps/driveable_area.png', skimage.img_as_ubyte(255*map_image), check_contrast=False)

        radius = 12

        tic = time.time()
        eroded_map = skimage.morphology.binary_erosion(driveable_area, footprint = np.ones((2*radius,2*radius)))
        toc = time.time()

        rospy.loginfo(f"Time for binary erosion: {toc - tic}")

        map_image = np.flip(np.flip(eroded_map, 1), 0)

        skimage.io.imsave('~/Documents/F1Tenth/maps/driveable_area.png', skimage.img_as_ubyte(map_image), check_contrast=False)
        print("Saved map")

        return eroded_map
    
    def breadth_first_search(self, map_msg, safe_area, finish_line_start, finish_line_end):
        # Currently implemented with a 4-neighborhood

        x = finish_line_start[0]
        visited = []
        finish_line = [(x,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)]
        my_queue = queue.Queue()
        one_bef_finish_line = [(x-1,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)]
        for el in one_bef_finish_line:
            my_queue.put(el)
        previous_node = {(x-1,y): (x,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)}
        path = []

        visited = np.array(safe_area)
        visited.fill(False)
        for el in one_bef_finish_line:
            visited[el] = True

        i = 0
        while not my_queue.empty():
            i += 1
            x,y = my_queue.get()
            visited[x,y] = True

            if i % 100 == 0:
                print(i)
                pos_x, pos_y = self.convert_grid_cell_to_position(x,y)
                self.visualize_point(pos_x,pos_y)
            if (x,y) in finish_line:
                rospy.loginfo("Shortest path reached finish line!")
                return previous_node, (x,y)

            # Not 100% sure if this always works
            if (x+1,y) in finish_line:
                new_x = x - 1
                new_y = y
                if safe_area[new_x, new_y] == 1 and not visited[new_x, new_y]:
                    my_queue.put((new_x,new_y))
                    visited[new_x, new_y] = True
                    previous_node[(new_x,new_y)] = (x,y)
            else:
                for new_x, new_y in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
                    if safe_area[new_x, new_y] == 1 and not visited[new_x, new_y]:
                        my_queue.put((new_x,new_y))
                        visited[new_x, new_y] = True
                        previous_node[(new_x,new_y)] = (x,y)
    
    def shortest_path(self, map_msg, safe_area, finish_line_start, finish_line_end):
        "Use a simple breadth-first search"

        previous_node, start_point = self.breadth_first_search(map_msg, safe_area, finish_line_start, finish_line_end)
        x = finish_line_start[0]
        finish_line = [(x,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)]
        
        node = previous_node[start_point]
        i = 0
        while node not in finish_line:
            i += 1
            print(i)
            node = previous_node[node]

            if (i % self.sparsity) == 0:
                pos_x, pos_y = self.convert_grid_cell_to_position(node[0],node[1])
                cur_pose = PoseStamped()
                cur_pose.header = map_msg.header
                cur_pose.pose.position.x = pos_x
                cur_pose.pose.position.y = pos_y
                cur_pose.pose.position.z = 0
                self.path.poses.append(cur_pose)
        
        print(len(self.path.poses))

    
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
