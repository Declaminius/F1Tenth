#!/usr/bin/env python3
import sys
import os
import numpy as np
import skimage
import time
from collections import defaultdict
import heapq as heap
from functools import wraps

from bspline import approximate_b_spline_path
from rviz_functions import visualize_point


import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker



def timing(f):
    "A simple decorator for timing functions"
    @wraps(f)
    def wrap(*args, **kw):
        ts = time.time()
        result = f(*args, **kw)
        te = time.time()
        rospy.loginfo('func:%r took: %2.4f sec' % \
          (f.__name__, te-ts))
        return result
    return wrap



class PathGenerator:
    """ Creating a path for the car to drive using only data from the /map topic.
    """

    def __init__(self):
        # Subscribers

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

        # Publishers

        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 1000)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 1000)
        self.path_pub = rospy.Publisher('/path', Path, latch=True, queue_size=1)
        self.path_flying_lap_pub = rospy.Publisher('/path_flying_lap', Path, latch=True, queue_size=1)


        # Controller parameters
        self.sparsity = 5
        self.scale = 1 # by which factor to downscale the map resolution before performing the path generation
        self.safety_margin = 0.7 # in meters
        self.occupancy_treshhold = 10 # pixel below this treshold (in percent) we consider free space
    

    
    def preprocess_map(self, map_msg):
        """
        Converts the map_msg.data array into a 2D numpy array.
        Current assumption: The map is centered at (0,0), which is also the robot's initial pose.
        WARNING: map_msg.info.origin represents the lower-right pixel of the map, not the starting position of the robot!
        Shouldn't we use /gt_pose messages to determine the inital pose of the robot, or can we assume it to always be at (0,0)?
        """

        self.map_width = int(np.ceil(map_msg.info.width/self.scale))
        self.map_height = int(np.ceil(map_msg.info.height/self.scale))
        self.map_res = map_msg.info.resolution*self.scale
        
        map_data = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)).T
        map_data = skimage.measure.block_reduce(map_data, (self.scale,self.scale), np.min)

        # Set unknown values to be occupied
        map_data[map_data == - 1] = 100
        map_binary = (map_data < self.occupancy_treshhold).astype(int)

        # TODO: Maybe replace hardcoded initial position? /gt_pose?
        self.start_point = self.convert_position_to_grid_cell(0, 0)

        return map_binary
    
    def calculate_finish_line(self, driveable_area):
        # TODO: Currently, we assume the car is always facing straight forward at the start
        # Maybe adjust to calculate the finish line perpendicular to the inital orientation of the car?
        x = self.start_point[0]
        y = self.start_point[1]
        left_end = y
        right_end = y

        while driveable_area[x, right_end] == 1:
            right_end += 1

        while driveable_area[x, left_end] == 1:
            left_end -= 1  

        # rospy.loginfo(f"{right_end=}, {left_end=}")
        return (x, left_end), (x, right_end)

    def save_map_image(self, map, path):
        map_image = np.flip(np.flip(map, 1), 0)

        skimage.io.imsave(path, skimage.img_as_ubyte(255*map_image), check_contrast=False)
        rospy.loginfo(f"Saved map image to {path}")
    
    def erode_map(self, driveable_area, save_maps = False):
        radius = int(self.safety_margin/self.map_res)

        tic = time.time()
        eroded_map = skimage.morphology.binary_erosion(driveable_area, footprint = np.ones((2*radius,2*radius)))
        toc = time.time()
        rospy.loginfo(f"Time for binary erosion: {toc - tic}")

        if save_maps:
            rospack = rospkg.RosPack()
            path = f"{rospack.get_path('pure_pursuit')}/maps"
            if not os.path.exists(path):
                os.makedirs(path)
            self.save_map_image(driveable_area, f'{path}/driveable_area.png')
            self.save_map_image(eroded_map, f'{path}/eroded_map.png')

        return eroded_map
    
    @timing
    def dijkstra(self, map_msg, safe_area, starting_point, finish_line_start, finish_line_end, neighborhood):
        # Currently implemented with a 4-neighborhood - since Dijkstra is equivalent to breadth-first search
        # for uniform weights
        # Currently expects the finishline to always be horizontal
        x = finish_line_start[0]
        finish_line = [(x,y) for y in range(finish_line_start[1], finish_line_end[1] + 1)]

        visited = np.array(safe_area)
        visited.fill(False)

        priority_queue = []

        nodeCosts = defaultdict(lambda: float('inf'))
        first_step = (starting_point[0] + 1, starting_point[1])
        nodeCosts[first_step] = 0
        heap.heappush(priority_queue, (0, first_step))
        previous_node = {first_step: starting_point}

        i = 0
        while priority_queue:
            i += 1
            dist, (x,y) = heap.heappop(priority_queue)

            if (x,y) in finish_line:
                return previous_node, (x,y), dist

            visited[x,y] = True

            # Funky visualization of where the algorithm is currently exploring
            if i % 100 == 0:
                pos_x, pos_y = self.convert_grid_cell_to_position(x,y)
                marker = visualize_point(pos_x,pos_y)
                self.marker_pub.publish(marker)

            for delta_x, delta_y, weight in neighborhood:
                new_x = x + delta_x
                new_y = y + delta_y 

                if visited[new_x, new_y]:
                    continue

                if safe_area[new_x, new_y] == 1:
                    # The loop is only completed if the finish line is reached from the bottom
                    if ((new_x,new_y) in finish_line and new_x == x + 1) or (new_x,new_y) not in finish_line:
                        new_costs = nodeCosts[(x,y)] + weight
                        if new_costs < nodeCosts[(new_x,new_y)]:
                            previous_node[(new_x,new_y)] = (x,y)
                            nodeCosts[(new_x,new_y)] = new_costs
                            heap.heappush(priority_queue, (new_costs, (new_x,new_y)))

                    
        rospy.logerr("No path found from startpoint to finish line! Reduce the self.safety_margin parameter")

    @timing
    def shortest_path(self, map_msg, safe_area, start_point, finish_line_start, finish_line_end, neighborhood):
        "Use Dijkstra with a 4-neighborhood or an 8-neighborhood"

        previous_node, finish_point, dist = self.dijkstra(map_msg, safe_area, start_point, finish_line_start, finish_line_end, neighborhood)
        shortest_path = Path()
        pos_x, pos_y = self.convert_grid_cell_to_position(self.start_point[0],self.start_point[1])
        self.append_pose(map_msg, shortest_path, pos_x, pos_y)
    
        node = previous_node[finish_point]
        i = 0
        while node != start_point:
            i += 1
            node = previous_node[node]

            if (i % self.sparsity) == 0:
                pos_x, pos_y = self.convert_grid_cell_to_position(node[0],node[1])
                self.append_pose(map_msg, shortest_path, pos_x, pos_y)
        
        shortest_path.poses = shortest_path.poses[::-1]
        return shortest_path, finish_point, dist
    
    @timing
    def optimize_raceline(self, map_msg, shortest_path):
        x_array = np.array([pose.pose.position.x for pose in shortest_path.poses])
        y_array = np.array([pose.pose.position.y for pose in shortest_path.poses])

        rax, ray, heading, curvature = approximate_b_spline_path(
        x_array, y_array, len(x_array), degree = 3, s=0.5)

        optimized_path = Path()
        distance = 0

        prev_x = rax[0]
        prev_y = ray[0]
        for (x,y) in zip(rax,ray):
            distance += np.sqrt((x-prev_x)**2 + (y-prev_y)**2)
            self.append_pose(map_msg, optimized_path, x, y)
            prev_x = x
            prev_y = y

        return optimized_path, distance
    
    def append_pose(self, map_msg, path, pos_x, pos_y):
        cur_pose = PoseStamped()
        cur_pose.header = map_msg.header
        cur_pose.pose.position.x = pos_x
        cur_pose.pose.position.y = pos_y
        cur_pose.pose.position.z = 0
        path.poses.append(cur_pose)

    
    def convert_position_to_grid_cell(self, pos_x, pos_y):
        "Takes a position in meters and converts it to the corresponding grid cell in the OccupancyGrid"

        index_x = int(pos_x/self.map_res + self.map_height/2)
        index_y = int(pos_y/self.map_res + self.map_width/2)

        return index_x, index_y
    
    def convert_grid_cell_to_position(self, index_x, index_y):
        "Takes a tuple (i,j) of indices on the grid and converts it to its coordinates in meters."
        
        pos_x = (index_x - self.map_height/2)*self.map_res
        pos_y = (index_y - self.map_width/2)*self.map_res

        return pos_x, pos_y


    @timing
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

        map_binary = self.preprocess_map(map_msg)
        rospy.loginfo(f"number of free grid cells: {np.sum(map_binary)}")

        driveable_area = self.fill4(map_binary, self.start_point[0], self.start_point[1])
        rospy.loginfo(f"number of driveable grid cells: {np.sum(driveable_area)}")

        finish_line_start, finish_line_end = self.calculate_finish_line(driveable_area)    
        safe_area = self.erode_map(driveable_area, save_maps = False)
        rospy.loginfo(f"number of safe grid cells: {np.sum(safe_area)}")

        
        # possible neighborhoods encoded in (delta_x, delta_y, weight) format
        neighborhood4 = [(0,1,1), (0,-1,1), (1,0,1), (-1,0,1)]
        neighborhood8 = [(0,1,1), (0,-1,1), (1,0,1), (-1,0,1), (1,1,np.sqrt(2)), (1,-1,np.sqrt(2)), (-1,1, np.sqrt(2)), (-1,1, np.sqrt(2))]

        # shortest_path, distance = self.shortest_path(map_msg, safe_area, finish_line_start, finish_line_end, neighborhood4)
        # rospy.loginfo(f"Length of shortest path: {self.map_res * distance} meters")

        shortest_path, finish_point, distance = self.shortest_path(map_msg, safe_area, self.start_point, finish_line_start, finish_line_end, neighborhood8)
        rospy.loginfo(f"Length of shortest path (with diagonals): {self.map_res * distance} meters")

        shortest_path_flying_lap, _, distance_flying_lap = self.shortest_path(map_msg, safe_area, finish_point, finish_line_start, finish_line_end, neighborhood8)
        rospy.loginfo(f"Length of shortest path (with diagonals): {self.map_res * distance_flying_lap} meters")

        for path, publisher in zip((shortest_path, shortest_path_flying_lap),(self.path_pub, self.path_flying_lap_pub)):
            optimized_path, distance = self.optimize_raceline(map_msg, path)
            rospy.loginfo(f"Length of optimized path (with diagonals): {distance} meters")

            optimized_path.header = map_msg.header
            publisher.publish(optimized_path)




def main(args):
    rospy.init_node("planner", anonymous=True)
    follow_the_gap = PathGenerator()
    # rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
