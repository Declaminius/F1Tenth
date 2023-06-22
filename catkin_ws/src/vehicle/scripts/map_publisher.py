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


class MapPublisher:
    def __init__(self) -> None:
        self.map_pub = rospy.Publisher('/testmap', OccupancyGrid, latch=True, queue_size=1)

    def set_map(self, map):
        self.map_pub.publish(map)

def main(args):
    rospy.init_node("map_publisher", anonymous=True)
    rfgs = MapPublisher()
    rospy.sleep(0.1)
    rospack = rospkg.RosPack()
    with open(f"{rospack.get_path('vehicle')}/map.bin", "rb") as f:
        buf = BytesIO(f.read())
        map = OccupancyGrid()
        map.deserialize(buf.getvalue())
        rfgs.set_map(map)
        rospy.sleep(0.1)
    rospy.spin()  



if __name__ == '__main__':
    main(sys.argv)

