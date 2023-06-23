#!/usr/bin/env python3
import sys
import numpy as np
import scipy
from PIL import Image

import rospy
import rospkg
from nav_msgs.msg import OccupancyGrid


class LikelihoodFieldGenerator:
    """ Creating a path for the car to drive using only data from the /map topic.
    """

    def __init__(self):
        # Subscribers

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.compute_likelihood_field, queue_size=1)
        self.scale = 1

        # Publishers

    def compute_likelihood_field(self, map_msg):
        
        self.map_width = int(np.ceil(map_msg.info.width/self.scale))
        self.map_height = int(np.ceil(map_msg.info.height/self.scale))
        self.map_res = map_msg.info.resolution*self.scale
        
        map_data = np.array(map_msg.data).reshape((map_msg.info.width, map_msg.info.height)).T
        map_data[map_data == - 1] = 100

        print(np.min(map_data), np.max(map_data))


        smoothening_matrix = np.ones((5,5))/25
        likelihood_field = scipy.signal.convolve2d(map_data, smoothening_matrix, mode='same')
        im = Image.fromarray(likelihood_field).convert("L")
        rospack = rospkg.RosPack()
        im.save(f"{rospack.get_path('vehicle')}/maps/likelihood_field.png")



def main(args):
    rospy.init_node("likelihood_field", anonymous=True)
    likelihood_field = LikelihoodFieldGenerator()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
