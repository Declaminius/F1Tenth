#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from safety_node2.cfg import safety_node2Config

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.threshold = 0.4
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.break_pub = rospy.Publisher("brake",AckermannDriveStamped,queue_size=1000)
        self.break_bool_pub = rospy.Publisher("brake_bool",Bool,queue_size=1000)
        self.ttc_pub = rospy.Publisher("ttc", Float32, queue_size = 1000)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        n = len(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(n)*scan_msg.angle_increment
        projected_speed = self.speed*np.cos(angles)
        mask = projected_speed > 0
        ttc = np.array([float("inf")]*n)
        ttc[mask] = np.array(scan_msg.ranges)[mask]/projected_speed[mask]
        
        min_ttc = np.min(ttc)
        self.ttc_pub.publish(min_ttc)
        
        if min_ttc < self.threshold and self.speed > 0:
            break_msg = AckermannDriveStamped()
            break_bool_msg = Bool()
            break_msg.drive.speed = 0
            break_bool_msg.data = True
            self.break_pub.publish(break_msg)
            self.break_bool_pub.publish(break_bool_msg)
        else:
            break_bool_msg = Bool()
            break_bool_msg.data = False
            self.break_bool_pub.publish(break_bool_msg)
    
    def dyn_callback(self, config, level):
        self.threshold = config.threshold
        return config


def main():
    rospy.init_node('safety_node2')
    sn = Safety()

    srv = Server(safety_node2Config, sn.dyn_callback)
    rospy.spin()
if __name__ == '__main__':
    main()
