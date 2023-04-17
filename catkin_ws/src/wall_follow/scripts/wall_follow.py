#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

# from dynamic_reconfigure.server import Server
# from wall_follow.cfg import dynamicConfig

#PID CONTROL PARAMS
kp = 1.4
kd = 0
ki = 0
thetas = [lol / 180 *np.pi for lol in [35,45,60,70,80]]
L = 1

servo_offset = 0.0
prev_error = 0.0 
prev_time = 0.0
error = 0.0
integral = 0.0
time_steps = 0
speed = 0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.9
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        
        self.desired_speed_pub = rospy.Publisher("/desired_speed_topic", Float32, queue_size = 1000)
        self.actual_speed_pub = rospy.Publisher("/actual_speed_topic", Float32, queue_size = 1000)
        self.steering_angle_pub = rospy.Publisher("/steering_angle_topic", Float32, queue_size = 1000)
        self.left_dist_pub = rospy.Publisher("/left_dist_topic", Float32, queue_size = 1000)
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 1000)

    def getRange(self, scan_msg, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # TODO: make sure to take care of nans etc.

        angle_degrees = angle * 180 / np.pi
        position = (angle_degrees + 45) / 270
        length = len(scan_msg.ranges)
        index = int(length * position)
        return scan_msg.ranges[index]

    def pid_control(self, scan_msg, error, ttc, time):
        global integral
        global prev_error
        global prev_time
        global kp
        global ki
        global kd
        global time_steps
        global speed
        threshold = 1.5
        
        integral += (error - integral)/(time_steps+1)
        
        derivative = (error - prev_error)/(time - prev_time)
        angle = kp*error + ki*integral + kd*derivative
        
        if (0 <= abs(angle) <= 10):
            speed = 1
        elif (10 < abs(angle) <= 20):
            speed = 0.75
        else:
            speed = 0.5
        
        if self.getRange(scan_msg, np.pi / 2) < threshold:
            speed = 0.5
        
        if ttc < 0.5:
            speed = 0.5

        prev_time = time
        prev_error = error
        time_steps += 1

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        
        self.drive_pub.publish(drive_msg)
        self.desired_speed_pub.publish(speed)
        self.steering_angle_pub.publish(angle)
        

    def followLeft(self, scan_msg, leftDist):
        #Follow left wall as per the algorithm 
        #TODO: change formula to left side
        global thetas
        global L
        global speed

        error_average = 0
        for theta in thetas:

            a = self.getRange(scan_msg, np.pi - theta)
            b = self.getRange(scan_msg, np.pi)
    
            alpha = np.arctan((a*np.cos(theta) - b)/(a*np.sin(theta)))
            
            D_t = b * np.cos(alpha)
            
            next_distance = D_t + L*np.sin(np.pi - alpha)
            
            error_average += next_distance - leftDist
            # print(f"{next_distance=}")
            # print(f"{theta=}")
        error_average /= len(thetas)
        self.left_dist_pub.publish(error_average)        
        return error_average
        
    def followRight(self, scan_msg, rightDist):
        #Follow left wall as per the algorithm 
        # TODO: averaging over thetas
       
        global theta
        global L
        
        a = self.getRange(scan_msg, theta)
        b = self.getRange(scan_msg, 0)
 
        alpha = np.arctan((a*np.cos(theta) - b)/(a*np.sin(theta)))
        D_t = b * np.cos(alpha)
        
        next_distance = D_t + L*np.sin(alpha)
        
        error = rightDist - next_distance
       
        return error
        
    def calculate_ttc(self, scan_msg):
    	global speed
    	n = len(scan_msg.ranges)
    	angles = scan_msg.angle_min + np.arange(n)*scan_msg.angle_increment
    	projected_speed = speed*np.cos(angles)
    	mask = projected_speed > 0
    	ttc = np.array([float("inf")]*n)
    	ttc[mask] = np.array(scan_msg.ranges)[mask]/projected_speed[mask]
    	return np.min(ttc)

    def lidar_callback(self, scan_msg):
        """ 
        """
        global DESIRED_DISTANCE_LEFT
        error = self.followLeft(scan_msg, DESIRED_DISTANCE_LEFT)
        ttc = self.calculate_ttc(scan_msg)
        time = scan_msg.header.stamp.secs + scan_msg.header.stamp.nsecs*1e-9
        #send error to pid_control
        self.pid_control(scan_msg, error, ttc, time)
    
    def odom_callback(self, odom_msg):
        self.actual_speed = odom_msg.twist.twist.linear.x
        self.actual_speed_pub.publish(self.actual_speed)
    
    def dyn_callback(self,config,levels):
        kd = config.kd
        kp = config.kp
        ki = config.ki

def main(args):
    rospy.init_node("wall_follow", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()
   # srv = Server(dynamicConfig, wf.dyn_callback)

if __name__=='__main__':
	main(sys.argv)
