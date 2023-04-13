#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 14
kd = 0.09
ki = 0
theta = (35/180)*np.pi
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
DESIRED_DISTANCE_LEFT = 0.55
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 1000)

    def getRange(self, scan_msg, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # TODO: make sure to take care of nans etc.
        index = int(((angle*180/np.pi + 45)*len(scan_msg.ranges))//270)
        return scan_msg.ranges[index]

    def pid_control(self, error, time):
        global integral
        global prev_error
        global prev_time
        global kp
        global ki
        global kd
        global time_steps
        global speed
        
        integral += (error - integral)/(time_steps+1)
        
        angle = kp*error + ki*integral + kd*(error - prev_error)/(time - prev_time)

        
        if (0 <= abs(angle) <= 10):
            speed = 1.5
        elif (10 < abs(angle) <= 20):
            speed = 1
        else:
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

    def followLeft(self, scan_msg, leftDist):
        #Follow left wall as per the algorithm 
        #TODO: change formula to left side
        global theta
        global L
        global speed
        
        a = self.getRange(scan_msg, np.pi - theta)
        b = self.getRange(scan_msg, np.pi)
 
        alpha = np.arctan((a*np.cos(theta) - b)/(a*np.sin(theta)))
        
        D_t = b * np.cos(alpha)
        
        next_distance = D_t + L*np.sin(np.pi - alpha)
        
        error = next_distance - leftDist
        
        return error
        
    def followRight(self, scan_msg, rightDist):
        #Follow left wall as per the algorithm 
       
        global theta
        global L
        
        a = self.getRange(scan_msg, theta)
        b = self.getRange(scan_msg, 0)
 
        alpha = np.arctan((a*np.cos(theta) - b)/(a*np.sin(theta)))
        D_t = b * np.cos(alpha)
        
        next_distance = D_t + L*np.sin(alpha)
        
        error = rightDist - next_distance
       
        return error

    def lidar_callback(self, scan_msg):
        """ 
        """
        global DESIRED_DISTANCE_LEFT
        error = self.followLeft(scan_msg, DESIRED_DISTANCE_LEFT)
        time = scan_msg.header.stamp.secs + scan_msg.header.stamp.nsecs*1e-9
        #send error to pid_control
        self.pid_control(error, time)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
