#!/usr/bin/env python3
import rospy
import tf
import sys

# Messages
from nav_msgs.msg import Odometry


class RosOdomPublisher:

    def __init__(self):
        self.tf_br = tf.TransformBroadcaster()
        self.odom_sub = rospy.Subscriber("/vesc/odom", Odometry, self.odom_callback, queue_size=1)

    def odom_callback(self, odom_msg):
        pos = (odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y,
                odom_msg.pose.pose.position.z)

        ori = (odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w)
        self.tf_br.sendTransform(pos, ori, odom_msg.header.stamp, odom_msg.child_frame_id, odom_msg.header.frame_id)

def main(args):
    rospy.init_node("odom_broadcaster")
    odom_pub = RosOdomPublisher()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)