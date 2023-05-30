import rospy
from visualization_msgs.msg import Marker


def visualize_point(x,y,frame='map',r=0.0,g=1.0,b=0.0):
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
    return marker
