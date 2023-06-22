import rospy
from visualization_msgs.msg import Marker


def visualize_point(x,y,frame='map',r=0.0,g=1.0,b=0.0, time=0.25):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.id = 150
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.25
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.lifetime = rospy.Duration(time)
    return marker

def visualize_finish_line(finish_line):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.
    marker.color.b = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    # marker.colors = colors
    marker.lifetime = rospy.Duration(0)

    marker.points = finish_line
    return marker