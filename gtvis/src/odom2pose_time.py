#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker

def odom_callback(msg):
    global last_position
    global marker_id
    # Extract position and orientation from Odometry message
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    
    # Create a Marker message for visualization in RViz
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.id = marker_id
    marker.header.stamp = msg.header.stamp
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position = position
    # marker.pose.orientation = orientation
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    # 更改球的大小
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    # 球的颜色
    marker.color.a = 1.0
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.lifetime = rospy.Duration(0)# 图像停留时间为5s 填0则为永久
    if last_position is not None:
        line = Marker()
        line.header.frame_id = "odom"
        line.header.stamp = msg.header.stamp
        line.id = marker_id
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        # 更改线宽
        line.scale.x = 0.01
        # 线的颜色
        line.color.r = red
        line.color.g = green
        line.color.b = blue
        line.color.a = 1.0
        line.lifetime = rospy.Duration(0)# 图像停留时间为5s 填0则为永久

        line.points.append(last_position)
        line.points.append(marker.pose.position)
        line_pub.publish(line)

    last_position = marker.pose.position
    # Publish the Marker message
    marker_publisher.publish(marker)
    marker_id += 1
    
if __name__ == '__main__':
    rospy.init_node('odom_to_pose')
    # Get parameters for topics

    last_position = None
    marker_id = 0
    odom_topic = rospy.get_param('~odom_topic', 'odom')
    pose_topic = rospy.get_param('~pose_topic', 'pose')
    line_topic = rospy.get_param('~line_topic', 'line')
    red = rospy.get_param('~red',1)
    green = rospy.get_param('~green',1)
    blue = rospy.get_param('~blue',1)
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    marker_publisher = rospy.Publisher(pose_topic, Marker, queue_size=100000)
    line_pub = rospy.Publisher(line_topic, Marker, queue_size=100000)
    
    
    rospy.spin()