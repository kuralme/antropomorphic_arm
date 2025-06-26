#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import sin, cos, pi
import random 

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    """
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


class MarkerBasics(object):

    def __init__(self):
        self.marker_publisher = rospy.Publisher('/ee_position', Marker, queue_size=1)
        self.current_index = 0  # Counter for unique marker IDs

    def publish_point(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "frame_0"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = ""
        marker.id = self.current_index  # Unique ID for each marker
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Random color
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()
        marker.color.a = 0.8

        marker.lifetime = rospy.Duration(0)  # Keep forever

        self.marker_publisher.publish(marker)
        self.current_index += 1  # Increment for next marker

    def start(self):
        rate = rospy.Rate(10)
        z_value = 0.0
        delta = 0.1
        up_flag = True
        z_MAX = 1.5
        z_MIN = -1.0
        theta = 0
        delta_theta = 0.2
        while not rospy.is_shutdown():
            self.publish_point(cos(theta), sin(theta), z_value)
            theta += delta_theta
            if theta >= 2 * pi:
                theta = 0.0
                if up_flag:
                    z_value += delta
                else:
                    z_value -= delta
                if z_value >= z_MAX:
                    up_flag = False
                if z_value <= z_MIN:
                    up_flag = True
            print("Z = " + str(z_value))
            rate.sleep()
   

if __name__ == '__main__':
    rospy.init_node('marker_basic_node', anonymous=True)
    markerbasics_object = MarkerBasics()
    try:
        markerbasics_object.start()
    except rospy.ROSInterruptException:
        pass