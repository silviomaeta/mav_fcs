#!/usr/bin/env python

import time
import math

import roslib; roslib.load_manifest('mav_fcs')
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ca_nav_msgs.msg import *


def getPathCircle(height, cruise_speed, radius):
    path = PathXYZVPsi()
    
    path.header.seq = 1
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = '/world'
        
    numSteps = 360
    for i in range(numSteps/2):
        wp = XYZVPsi()
        wp.position.x = math.cos(i * (2 * math.pi / numSteps)) * radius - radius
        wp.position.y = math.sin(i * (2 * math.pi / numSteps)) * radius
        wp.position.z = height
        wp.vel = cruise_speed
        wp.heading = 0.0
        path.waypoints.append(wp)
        rospy.loginfo(wp)

    return path

'''
def getMissionStraightLine(height, cruise_speed, distance, direction):
    mission = Mission()

    direction_angle = 0.0
    if (direction == 'north'):
        direction_angle = 0.0
    elif (direction == 'north_east'):
        direction_angle = math.pi/4.0
    elif (direction == 'east'):
        direction_angle = 2.0*math.pi/4.0
    elif (direction == 'south_east'):
        direction_angle = 3.0*math.pi/4.0
    elif (direction == 'south'):
        direction_angle = math.pi
    elif (direction == 'south_west'):
        direction_angle = 5.0*math.pi/4.0
    elif (direction == 'west'):
        direction_angle = 6.0*math.pi/4.0
    elif (direction == 'north_west'):
        direction_angle = 7.0*math.pi/4.0
    
    mission.index = 1
    mission.coordinate_type = Mission.COORDINATES_LOCAL
    mission.operational_air_volume = Polygon()

    wp_ini_1 = Waypoint()
    wp_ini_1.position.x = 0.0
    wp_ini_1.position.y = 0.0
    wp_ini_1.position.z = height
    wp_ini_1.heading_rad = 0.0
    wp_ini_1.speed_mps = cruise_speed
    mission.waypoints.append(wp_ini_1)
    rospy.loginfo(wp_ini_1)

    wp_end_1 = Waypoint()
    wp_end_1.position.x = distance * math.cos(direction_angle)
    wp_end_1.position.y = distance * math.sin(direction_angle)
    wp_end_1.position.z = height
    wp_end_1.heading_rad = 0.0
    wp_end_1.speed_mps = cruise_speed
    mission.waypoints.append(wp_end_1)
    rospy.loginfo(wp_end_1)
    
    return mission
'''

def main():
    rospy.init_node('pub_path')
    rospy.loginfo('pub_path - START')

    pubPath = rospy.Publisher("/fcs/path", PathXYZVPsi, queue_size=1)
    
    height       = -1.5
    cruise_speed =  0.5
    radius       = 10.0
        
    path = getPathCircle(height, cruise_speed, radius)
    
    counter = 0
    r = rospy.Rate(1)
    while ((not rospy.is_shutdown()) and (counter < 2)):
        #rospy.loginfo(".")
        path.header.seq   = path.header.seq + 1
        path.header.stamp = rospy.Time.now()
        pubPath.publish(path)
        r.sleep()
        counter = counter + 1
    
    rospy.loginfo('pub_path - END')


if __name__ == '__main__':
    main()

