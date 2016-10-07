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

def getWaypointTarget(target_x, target_y, target_z):
    path = PathXYZVPsi()
    
    path.header.seq = 1
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = '/world'
        
    wp1 = XYZVPsi()
    wp1.position.x = target_x
    wp1.position.y = target_y
    wp1.position.z = target_z
    wp1.vel = 1.0
    wp1.heading = 0.0
    path.waypoints.append(wp1)
    rospy.loginfo(wp1)

    wp2 = XYZVPsi()
    wp2.position.x = 0.0
    wp2.position.y = 0.0
    wp2.position.z = -1.5
    wp2.vel = 1.0
    wp2.heading = 0.0
    path.waypoints.append(wp2)
    rospy.loginfo(wp2)

    return path



def main():

    if (len(sys.argv) != 2):
        print './pub_waypoints.py <position code>'
        print 'Publishes waypoints given position code: 1 to 4'
        exit(-1)


    rospy.init_node('pub_waypoints')
    rospy.loginfo('pub_waypotins - START')

    pubPath = rospy.Publisher("/fcs/path", PathXYZVPsi, queue_size=1)

    if (sys.argv[1] == "1"):
        path = getWaypointTarget(5.0, 5.0, -2.5)
    elif (sys.argv[1] == "2"):
        path = getWaypointTarget(-10.0, 10.0, -3.0)            
    elif (sys.argv[1] == "3"):
        path = getWaypointTarget(-10.0, -10.0, -3.0)            
    elif (sys.argv[1] == "4"):
        path = getWaypointTarget(10.0, -10.0, -3.0)            
    
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

