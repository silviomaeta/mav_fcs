#!/usr/bin/env python
import sys
import time
import math

import roslib; roslib.load_manifest('mav_fcs')
import rospy

from mav_gcs_msgs.msg import *


def main():

    if (len(sys.argv) != 2):
        print './pub_usercmd.py <user cmd type>'
        print 'Publishes user command: TAKEOFF / LAND / START'
        exit(-1)

    rospy.init_node('pub_usercmd')
    rospy.loginfo('pub_usercmd - START : {0}'.format(len(sys.argv)))

    pubUserCmd = rospy.Publisher("/fcs/user_cmd", UserCmd, queue_size=1)
    
    userCmd = UserCmd()
    userCmd.index = 1
    if (sys.argv[1] == 'TAKEOFF'):
        userCmd.user_cmd = UserCmd.CMD_TAKEOFF
    elif (sys.argv[1] == 'LAND'):
        userCmd.user_cmd = UserCmd.CMD_LAND
    elif (sys.argv[1] == 'START'):
        userCmd.user_cmd = UserCmd.CMD_MISSION_START
    else:
        print 'ERROR: Unknown user command type'
        exit(-1)
    
    counter = 0
    r = rospy.Rate(1)
    while ((not rospy.is_shutdown()) and (counter < 2)):
        userCmd.header.stamp = rospy.Time.now()
        pubUserCmd.publish(userCmd)
        r.sleep()
        counter = counter + 1
    
    time.sleep(1)
    rospy.loginfo('pub_usercmd - END')


if __name__ == '__main__':
    main()

