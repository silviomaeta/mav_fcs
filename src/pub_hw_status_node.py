#!/usr/bin/env python

import time
import thread

import roslib; roslib.load_manifest('mav_fcs')
import rospy

import hw_status

from mav_gcs_msgs.msg import *

#-------------------------------------------------------------------------------

def main():
    rospy.init_node('pub_hw_status_node')
    rospy.loginfo('pub_hw_status_node - START')

    mavHwStatusPub  = rospy.Publisher('/mav/hardware_status', HardwareStatus, queue_size = 1)
    hwStatusCounter = 0
    
    r = rospy.Rate(1)
    while (not rospy.is_shutdown()):

        hwstatus = HardwareStatus()
        hwstatus.header.stamp = rospy.Time.now()
        hwstatus.index = hwStatusCounter
        hwstatus.battery_level = 0
        hwstatus.rc_comm_ok = True
        hwstatus.gcs_comm_ok = True
        hwstatus.cpu_usage = hw_status.getCpuUsage()
        hwstatus.ram_usage = hw_status.getRamUsage()
        hwstatus.storage_usage = hw_status.getStorageUsage()
        
        sensor = ComponentStatus()
        sensor.name = 'camera'
        sensor.status = ComponentStatus.OK
        sensor.status_description = 'mav camera sensor (test)'
        hwstatus.sensors.append(sensor)

        mavHwStatusPub.publish(hwstatus)
        hwStatusCounter = hwStatusCounter + 1

        r.sleep()
        
    time.sleep(1)
    rospy.loginfo('pub_hw_status_node - END')


if __name__ == '__main__':
    main()

