#!/usr/bin/env python

import threading
import time
import math

import roslib; roslib.load_manifest('optnav_fcs')
import rospy

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import std_msgs.msg
import nav_msgs.msg

from dji_sdk.dji_drone import DJIDrone
import dji_sdk.msg
import dji_sdk.srv


'''
        self.activation_subscriber = rospy.Subscriber("dji_sdk/activation", std_msgs.msg.UInt8, self.activation_subscriber_callback)

        self.rc_channels_subscriber = rospy.Subscriber("dji_sdk/rc_channels", dji_sdk.msg.RCChannels, self.rc_channels_subscriber_callback)

        self.power_status_subscriber = rospy.Subscriber("dji_sdk/power_status", dji_sdk.msg.PowerStatus, self.power_status_subscriber_callback)

        self.flight_status_subscriber = rospy.Subscriber("dji_sdk/flight_status", std_msgs.msg.UInt8, self.flight_status_subscriber_callback)

        self.odometry_subscriber = rospy.Subscriber("dji_sdk/odometry", nav_msgs.msg.Odometry, self.odometry_subscriber_callback)

----
        self.sdk_permission_control_service = rospy.ServiceProxy("dji_sdk/sdk_permission_control", dji_sdk.srv.SDKPermissionControl)
 
    def request_sdk_permission_control(self):
        self.sdk_permission_control_service(control_enable = 1)

    def release_sdk_permission_control(self):
        self.sdk_permission_control_service(control_enable = 0)

----
        self.attitude_control_service = rospy.ServiceProxy("dji_sdk/attitude_control", dji_sdk.srv.AttitudeControl)

    def attitude_control(self, flag, x, y, z, yaw):
        self.attitude_control_service(flag = flag, x = x, y = y, z = z, yaw = yaw)
 
 
----
        self.drone_task_control_service = rospy.ServiceProxy("dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl)

    def takeoff(self):
        self.drone_task_control_service(task=4)

    def landing(self):
        self.drone_task_control_service(task=6)


'''

#===============================================================================

def limit_to_pi(angle):
    if (angle > math.pi):
        angle = angle - 2.0 * math.pi
    elif (angle < -math.pi):
        angle = angle + 2.0 * math.pi
    return angle

#===============================================================================
class PosSim:

    def __init__(self):
        self.max_horiz_speed = 5.0
        self.max_horiz_accel = 1.0
        self.max_vert_speed = 2.0
        self.max_vert_accel = 0.5
        self.max_yaw_rate  = math.pi/2.0
        self.max_yaw_accel = math.pi/2.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0


    def updateHorizontal(self, time_step, vx, vy):
        cmd_vx = vx 
        cmd_vy = vy
        cmd_ax = 0.0
        cmd_ay = 0.0        
        #Limit speed
        if (math.sqrt(cmd_vx*cmd_vx + cmd_vy*cmd_vy) > self.max_horiz_speed):
            angle = math.atan2(cmd_vy, cmd_vx)
            cmd_vx = self.max_horiz_speed * math.cos(angle)
            cmd_vy = self.max_horiz_speed * math.sin(angle)
        #Limit acceleration
        cmd_ax = (cmd_vx - self.vel_x)/time_step
        cmd_ay = (cmd_vy - self.vel_y)/time_step
        if (math.sqrt(cmd_ax*cmd_ax + cmd_ay*cmd_ay) > self.max_horiz_accel):
            angle = math.atan2(cmd_ay, cmd_ax)
            cmd_ax = self.max_horiz_accel * math.cos(angle)
            cmd_ay = self.max_horiz_accel * math.sin(angle)
        #Update speed 
        self.vel_x = self.vel_x + cmd_ax * time_step
        self.vel_y = self.vel_y + cmd_ay * time_step
        #Update position
        self.pos_x = self.pos_x + self.vel_x * time_step
        self.pos_y = self.pos_y + self.vel_y * time_step
        #rospy.loginfo("PosX={0} / VelX={1}".format(self.pos_x, self.vel_x))
        #rospy.loginfo("PosY={0} / VelY={1}".format(self.pos_y, self.vel_y))
        

    def updateVertical(self, time_step, vz):
        cmd_vz = vz
        #Limit speed
        if (math.fabs(cmd_vz) > self.max_vert_speed):
            if (cmd_vz > 0):
                cmd_vz = self.max_vert_speed
            else:        
                cmd_vz = -1.0 * self.max_vert_speed
        #Limit acceleration
        cmd_az = (cmd_vz - self.vel_z)/time_step
        if (math.fabs(cmd_az) > self.max_vert_accel):
            if (cmd_az > 0):
                cmd_az = self.max_vert_accel
            else: 
                cmd_az = -1.0 * self.max_vert_accel
        #Update speed
        self.vel_z = self.vel_z + cmd_az * time_step
        #update position
        self.pos_z = self.pos_z + self.vel_z * time_step
        #rospy.loginfo("PosZ={0} / VelZ={1}".format(self.pos_z, self.vel_z))


    def updateYaw(self, time_step, yaw):
        yaw = limit_to_pi(yaw)
        yaw_rate = self.yaw_rate
        if ((yaw - self.yaw) < 0.0):
            yaw_rate = -1.0 * self.yaw_rate
        yaw_accel = (yaw_rate - self.yaw_rate) / time_step
        yaw_accel = limit_to_pi(yaw_accel)
        if (math.fabs(yaw_accel) > self.max_yaw_accel):
            if (yaw_accel > 0.0):
                yaw_accel = self.max_yaw_accel
            else:
                yaw_accel = -1.0 * self.max_yaw_accel
        self.yaw_rate = self.yaw_rate + yaw_accel * time_step
        if (math.fabs(self.yaw_rate) > self.max_yaw_rate):
            if (self.yaw_rate > 0.0):
                self.yaw_rate = self.max_yaw_rate
            else:
                self.yaw_rate = -1.0 * self.max_yaw_rate
        self.yaw = limit_to_pi(self.yaw + self.yaw_rate * time_step)
        #rospy.loginfo("Yaw={0} / YawRate={1}".format(self.yaw, self.yaw_rate))
        

    def update(self, time_step, vx, vy, vz, yaw):
        #rospy.loginfo("CMD: vx={0} / vy={1} / vz={2} / yaw={3}".format(vx, vy, vz, yaw))
        self.updateHorizontal(time_step, vx, vy)
        self.updateVertical(time_step, vz)
        self.updateYaw(time_step, yaw)

    
#===============================================================================

class DjiSimMain:
    def __init__(self):
        rospy.loginfo("DjiSimMain() - Initialization")
        
        self.state = 'ONGROUND'
        self.takeOffCmdRecvTime = 0
        self.landCmdRecvTime = 0
        self.landedTime = 0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw = 0.0
        self.lastCmdTime = 0
        self.takeOffHeight = 0.0
        
        self.posSim = PosSim()
        
        #Messages to be published periodically
        self.activation = std_msgs.msg.UInt8()
        self.activation.data = True
        
        self.rc_channels = dji_sdk.msg.RCChannels()
        #self.rc_channels.header = 
        #self.rc_channels.ts = 
        self.rc_channels.roll = 0
        self.rc_channels.pitch = 0
        self.rc_channels.yaw = 0
        self.rc_channels.throttle = 0 
        self.rc_channels.mode = 10000 #API mode
        self.rc_channels.gear = 0
        
        self.power_status = dji_sdk.msg.PowerStatus()
        self.power_status.percentage = 100
        
        self.flight_status = std_msgs.msg.UInt8()
        self.flight_status.data = 1 #initial state on ground
        #1 initial state / on ground
        #2 taking off
        #3 in the air
        #4 landing
        #5 landed

        self.sdk_permission = std_msgs.msg.UInt8()
        self.sdk_permission.data = 0 #sdk permission enabled
        
        self.odometry = nav_msgs.msg.Odometry()
        
        #Message publishers
        self.activation_publisher     = rospy.Publisher("dji_sdk/activation", std_msgs.msg.UInt8, queue_size=1)
        self.rc_channels_publisher    = rospy.Publisher("dji_sdk/rc_channels", dji_sdk.msg.RCChannels, queue_size=1)
        self.power_status_publisher   = rospy.Publisher("dji_sdk/power_status", dji_sdk.msg.PowerStatus, queue_size=1)
        self.flight_status_publisher  = rospy.Publisher("dji_sdk/flight_status", std_msgs.msg.UInt8, queue_size=1)
        self.odometry_publisher       = rospy.Publisher("dji_sdk/odometry", nav_msgs.msg.Odometry, queue_size=1)
        self.sdk_permission_publisher = rospy.Publisher("dji_sdk/sdk_permission", std_msgs.msg.UInt8, queue_size=1)
        
        #Service handlers
        self.sdk_permission_control_service = rospy.Service("dji_sdk/sdk_permission_control", dji_sdk.srv.SDKPermissionControl, self.handle_sdk_permission_control)
        self.attitude_control_service       = rospy.Service("dji_sdk/attitude_control", dji_sdk.srv.AttitudeControl, self.handle_attitude_control)
        self.drone_task_control_service     = rospy.Service("dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl, self.handle_drone_task_control)
        
        self.start_time = rospy.Time.now().to_sec()
        self.counter = 0

    #-------------------------------------------------------------------------------

    '''
    #request control: 1
    #release control: 0
    uint8 control_enable
    ---
    bool result    
    '''

    def handle_sdk_permission_control(self, req):
        rospy.loginfo("Service - SDKPermissionControl")
        rospy.loginfo(req)
        if (req.control_enable == 1):
            self.sdk_permission.data = 1
        else:
            self.sdk_permission.data = 0
        return dji_sdk.srv.SDKPermissionControlResponse(True)
        
    '''    
    uint8 flag
    float32 x
    float32 y
    float32 z
    float32 yaw
    ---
    bool result

    HORIZ_ATT  = 0x00
    HORIZ_VEL  = 0x40
    HORIZ_POS  = 0x80
    VERT_VEL   = 0x00
    VERT_POS   = 0x10
    VERT_TRU   = 0x20
    YAW_ANG    = 0x00
    YAW_RATE   = 0x08
    HORIZ_GND  = 0x00
    HORIZ_BODY = 0x02
    YAW_GND    = 0x00
    YAW_BODY   = 0x01
    '''
    def handle_attitude_control(self, req):
        rospy.loginfo("Service - AttitudeControl")
        rospy.loginfo(req)
        #DJIDrone.HORIZ_VEL|DJIDrone.VERT_VEL|DJIDrone.YAW_ANG|DJIDrone.HORIZ_GND|DJIDrone.YAW_GND
        if (req.flag != (0x40|0x00|0x00|0x00|0x00)):
            return AttitudeControlResponse(False)

        self.cmd_vx  = req.x
        self.cmd_vy  = req.y
        self.cmd_vz  = -req.z
        self.cmd_yaw = req.yaw
        self.lastCmdTime = rospy.Time.now().to_sec()
        return dji_sdk.srv.AttitudeControlResponse(True)

    '''
    #takeoff: 4
    #landing: 6
    #gohome:  1
    uint8 task
    ---
    bool result
    '''
    def handle_drone_task_control(self, req):
        rospy.loginfo("Service - DroneTaskControl")
        rospy.loginfo(req)
        if (req.task == 4):
            if (self.state == 'ONGROUND'):
                self.state = 'TAKINGOFF'
                self.takeOffCmdRecvTime = rospy.Time.now().to_sec()
                self.takeOffHeight = self.posSim.pos_z
        elif (req.task == 6):
            if (self.state == 'INAIR'):
                self.state = 'LANDING'
                self.landCmdRecvTime = rospy.Time.now().to_sec()
        else:
            rospy.loginfo("Not Implemented")
        return dji_sdk.srv.DroneTaskControlResponse(True)

#-------------------------------------------------------------------------------

    def update_state(self):
        currTime = rospy.Time.now().to_sec()
        
        if (self.state == 'TAKINGOFF'):
            #if ((currTime - self.takeOffCmdRecvTime) > 1.0):
            if (math.fabs(-1.5 - self.posSim.pos_z) < 0.05):
                self.state = 'INAIR'

        elif (self.state == 'LANDING'):
            if (math.fabs(self.takeOffHeight - self.posSim.pos_z) < 0.05):
                self.state = 'LANDED'
                self.landedTime = rospy.Time.now().to_sec()

        elif (self.state == 'LANDED'):
            if ((currTime - self.landedTime) > 1.0):
                self.state = 'ONGROUND'
        
        if (self.state == 'ONGROUND'):
            self.flight_status.data = 1
        elif (self.state == 'TAKINGOFF'):
            self.flight_status.data = 2
        elif (self.state == 'INAIR'):
            self.flight_status.data = 3
        elif (self.state == 'LANDING'):
            self.flight_status.data = 4
        elif (self.state == 'LANDED'):
            self.flight_status.data = 5


    def update_odometry(self):
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = '/world'
        self.odometry.child_frame_id = ''

        currTime = rospy.Time.now().to_sec()
        if ((currTime - self.lastCmdTime) > 0.04):
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_vz = 0.0
            self.cmd_yaw = self.posSim.yaw

        if (self.state == 'TAKINGOFF'):
            diffHeight = -1.5 - self.posSim.pos_z
            vz = diffHeight
            if (vz < -1.0):
                vz = -1.0
            if (vz > -0.1):
                vz = -0.1
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_vz = vz
            self.cmd_yaw = self.posSim.yaw
        
        if (self.state == 'LANDING'):
            diffHeight = self.takeOffHeight - self.posSim.pos_z
            vz = diffHeight
            if (vz > 1.0):
                vz = 1.0
            if (vz < 0.1):
                vz = 0.1
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_vz = vz
            self.cmd_yaw = self.posSim.yaw

        self.posSim.update(0.02, self.cmd_vx, self.cmd_vy, self.cmd_vz, self.cmd_yaw)

        if ((self.counter % 250) == 0):
            rospy.loginfo("DJI Pos: x={0} / y={1} / z={2} / yaw={3}".format(self.posSim.pos_x, self.posSim.pos_y, self.posSim.pos_z, self.posSim.yaw))

        self.odometry.pose.pose.position.x = self.posSim.pos_x
        self.odometry.pose.pose.position.y = self.posSim.pos_y
        self.odometry.pose.pose.position.z = -self.posSim.pos_z
                
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.posSim.yaw)
        self.odometry.pose.pose.orientation.x = quaternion[0]
        self.odometry.pose.pose.orientation.y = quaternion[1]
        self.odometry.pose.pose.orientation.z = quaternion[2]
        self.odometry.pose.pose.orientation.w = quaternion[3]    
        
        self.odometry.twist.twist.linear.x = self.posSim.vel_x
        self.odometry.twist.twist.linear.y = self.posSim.vel_y
        self.odometry.twist.twist.linear.z = self.posSim.vel_z
        
        self.odometry.twist.twist.angular.x = 0.0
        self.odometry.twist.twist.angular.y = 0.0
        self.odometry.twist.twist.angular.z = self.posSim.yaw_rate


    def periodicTask(self):
        if ((self.counter % 20) == 0):
            self.activation_publisher.publish(self.activation)
            self.rc_channels_publisher.publish(self.rc_channels)
            self.power_status_publisher.publish(self.power_status)
            self.flight_status_publisher.publish(self.flight_status)
            self.sdk_permission_publisher.publish(self.sdk_permission)
        if (self.counter == 50):
            self.counter = 0
        self.update_state()
        self.update_odometry()
        self.odometry_publisher.publish(self.odometry)
        

#===============================================================================

def main():
    rospy.loginfo('dji_sim_node - START')
    
    rospy.init_node('dji_sim_node')

    sim = DjiSimMain()

    r = rospy.Rate(50)
    while (not rospy.is_shutdown()):
        sim.periodicTask()
        r.sleep()
        
    time.sleep(1)
    rospy.loginfo('dji_sim_node - END')


if __name__ == '__main__':
    main()

