/*
* Copyright (c) 2016 Carnegie Mellon University, Silvio Maeta <smaeta@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#include <limits>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>

#include <geom_cast/geom_cast.hpp>

#include "mav_fcs/copter_interface.h"

using namespace mavfcs;

//==============================================================================
//Auxiliar functions - flat earth conversions
//Input lat/long angles in radians

const static double math_pi = 3.1415926535897932384626433832795;

const static double r_earth = 180*60*1852/3.1415926535897932384626433832795;

double deg2rad(double deg) {
    return ((deg * 3.1415926535897932384626433832795) / 180.0);
}

double rad2deg(double rad) {
    return ((rad * 180.0) / 3.1415926535897932384626433832795);
}


//==============================================================================

double limit_to_pi(double angle) {
    if (angle > math_pi) {
        angle = angle - 2.0 * math_pi;
    }
    else if (angle < -math_pi) {
        angle = angle + 2.0 * math_pi;
    }
    return angle;
}

//==============================================================================

class CopterPosSim {

public:

    //Limit values
    double max_horiz_speed;
    double max_horiz_accel;
    double max_vert_speed;
    double max_vert_accel;
    double max_yaw_rate;
    double max_yaw_accel;

    //Internal state values
    double pos_x;
    double pos_y;
    double pos_z;
    double vel_x;
    double vel_y;
    double vel_z;
    double yaw;
    double yaw_rate;

    CopterPosSim() {
        max_horiz_speed = 5.0;
        max_horiz_accel = 1.0;
        max_vert_speed = 2.0;
        max_vert_accel = 0.5;
        max_yaw_rate  = math_pi/2.0;
        max_yaw_accel = math_pi/2.0;

        pos_x = 0.0;
        pos_y = 0.0;
        pos_z = 0.0;
        vel_x = 0.0;
        vel_y = 0.0;
        vel_z = 0.0;
        yaw = 0.0;
        yaw_rate = 0.0;
    };
    
    ~CopterPosSim() {
    }
    
    void updateHorizontal(double time_step, double vx, double vy) {
        double cmd_vx = vx;
        double cmd_vy = vy;
        double cmd_ax = 0.0;
        double cmd_ay = 0.0;
        //Limit speed
        if (sqrt(cmd_vx*cmd_vx + cmd_vy*cmd_vy) > max_horiz_speed) {
            double angle = atan2(cmd_vy, cmd_vx);
            cmd_vx = max_horiz_speed * cos(angle);
            cmd_vy = max_horiz_speed * sin(angle);
        }
        //Limit acceleration
        cmd_ax = (cmd_vx - vel_x)/time_step;
        cmd_ay = (cmd_vy - vel_y)/time_step;
        if (sqrt(cmd_ax*cmd_ax + cmd_ay*cmd_ay) > max_horiz_accel) {
            double angle = atan2(cmd_ay, cmd_ax);
            cmd_ax = max_horiz_accel * cos(angle);
            cmd_ay = max_horiz_accel * sin(angle);
        }
        //Update speed 
        vel_x = vel_x + cmd_ax * time_step;
        vel_y = vel_y + cmd_ay * time_step;
        //Update position
        pos_x = pos_x + vel_x * time_step;
        pos_y = pos_y + vel_y * time_step;
        //rospy.loginfo("PosX={0} / VelX={1}".format(self.pos_x, self.vel_x))
        //rospy.loginfo("PosY={0} / VelY={1}".format(self.pos_y, self.vel_y))
    }

    void updateVertical(double time_step, double vz) {
        double cmd_vz = vz;
        //Limit speed
        if (fabs(cmd_vz) > max_vert_speed) {
            if (cmd_vz > 0) {
                cmd_vz = max_vert_speed;
            }
            else {        
                cmd_vz = -1.0 * max_vert_speed;
            }
        }
        //Limit acceleration
        double cmd_az = (cmd_vz - vel_z)/time_step;
        if (fabs(cmd_az) > max_vert_accel) {
            if (cmd_az > 0) {
                cmd_az = max_vert_accel;
            }
            else { 
                cmd_az = -1.0 * max_vert_accel;
            }
        }
        //Update speed
        vel_z = vel_z + cmd_az * time_step;
        //update position
        pos_z = pos_z + vel_z * time_step;
        //rospy.loginfo("PosZ={0} / VelZ={1}".format(self.pos_z, self.vel_z))
    }

    void updateYaw(double time_step, double yaw) {
        double cmd_yaw = limit_to_pi(yaw);
        double cmd_yaw_rate = yaw_rate;
        if ((cmd_yaw - yaw) < 0.0) {
            cmd_yaw_rate = -1.0 * yaw_rate;
        }
        double cmd_yaw_accel = (cmd_yaw_rate - yaw_rate) / time_step;
        //Limit yaw acceleration
        cmd_yaw_accel = limit_to_pi(cmd_yaw_accel);
        if (fabs(cmd_yaw_accel) > max_yaw_accel) {
            if (cmd_yaw_accel > 0.0) {
                cmd_yaw_accel = max_yaw_accel;
            }
            else {
                cmd_yaw_accel = -1.0 * max_yaw_accel;
            }
        }
        yaw_rate = yaw_rate + cmd_yaw_accel * time_step;
        //Limit yaw rate
        if (fabs(yaw_rate) > max_yaw_rate) {
            if (yaw_rate > 0.0) {
                yaw_rate = max_yaw_rate;
            }
            else {
                yaw_rate = -1.0 * max_yaw_rate; 
            }
        }
        //Set new yaw value
        yaw = limit_to_pi(yaw + yaw_rate * time_step);
        //rospy.loginfo("Yaw={0} / YawRate={1}".format(self.yaw, self.yaw_rate))
    }

    void update(double time_step, double vx, double vy, double vz, double cyaw) {
        //rospy.loginfo("CMD: vx={0} / vy={1} / vz={2} / yaw={3}".format(vx, vy, vz, yaw))
        updateHorizontal(time_step, vx, vy);
        updateVertical(time_step, vz);
        updateYaw(time_step, cyaw);
        //ROS_INFO_STREAM_THROTTLE(1.0, "DRONE SIM: Timestep=" << time_step << " / vx=" << vx << " / vy=" << vy << " / vz=" << vz << " / yaw=" << cyaw);
        //ROS_INFO_STREAM_THROTTLE(1.0, "DRONE SIM: Pos x=" << pos_x << " / y=" << pos_y << " / z=" << pos_z << " / yaw=" << yaw);
    }

};

//==============================================================================

class DroneSim {

public:
    CopterPosSim pos_sim;
    bool activation;
    unsigned int flight_status;
    double prev_time;
    double cmd_vx, cmd_vy, cmd_vz, cmd_yaw;
    double takeOffHeight;
    nav_msgs::Odometry odometry;
    double home_latitude, home_longitude, home_altitude;
    double latitude, longitude, altitude;
    
    DroneSim(void) {
        activation = false;
        flight_status = ON_GROUND;
        prev_time = -1.0;
        cmd_vx = cmd_vy = cmd_vz = 0.0;
        cmd_yaw = 0.0;
        home_latitude  = latitude  = 40.583401;
        home_longitude = longitude = -79.899775;
        home_altitude  = altitude  = 347.47;
        odometry.header.seq = 0;
    }

    void release_sdk_permission_control(void) {
        activation = false;
    }
    
    void request_sdk_permission_control(void) {
        activation = true;
    }

    void takeoff(void) {
        if (flight_status == ON_GROUND) {
            flight_status = TAKING_OFF;
            takeOffHeight = pos_sim.pos_z;
        }
    }
    
    void landing(void) {
        if (flight_status == IN_AIR) {
            flight_status = LANDING;
        }
    }

    void attitude_control(double vx, double vy, double vz, double yaw) {
        cmd_vx = vx;
        cmd_vy = vy;
        cmd_vz = vz;
        cmd_yaw = yaw;
    }

    void update_position(void) {
        //Set appropriate commands
        if (flight_status == TAKING_OFF) {
            double diffHeight = takeOffHeight - 1.5 - pos_sim.pos_z;
            double vz = diffHeight;
            if (vz < -1.0) vz = -1.0;
            if (vz > -0.1) vz = -0.1;
            
            cmd_vx = 0.0;
            cmd_vy = 0.0;
            cmd_vz = vz;
            cmd_yaw = pos_sim.yaw;
            
            if (fabs(diffHeight) < 0.2) {
                flight_status = IN_AIR;
            }
        }
        else if (flight_status == LANDING) {
            double diffHeight = takeOffHeight - pos_sim.pos_z;
            double vz = diffHeight;
            if (vz > 1.0) vz = 1.0;
            if (vz < 0.1) vz = 0.1;
            
            cmd_vx = 0.0;
            cmd_vy = 0.0;
            cmd_vz = vz;
            cmd_yaw = pos_sim.yaw;

            if (fabs(diffHeight) < 0.1) {
                flight_status = ON_GROUND;
            }
        }
        else if (flight_status == IN_AIR) {
            //Use speed set in attitude_control
        }
        else {
            cmd_vx = 0.0;
            cmd_vy = 0.0;
            cmd_vz = 0.0;
            cmd_yaw = pos_sim.yaw;        
        }

        //Update position using commands
        double curr_time = ros::Time::now().toSec();
        if (prev_time == -1.0) {
            prev_time = curr_time;
            return;
        }
        double time_step = curr_time - prev_time;
        prev_time = curr_time;

        pos_sim.update(time_step, cmd_vx, cmd_vy, cmd_vz, cmd_yaw);

        //Update odometry information
        odometry.header.stamp = ros::Time::now();
        odometry.header.seq = odometry.header.seq + 1;

        odometry.pose.pose.position.x = pos_sim.pos_x;
        odometry.pose.pose.position.y = pos_sim.pos_y;
        odometry.pose.pose.position.z = pos_sim.pos_z;

        odometry.twist.twist.linear.x = pos_sim.vel_x;
        odometry.twist.twist.linear.y = pos_sim.vel_y;
        odometry.twist.twist.linear.z = pos_sim.vel_z;
        
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, pos_sim.yaw);
        odometry.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

        odometry.twist.twist.angular.x = 0.0;
        odometry.twist.twist.angular.y = 0.0;
        odometry.twist.twist.angular.z = pos_sim.yaw_rate;

        //Update global position in latitude, longitude and altitude        
        double lat_rad = deg2rad(home_latitude);
        double lon_rad = deg2rad(home_longitude);
        latitude  = rad2deg(lat_rad + (pos_sim.pos_x/r_earth));
        longitude = rad2deg(lon_rad + (pos_sim.pos_y/(r_earth * cos(lat_rad))));
        altitude  = home_altitude - pos_sim.pos_z;
        
    }

};

//==============================================================================
//Global variables

DroneSim *g_drone = NULL;

CopterPosSim *g_copterPosSim = NULL;

//==============================================================================
// Copter interface implementation / DJI Matrice
// Basic commands (takeoff, land, speed control) and provide status information

namespace mavfcs {

CopterInterface::CopterInterface(ros::NodeHandle & nh) {

  _is_initialized = false;

  g_drone = new DroneSim();
  
  g_copterPosSim = new CopterPosSim();
}

CopterInterface::~CopterInterface(void) {
  if (_is_initialized) {
    ROS_INFO_STREAM("[CopterInterface - DJI] Releasing SDK permission");
    g_drone->release_sdk_permission_control();
  }
  delete g_drone;
}


int CopterInterface::initialize(void) {
  if (_is_initialized) return -1;
  ROS_INFO_STREAM("[CopterInterface - DJI] Requesting SDK permission");
  g_drone->request_sdk_permission_control();
  _is_initialized = true;
  
  return 0;
}

void CopterInterface::terminate(void) {
  if (_is_initialized) {
    ROS_INFO_STREAM("[CopterInterface - DJI] Releasing SDK permission");
    g_drone->release_sdk_permission_control();
  }
}

bool CopterInterface::isEngaged(void) {
  if (!g_drone->activation) {
    g_drone->request_sdk_permission_control();
  }
  if (g_drone->activation) {
    return true;
  } 
  return false;
}

FlightState CopterInterface::getFlightState(void) {
  unsigned int state = g_drone->flight_status;
  //ROS_INFO_STREAM_THROTTLE(1.0, "[CopterInterface - DJI] Flight State=" << state);
  
  return (FlightState)state;
}

int CopterInterface::takeOff(void) {
  ROS_INFO_STREAM("[CopterInterface - DJI] TakeOff");
  g_drone->takeoff();
  return 0;
}

int CopterInterface::land(void) {
  ROS_INFO_STREAM("[CopterInterface - DJI] Land");
  g_drone->landing();
  return 0;
}

int CopterInterface::setVelocityCommand(double vx, double vy, double vz, double yaw) {
  g_drone->attitude_control(vx, vy, vz, yaw);
  return 0;
}

nav_msgs::Odometry CopterInterface::getOdometry(void) {

  g_drone->update_position();

  nav_msgs::Odometry odom;
  //odom = g_drone->odometry;
  //@TODO: investigate why this was crashing the application
  odom.header.stamp = g_drone->odometry.header.stamp;
  odom.header.seq = g_drone->odometry.header.seq;
  odom.header.frame_id = "/world";
  odom.child_frame_id = "/drone";
  odom.pose = g_drone->odometry.pose;
  odom.twist = g_drone->odometry.twist;
  //Altitude is Z down - DJI has Z up
  //odom.pose.pose.position.z = -1.0 * odom.pose.pose.position.z;
  //odom.twist.twist.linear.z = -1.0 * odom.twist.twist.linear.z;
  //@TODO: need to check if velocities are correct (world frame or body frame)
  return odom;
}

int CopterInterface::getBatteryLevel(void) {
    return 50;
}

std::string CopterInterface::getStatusDescriptionStr(void) {
  std::string str;
  str = std::string("[CopterInterface] Status: ");
  if (!g_drone->activation) {
    g_drone->request_sdk_permission_control();
  }
  if (g_drone->activation) {
    str += std::string(" Engaged / ");
  }
  else {
    str += std::string(" Not Engaged / ");
  } 
  
  int state = g_drone->flight_status;
  switch (state) {
  case ON_GROUND:  str += std::string("State: ON_GROUND"); break;
  case TAKING_OFF: str += std::string("State: TAKING_OFF"); break;
  case IN_AIR:     str += std::string("State: IN_AIR"); break;
  case LANDING:    str += std::string("State: LANDING"); break;
  default:         str += std::string("State: INVALID"); break;
  }

  return str;
}

}


