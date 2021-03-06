/*
* Copyright (c) 2016 Carnegie Mellon University, Silvio Maeta <smaeta@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


#include <limits>
#include <math.h>

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <dji_sdk/dji_drone.h>
//#include <dji_sdk_lib/DJI_Flight.h>

#include "mav_fcs/copter_interface.h"

using namespace mavfcs;

//==============================================================================
//Auxiliar functions - flat earth conversions
//Input lat/long angles in radians

const static double r_earth = 180*60*1852/3.1415926535897932384626433832795;

double deg2rad(double deg) {
    return ((deg * 3.1415926535897932384626433832795) / 180.0);
}

double rad2deg(double rad) {
    return ((rad * 180.0) / 3.1415926535897932384626433832795);
}


//==============================================================================

/// Describe the PAF (Position Attitude Function) switch on the DJI.
/// These are the same as the numbers the DJI SDK publishes.
enum PAFSwitch
{
    /// Copter in Position mode.
    POSITION = -10000,
    /// Copter in Attitude mode.
    ATTITUDE = 0,
    /// Copter in SDK Control mode.
    FUNCTION = 10000
};

//==============================================================================
//Global variables

DJIDrone *g_drone = NULL;

//==============================================================================
// Copter interface implementation / DJI Matrice
// Basic commands (takeoff, land, speed control) and provide status information

namespace mavfcs {

CopterInterface::CopterInterface(ros::NodeHandle & nh) {

  _is_initialized = false;

  g_drone = new DJIDrone(nh);
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
  bool activation = g_drone->activation;
// rmh 2.3 bool control = g_drone->sdk_permission_opened;
  // New in 3.1. Request API control and check if it was granted
  int control = 1;
  if (!activation) {
    control = g_drone->request_sdk_permission_control();
  }
  PAFSwitch paf_switch = (PAFSwitch) g_drone->rc_channels.mode;
  if (activation) {
    if (control) {
      if (paf_switch == PAFSwitch::FUNCTION) {
        return true;
      }
    }
  } 
  return false;
}

FlightState CopterInterface::getFlightState(void) {
  unsigned int state = g_drone->flight_status;
  //ROS_INFO_STREAM_THROTTLE(1.0, "[CopterInterface - DJI] Flight State=" << state);
  
  switch (state) {
  case 1: return ON_GROUND; break;
  case 2: return TAKING_OFF; break;
  case 3: return IN_AIR; break;
  case 4: return LANDING; break;
  case 5: return ON_GROUND; break;
  default: return INVALID; break;
  }
  return INVALID;
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
  g_drone->attitude_control(HORIZ_VEL|VERT_VEL|YAW_ANG|HORIZ_GND|YAW_GND, 
                            vx, vy, -vz, yaw*180.0/M_PI);
  return 0;
}

nav_msgs::Odometry CopterInterface::getOdometry(void) {
  nav_msgs::Odometry odom;
  //odom = g_drone->odometry;
  //@TODO: investigate why this was crashing the application
  odom.header.stamp = g_drone->odometry.header.stamp;
  odom.header.seq = g_drone->odometry.header.seq;
  odom.pose = g_drone->odometry.pose;
  odom.twist = g_drone->odometry.twist;
  //Altitude is Z down - DJI has Z up
  odom.pose.pose.position.z = -1.0 * odom.pose.pose.position.z;
  odom.twist.twist.linear.z = -1.0 * odom.twist.twist.linear.z;
  //@TODO: need to check if velocities are correct (world frame or body frame)
  return odom;
}

int CopterInterface::getBatteryLevel(void) {
    return (int)g_drone->power_status.percentage;
}

std::string CopterInterface::getStatusDescriptionStr(void) {
  std::string str;
  str = std::string("[CopterInterface - DJI] Status: ");
  bool activation = g_drone->activation;
// rmh 2.3 int control = g_drone->sdk_permission_opened;
  // New in 3.1. Request API control and check if it was granted
  int control = 1;
  if (!activation) {
    control = g_drone->request_sdk_permission_control();
  }
  //ROS_INFO_STREAM("PAF switch=" << g_drone->rc_channels.mode);
  PAFSwitch paf_switch = (PAFSwitch) g_drone->rc_channels.mode;
  if ((activation) && (control == 1) && (paf_switch == PAFSwitch::FUNCTION)) {
    str += std::string(" Engaged /");
  }
  else {
    str += std::string(" Not Engaged (");
    if (activation != 1) {
      str += std::string(" ActivationFailed ");
    }
    if (control != 1) {
      str += std::string(" NoControl ");
    }
    if (paf_switch != PAFSwitch::FUNCTION) {
      str += std::string(" SwitchNotInFunctionMode ");
    }
    str += std::string(") / ");
  } 
  
  int state = g_drone->flight_status;
  switch (state) {
  case 1: str += std::string("State: ON_GROUND"); break;
  case 2: str += std::string("State: TAKING_OFF"); break;
  case 3: str += std::string("State: IN_AIR"); break;
  case 4: str += std::string("State: LANDING"); break;
  case 5: str += std::string("State: ON_GROUND"); break;
  default: str += std::string("State: INVALID"); break;
  }

  return str;
}

}


