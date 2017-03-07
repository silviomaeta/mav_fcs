/**
 * @file   copter_interface.cpp
 * @author Silvio Maeta
 * @date   04/28/2016
 * @brief  Interface with copter onboard system - modify to support different copter model
 *
 * @copyright
 * Copyright (C) 2016.
 */

#include <limits>
#include <math.h>
#include <iostream>
#include <fstream>
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

//std::ofstream file;
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
  bool activation = _is_initialized;
// rmh 2.3 bool control = g_drone->sdk_permission_opened;
  // New in 3.1. Request API control and check if it was granted
  int control = 1;
  if (!activation) {
    control = g_drone->request_sdk_permission_control();
  }
  PAFSwitch paf_switch = (PAFSwitch) g_drone->rc_channels.mode;
  if (activation) {
    if (control) {
      //ROS_INFO_STREAM("Initialized and Got control");
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
  //ROS_INFO_STREAM("[CopterInterface - DJI] Start Recording Video");
  //g_drone->start_video();

  //file.open("/home/ubuntu/dji_ws/src/mav_fcs/video_ts.txt");
  //file << "video start-end timestamp:\t" << std::setprecision(15) << ros::Time::now().toSec();
  return 0;
}

int CopterInterface::land(void) {
  ROS_INFO_STREAM("[CopterInterface - DJI] Land");
  g_drone->landing();
  //ROS_INFO_STREAM("[CopterInterface - DJI] Stop Recording Video");
  //g_drone->stop_video(); 
  //file << "\t" << std::setprecision(15) << ros::Time::now().toSec() << "\n";
  //file.close();
  return 0;
}

int CopterInterface::startVideo(void) {
  ROS_INFO_STREAM("[CopterInterface - DJI] Start Recording Video");
  g_drone->start_video();
  return 0;
}

int CopterInterface::stopVideo(void) {
  ROS_INFO_STREAM("[CopterInterface - DJI] Stop Recording Video");
  g_drone->stop_video();
  return 0;
}

int CopterInterface::setVelocityCommand(double vx, double vy, double vz, double yawrate) {
  g_drone->attitude_control(HORIZ_ATT|VERT_VEL|YAW_RATE|HORIZ_BODY|YAW_BODY, 
                            vx, vy, -vz, yawrate);
  return 0;
}

int CopterInterface::setGimbalCommand(double r_rate, double p_rate, double y_rate) {
  g_drone->gimbal_speed_control((int)r_rate, (int)p_rate, (int)y_rate);
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
  
  bool active = _is_initialized;
  // rmh 2.3 int control = g_drone->sdk_permission_opened;
  // New in 3.1. Request API control and check if it was granted
  int control = 1;
  if (!active) {
    control = g_drone->request_sdk_permission_control();
  }
  ROS_INFO_STREAM_THROTTLE(5.0, "[CopterInterface] PAF switch=" << g_drone->rc_channels.mode);
  PAFSwitch paf_switch = (PAFSwitch) g_drone->rc_channels.mode;
  if ((active) && (control == 1) && ((paf_switch == PAFSwitch::FUNCTION) || (paf_switch > 0))) {
    str += std::string(" Engaged /");
  }
  else {
    str += std::string(" Not Engaged (");
    if (active != 1) {
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
  
  /*int control = g_drone->request_sdk_permission_control();

  ROS_INFO_STREAM_THROTTLE(5.0, "[CopterInterface] PAF switch=" << g_drone->rc_channels.mode);
  PAFSwitch paf_switch = (PAFSwitch) g_drone->rc_channels.mode;
  if ((control == 1) && ((paf_switch == PAFSwitch::FUNCTION) || (paf_switch > 0))) {
    str += std::string(" Engaged /");
  }
  else {
    str += std::string(" Not Engaged (");
    if (control != 1) {
      str += std::string(" NoControl ");
    }
    if (paf_switch != PAFSwitch::FUNCTION) {
      str += std::string(" SwitchNotInFunctionMode ");
    }
    str += std::string(") / ");
  } */
  
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


