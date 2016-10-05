/**
 * @file   fcs_node.cpp
 * @author Silvio Maeta
 * @date   04/24/2016
 * @brief  
 *
 * @copyright
 * Copyright (C) 2016.
 * All rights reserved.
 * Near Earth Autonomy.
 * www.nearearth.aero
 */

#define _USE_MATH_DEFINES

#include <limits>
#include <math.h>

#include <atomic>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include "geom_cast/geom_cast.hpp"

#include "mav_fcs/speed_control.h"


using namespace mavfcs;

//==============================================================================

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fcs");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  SpeedControlParameters speedControlParams;
  
  //----------------------------------------------------------------------------
  //Initialization
  if (!speedControlParams.LoadParameters(pnh)) {
    ROS_ERROR_STREAM("Could not load speed control parameters.");
    return -1;
  }  

  double period = 1.0/10.0; // 10 Hz update rate
  VelAccPsi speedCmd, targetCmd;
  nav_msgs::Odometry odom;
  tf::Quaternion quat;

  //----------------------------------------------------------------------------
  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 1 - VX POSITIVE ");

  quat.setRPY(0.0, 0.0, 0.0);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 1.5;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = 0.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 2 - VX NEGATIVE ");

  speedControlParams.has_prev_cmd = false;

  quat.setRPY(0.0, 0.0, 0.0);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = -1.5;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = 0.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 3 - VZ POSITIVE ");

  speedControlParams.has_prev_cmd = false;

  quat.setRPY(0.0, 0.0, 0.0);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 0.0;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.5;
  targetCmd.heading     = 0.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 4 - VZ NEGATIVE ");

  speedControlParams.has_prev_cmd = false;

  quat.setRPY(0.0, 0.0, 0.0);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 0.0;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = -0.5;
  targetCmd.heading     = 0.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  double yaw = 0.0;

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 5 - HEADING ONLY (POSITIVE)");

  speedControlParams.has_prev_cmd = false;

  yaw = 0.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 0.0;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = M_PI/2.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 6 - HEADING ONLY (NEGATIVE)");

  speedControlParams.has_prev_cmd = false;

  yaw = 0.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 0.0;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = -M_PI/2.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 7 - HEADING ONLY (LARGE DIFF)");

  speedControlParams.has_prev_cmd = false;

  yaw = M_PI/2.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 0.0;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = -M_PI/2.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }


  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 8 - HEADING ONLY (LARGE DIFF)");

  speedControlParams.has_prev_cmd = false;

  yaw = -M_PI/2.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 0.0;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = M_PI/2.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 9 - HEADING ONLY (SECTORS)");

  speedControlParams.has_prev_cmd = false;

  yaw = -3.0*M_PI/4.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 0.0;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = 3.0*M_PI/4.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 10 - HEADING ONLY (SECTORS)");

  speedControlParams.has_prev_cmd = false;

  yaw = M_PI/4.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 0.0;
  targetCmd.velocity[1] = 0.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = -M_PI/4.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }


  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 11 - V LIMITS");

  speedControlParams.has_prev_cmd = false;

  yaw = 0.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.5;
  odom.twist.twist.linear.y = 0.5;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 1.0;
  targetCmd.velocity[1] = 1.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = 0.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 12 - V LIMITS");

  speedControlParams.has_prev_cmd = false;

  yaw = 0.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.6;
  odom.twist.twist.linear.y = 0.6;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 1.0;
  targetCmd.velocity[1] = -1.0;
  targetCmd.velocity[2] = 0.0;
  targetCmd.heading     = 0.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }

  ROS_INFO_STREAM("==========================================================");
  ROS_INFO_STREAM("SPEED CONTROL - TEST 13 - COMBINED");

  speedControlParams.has_prev_cmd = false;

  yaw = M_PI/4.0;
  quat.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = ca::rot_cast<geometry_msgs::Quaternion>(quat);

  odom.twist.twist.linear.x = 0.6;
  odom.twist.twist.linear.y = 0.1;
  odom.twist.twist.linear.z = 0.0;

  targetCmd.velocity[0] = 1.0;
  targetCmd.velocity[1] = -0.5;
  targetCmd.velocity[2] = 0.2;
  targetCmd.heading     = M_PI/2.0;

  for (int i=0; i<15; i++) {
    ROS_INFO_STREAM("ODOM: vx=" << odom.twist.twist.linear.x << " / vy=" << odom.twist.twist.linear.y << " / vz=" << odom.twist.twist.linear.z << " / yaw=" << yaw);
    ROS_INFO_STREAM("TRGT: vx=" << targetCmd.velocity[0] << " / vy=" << targetCmd.velocity[1] << " / vz=" << targetCmd.velocity[2] << " / yaw=" << targetCmd.heading);
    speedCmd = ComputeSpeedControl(period, odom, speedControlParams, targetCmd);
    ROS_INFO_STREAM("COMD: vx=" << speedCmd.velocity[0] << " / vy=" << speedCmd.velocity[1] << " / vz=" << speedCmd.velocity[2] << " / yaw=" << speedCmd.heading);
  }


  return 0;
}

