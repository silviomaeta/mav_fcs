/*
 * speed_control.cpp
 * Author: Carnegie Mellon University / Silvio Maeta
 */

#define _USE_MATH_DEFINES

#include <math.h>
#include <vector>

#include "mav_fcs/speed_control.h"

namespace mavfcs {

double limitToPi(double angle) {
  double val = fmod(angle, 2*M_PI);
  if (val > M_PI) {
    val -= 2*M_PI;
  }
  if (val < -M_PI) {
    val += 2*M_PI;
  }
  return val;
}

double diffAngles(double a, double b) {
    a = limitToPi(a);
    b = limitToPi(b);
    double diff = b - a;
    return limitToPi(diff);
}


VelAccPsi ComputeSpeedControl(double dt, const nav_msgs::Odometry &curr_pose, SpeedControlParameters &control_params, VelAccPsi &desired_cmd) {
  OdometryEuler odom_euler;
  odom_euler.SetMsg(curr_pose);
  double cmd_vx, cmd_vy, cmd_vz, cmd_yaw;
  
  //If doesn't have prev cmd, then use current pose information
  if (!control_params.has_prev_cmd) {
    control_params.prev_cmd.velocity[0] = odom_euler.linear_velocity[0];
    control_params.prev_cmd.velocity[1] = odom_euler.linear_velocity[1];
    control_params.prev_cmd.velocity[2] = odom_euler.linear_velocity[2];
    control_params.prev_cmd.heading = odom_euler.orientation[2];
    control_params.has_prev_cmd = true;
  }
  
  //ROS_INFO_STREAM_THROTTLE(1.0, "[ComputeSpeedControl] Accel Limit: horiz=" << control_params.horiz_accel_limit << " / vert=" << control_params.vert_accel_limit << " / dt=" << dt);
  //----------------------------------------------------------------------------
  //Step 1) Update speed command based on target speed and previous commanded speed
  double prev_vx = control_params.prev_cmd.velocity[0];
  double prev_vy = control_params.prev_cmd.velocity[1];
  double prev_vz = control_params.prev_cmd.velocity[2];

  //ROS_INFO_STREAM_THROTTLE(1.0, "[ComputeSpeedControl] Previous Velocity: vx=" << prev_vx << " / vy=" << prev_vy << " / vz=" << prev_vz);

  double target_vx = desired_cmd.velocity[0];
  double target_vy = desired_cmd.velocity[1];
  double target_vz = desired_cmd.velocity[2];

  //ROS_INFO_STREAM_THROTTLE(1.0, "[ComputeSpeedControl] Target Velocity: vx=" << target_vx << " / vy=" << target_vy << " / vz=" << target_vz);

  //Max horizontal and vertical speeds increments by time step
  double hv_inc = fabs(control_params.horiz_accel_limit * dt);
  double vz_inc = fabs(control_params.vert_accel_limit * dt);
  //ROS_INFO_STREAM_THROTTLE(1.0, "[ComputeSpeedControl] Speed Increment: horiz=" << hv_inc << " / vert=" << vz_inc);
  
  //Set horizontal speed
  double delta_vx = target_vx - prev_vx;
  double delta_vy = target_vy - prev_vy;
  double delta_hv = sqrt(delta_vx*delta_vx + delta_vy*delta_vy);
  if (hv_inc > delta_hv) hv_inc = delta_hv;
  double angle = atan2(delta_vy, delta_vx);
  double vx_inc = hv_inc * cos(angle);
  double vy_inc = hv_inc * sin(angle);
  cmd_vx = prev_vx + vx_inc;
  cmd_vy = prev_vy + vy_inc;
  if (sqrt(cmd_vx*cmd_vx + cmd_vy*cmd_vy) > control_params.max_horiz_speed) {
    angle = atan2(cmd_vy, cmd_vx);
    cmd_vx = control_params.max_horiz_speed * cos(angle);
    cmd_vy = control_params.max_horiz_speed * sin(angle);
  }
  
  //Set vertical speed
  double delta_vz = fabs(target_vz - prev_vz);
  if (delta_vz < vz_inc) vz_inc = delta_vz;
  if (target_vz > prev_vz) {
    cmd_vz = prev_vz + vz_inc;
  }
  else {
    cmd_vz = prev_vz - vz_inc;
  }
  if (cmd_vz > control_params.max_vert_speed) {
    cmd_vz = control_params.max_vert_speed;
  }
  if (cmd_vz < -control_params.max_vert_speed) {
    cmd_vz = -control_params.max_vert_speed;
  }
  
  //ROS_INFO_STREAM_THROTTLE(1.0, "[ComputeSpeedControl] Intermediate Velocity: vx=" << cmd_vx << " / vy=" << cmd_vy << " / vz=" << cmd_vz);
  
  //----------------------------------------------------------------------------
  //Step 2) Use current aircraft speed to limit commanded speed  
  double curr_vx = odom_euler.linear_velocity[0];
  double curr_vy = odom_euler.linear_velocity[1];
  double curr_vz = odom_euler.linear_velocity[2];

  double max_hv_diff = fabs(control_params.horiz_accel_limit * 1.0); // one second?
  double max_vz_diff = fabs(control_params.vert_accel_limit * 2.0); // two seconds?

  //Set horizontal speed
  //Limit horizontal speed difference
  delta_vx = cmd_vx - curr_vx;
  delta_vy = cmd_vy - curr_vy;
  if (sqrt(delta_vx*delta_vx + delta_vy*delta_vy) > max_hv_diff) {
    angle = atan2(delta_vy, delta_vx);
    cmd_vx = curr_vx + max_hv_diff * cos(angle);
    cmd_vy = curr_vy + max_hv_diff * sin(angle);
  }
  //Limit max commanded horizontal speed
  if (sqrt(cmd_vx*cmd_vx + cmd_vy*cmd_vy) > control_params.max_horiz_speed) {
    angle = atan2(cmd_vy, cmd_vx);
    cmd_vx = control_params.max_horiz_speed * cos(angle);
    cmd_vy = control_params.max_horiz_speed * sin(angle);
  }
    
  //Set vertical speed
  delta_vz = fabs(cmd_vz - curr_vz);
  if (delta_vz > max_vz_diff) delta_vz = max_vz_diff;
  //Does not allow to differ too much from the current speed
  if (cmd_vz > curr_vz) {
    cmd_vz = curr_vz + delta_vz;
  }
  else {
    cmd_vz = curr_vz - delta_vz;
  }
  //Limit max vertical speed
  if (cmd_vz > control_params.max_vert_speed) {
    cmd_vz = control_params.max_vert_speed;
  }
  if (cmd_vz < -control_params.max_vert_speed) {
    cmd_vz = -control_params.max_vert_speed;
  }

  //----------------------------------------------------------------------------
  //Step 3) Set yaw - use just current yaw and target yaw values
  double curr_yaw   = odom_euler.orientation[2];
  double prev_yaw   = control_params.prev_cmd.heading;
  double target_yaw = desired_cmd.heading;
  double yaw_inc = fabs(control_params.max_yaw_rate * dt);
  double max_yaw_diff = fabs(control_params.max_yaw_rate * 1.0); // one second?

  //Limit commanded yaw increment - max yaw rate
  double delta_yaw = diffAngles(prev_yaw, target_yaw);
  if (fabs(delta_yaw) < yaw_inc) yaw_inc = fabs(delta_yaw);
  if (delta_yaw < 0.0) yaw_inc = -yaw_inc;
  cmd_yaw = limitToPi(prev_yaw + yaw_inc);

  //Limit commanded yaw based on current pose
  delta_yaw = diffAngles(curr_yaw, cmd_yaw);
  if (fabs(delta_yaw) > max_yaw_diff) {
    if (delta_yaw > 0.0) delta_yaw =  max_yaw_diff;
    else                 delta_yaw = -max_yaw_diff;
    cmd_yaw = limitToPi(curr_yaw + delta_yaw);
  }

  //ROS_INFO_STREAM_THROTTLE(1.0, "[ComputeSpeedControl] Final Velocity: vx=" << cmd_vx << " / vy=" << cmd_vy << " / vz=" << cmd_vz);

  //----------------------------------------------------------------------------
  //Update prev cmd for next iteration
  control_params.prev_cmd.velocity[0] = cmd_vx;
  control_params.prev_cmd.velocity[1] = cmd_vy;
  control_params.prev_cmd.velocity[2] = cmd_vz;
  control_params.prev_cmd.heading     = cmd_yaw;

  //Return speed command
  return control_params.prev_cmd;
}

}

