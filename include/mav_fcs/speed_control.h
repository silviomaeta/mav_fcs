/*
* Copyright (c) 2016 Carnegie Mellon University, Silvio Maeta <smaeta@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#ifndef _MAV_FCS_SPEED_CONTROL_H_
#define _MAV_FCS_SPEED_CONTROL_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include "geom_cast/geom_cast.hpp"
#include "geom_cast/rot_cast.hpp"

#include "mav_fcs/path_tracking_control.h"

namespace mavfcs {

using namespace mavfcs;

class SpeedControlParameters {
 public:
  double max_horiz_speed;   // m/s
  double horiz_accel_limit; // m/s^2

  double max_vert_speed;    // m/s
  double vert_accel_limit;  // m/s^2

  double max_yaw_rate;      // rad/s
  
  VelAccPsi prev_cmd;
  bool has_prev_cmd;
  
  SpeedControlParameters():
    max_horiz_speed(1.0),
    horiz_accel_limit(0.25),
    max_vert_speed(0.2),
    vert_accel_limit(0.05),
    max_yaw_rate(0.0),
    has_prev_cmd(false) {
  }

  bool LoadParameters(ros::NodeHandle &n) {
    bool found = true;

    found = found && n.getParam("max_horiz_speed",   max_horiz_speed);
    found = found && n.getParam("horiz_accel_limit", horiz_accel_limit);
    found = found && n.getParam("max_vert_speed",    max_vert_speed);
    found = found && n.getParam("vert_accel_limit",  vert_accel_limit);
    found = found && n.getParam("max_yaw_rate",      max_yaw_rate);

    return found;
  }
};


VelAccPsi ComputeSpeedControl(double dt, const nav_msgs::Odometry &curr_pose, SpeedControlParameters &control_params, VelAccPsi &desired_cmd);

}

#endif /* __MAV_FCS_SPEED_CONTROL_H_ */
