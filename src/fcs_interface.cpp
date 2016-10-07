/**
 * @file   fcs_interface.cpp
 * @author Silvio Maeta
 * @date   04/24/2016
 * @brief  
 *
 * @copyright
 * Copyright (C) 2016.
 */

#include <limits>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "mav_fcs/fcs_interface.h"

using namespace mavfcs;


//==============================================================================
//==============================================================================

  FcsInterface::FcsInterface(const ros::NodeHandle & nh, const ros::Time & stamp, bool isSim)
  {
    bool found = true;
    
    _is_simulation = isSim;
    
    _has_odometry_msg = false;
    _has_path_msg = false;
    _has_user_cmd_msg = false;
      
    _on_ground = true;
    _energy_level = 0.0;
    _curr_traj_time = 0.0;
    
    tf::TransformerPtr transformer(&_tf_listener);
    transformer_ = transformer;
  }


