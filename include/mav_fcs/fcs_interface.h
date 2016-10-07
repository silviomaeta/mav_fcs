/**
 * @file   fcs_interface.cpp
 * @author Silvio Maeta
 * @date   04/24/2016
 * @brief  
 *
 * @copyright
 * Copyright (C) 2016.
 */

#ifndef _MAV_FCS_INTERFACE_H_
#define _MAV_FCS_INTERFACE_H_

#include <limits>
#include <math.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ca_nav_msgs/PathXYZVPsi.h>
#include <mav_gcs_msgs/FCSStatus.h>
#include <mav_gcs_msgs/UserCmd.h>

#include "mav_fcs/path_tracking_control.h"


namespace tf
{
  typedef boost::shared_ptr<TransformListener> TransformListenerPtr;
  typedef boost::shared_ptr<Transformer> TransformerPtr;
  typedef boost::shared_ptr<StampedTransform> StampedTransformPtr;
}


class FcsInterface
{
public:
  FcsInterface(const ros::NodeHandle & nh, const ros::Time & stamp, bool isSim);



  void setOdometry(const nav_msgs::Odometry::ConstPtr odom_msg)
  {
    _odometry_msg = *odom_msg;
    _has_odometry_msg = true;
  }

  nav_msgs::Odometry getOdometry(void)
  {
    _has_odometry_msg = false;
    return _odometry_msg;
  }

  bool hasOdometry(void) { return _has_odometry_msg; }



  void setPath(const ca_nav_msgs::PathXYZVPsi::ConstPtr path_msg)
  {
    ROS_ERROR_STREAM("Path RECV!");
    _path_msg = *path_msg;
    _has_path_msg = true;
  }

  ca_nav_msgs::PathXYZVPsi getPath(void)
  {
    _has_path_msg = false;
    return _path_msg;
  }

  bool hasPath(void) { return _has_path_msg; }



  void setUserCmd(const mav_gcs_msgs::UserCmd::ConstPtr cmd_msg)
  {
    ROS_ERROR_STREAM("UserCmd RECV!");
    _user_cmd_msg = *cmd_msg;
    _has_user_cmd_msg = true;
  }

  mav_gcs_msgs::UserCmd getUserCmd(void)
  {
    _has_user_cmd_msg = false;
    return _user_cmd_msg;
  }

  bool hasUserCmd(void) { return _has_user_cmd_msg; }



  void setFcsOdometry(nav_msgs::Odometry odom_msg)
  {
    _fcs_odometry_msg = odom_msg;
  }

  nav_msgs::Odometry getFcsOdometry(void)
  {
    return _fcs_odometry_msg;
  }


  void setFcsStatus(mav_gcs_msgs::FCSStatus stt_msg)
  {
    _fcs_status_msg = stt_msg;
  }

  mav_gcs_msgs::FCSStatus getFcsStatus(void)
  {
    return _fcs_status_msg;
  }


private:
  bool _is_simulation;
  
  bool _on_ground;
  float _energy_level;
  float _curr_traj_time;

  nav_msgs::Odometry _odometry_msg;
  bool _has_odometry_msg;

  ca_nav_msgs::PathXYZVPsi _path_msg;
  bool _has_path_msg;

  mav_gcs_msgs::UserCmd _user_cmd_msg;
  bool _has_user_cmd_msg;

  nav_msgs::Odometry _fcs_odometry_msg;

  mav_gcs_msgs::FCSStatus _fcs_status_msg;

  tf::TransformListener _tf_listener;
  ros::Time _init_traj_time;
  tf::StampedTransform _dji_world_tf;
  tf::TransformerPtr transformer_;
    
};

#endif /* _MAV_FCS_INTERFACE_H_ */
