/*
* Copyright (c) 2016 Carnegie Mellon University, Silvio Maeta <smaeta@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
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
    : _traj_header_frame_id("world")
    , _traj_child_frame_id("pose_estimation")
    , _dji_header_frame_id("dji_world")
    , _dji_child_frame_id("dji_estimate")
    , _ideal_dji_child_frame_id("dji")
    , _imu_world_frame_id("imu_world")
    , _imu_frame_id("imu")
  {
    bool found = true;
    
    _is_simulation = isSim;
    
    _has_odometry_msg = false;
    _has_path_msg = false;
    _has_user_cmd_msg = false;
  
    found = found && nh.getParam("traj_header_frame_id", _traj_header_frame_id);
    found = found && nh.getParam("traj_child_frame_id", _traj_child_frame_id);
    found = found && nh.getParam("dji_header_frame_id", _dji_header_frame_id);
    found = found && nh.getParam("dji_child_frame_id", _dji_child_frame_id);
    found = found && nh.getParam("ideal_dji_child_frame_id", _ideal_dji_child_frame_id);
    found = found && nh.getParam("imu_world_frame_id", _imu_world_frame_id);
    found = found && nh.getParam("imu_frame_id", _imu_frame_id);

    found = found && nh.getParam("utm_zone", _utm_zone);
    found = found && nh.getParam("is_northern_hemisphere", _is_northern_hemisphere);

    //small_copter_msgs::Polynomial noop_spline_msg;
    //noop_spline_msg.duration = std::numeric_limits<double>::max();
    //found = found && nh.getParam("initial_x", noop_spline_msg.x_coefficients[5]);
    //found = found && nh.getParam("initial_y", noop_spline_msg.y_coefficients[5]);
    //found = found && nh.getParam("initial_z", noop_spline_msg.z_coefficients[5]);
    //found = found && nh.getParam("initial_yaw", noop_spline_msg.yaw_coefficients[5]);

    if (not found) {
        ROS_ERROR_STREAM("[FcsInterface] configuration parameter not found. Check launch file.");
        return;
    }
    /*
    //Initialize coordinate conversion config
    _utm_config = nea_geographic::UTMConfig::Ptr(new nea_geographic::UTMConfig(_utm_zone, _is_northern_hemisphere));

    //Set inital trajectory message
    small_copter_msgs::Trajectory::Ptr init_traj_msg(
        new small_copter_msgs::Trajectory());
    init_traj_msg->header.frame_id = _traj_header_frame_id;
    init_traj_msg->header.stamp = stamp;
    init_traj_msg->child_frame_id = _traj_child_frame_id;
    init_traj_msg->id.id = 0;
    init_traj_msg->command_start_time = stamp;

    small_copter_msgs::Command noop_command_msg;
    noop_command_msg.type = small_copter_msgs::Command::NOOP;
    noop_command_msg.spline.push_back(noop_spline_msg);

    init_traj_msg->commands.push_back(noop_command_msg);
    
    _traj_msg = init_traj_msg;
    _has_traj_msg = false;
    */
    
    _on_ground = true;
    _energy_level = 0.0;
    _curr_traj_time = 0.0;
    
    tf::TransformerPtr transformer(&_tf_listener);
    transformer_ = transformer;
  }

/*
  void FcsInterface::update(const ros::Time & stamp)
  {
    small_copter_msgs::Trajectory traj_msg = *_traj_msg;
    NEA::small_copter_interface::Trajectory traj(traj_msg);

    if (traj_msg.header.frame_id != _traj_header_frame_id)
    {
      throw std::runtime_error(NEA::sf(
          "[FcsInterface] Trajectory header frame '%s' is required to be '%s'.",
          traj_msg.header.frame_id.c_str(), _traj_header_frame_id.c_str()));
    }
    if (traj_msg.child_frame_id != _traj_child_frame_id)
    {
      throw std::runtime_error(NEA::sf(
          "[FcsInterface] Trajectory child frame '%s' is required to be '%s'.",
          traj_msg.child_frame_id.c_str(), _traj_child_frame_id.c_str()));
    }

    //@TODO need to check if update is correct
    //Update FCS status message     
    _status_msg.header.stamp = stamp;
    _status_msg.header.frame_id = traj_msg.header.frame_id;
    if (_on_ground)
    {
      _status_msg.flight_state.state = small_copter_msgs::FCSFlightState::ON_GROUND;
    }
    else
    {
      _status_msg.flight_state.state = small_copter_msgs::FCSFlightState::IN_FLIGHT;
    }
    _status_msg.execution_mode.mode = small_copter_msgs::FCSExecutionMode::EXECUTING_TRAJECTORY;
    _status_msg.trajectory_status.header.stamp = stamp;
    _status_msg.trajectory_status.header.frame_id = traj_msg.header.frame_id;
    _status_msg.trajectory_status.id = traj_msg.id;
    _status_msg.trajectory_status.command_time = _curr_traj_time;
    _status_msg.fuel_status.fuel_level = ((float)_energy_level) / 100.0;
    _status_msg.fuel_status.critical_fuel_level = 0.5;
    _status_msg.fuel_status.flight_time_remaining = 600.0;

    //ROS_ERROR_STREAM_THROTTLE(1.0, "FCS status - command_time=" << _curr_traj_time);
  }
*/
/*
  void FcsInterface::updateDjiOdometry(nav_msgs::Odometry odom_msg)
  {
    //ROS_WARN_STREAM_THROTTLE(1.0, "DJI Home: lat=" << _dji_home_msg.latitude << " / long=" << _dji_home_msg.longitude << " / alt=" << _dji_home_msg.altitude);
    //ROS_WARN_STREAM_THROTTLE(1.0, "DJI Global: lat=" << _dji_global_pose_msg.latitude << " / long=" << _dji_global_pose_msg.longitude << " / alt=" << _dji_global_pose_msg.altitude);

    //Set DJI world frame
    nea_geographic::LTPConfig ltp_world(_dji_home_msg.latitude, _dji_home_msg.longitude);
    Eigen::Isometry3d T_ecef_ltp_world;
    T_ecef_ltp_world = ltp_world.getECEFFromLTP();
    tf::toTf(T_ecef_ltp_world, _dji_world_tf);
    _dji_world_tf.stamp_    = odom_msg.header.stamp;
    _dji_world_tf.frame_id_ = "ecef";
    _dji_world_tf.child_frame_id_ = "dji_world";


    //Calculate position in DJI world frame (current pose is ECEF)
    nea_geographic::LTPConfig ltp(_dji_global_pose_msg.latitude, _dji_global_pose_msg.longitude, _dji_global_pose_msg.altitude);
    Eigen::Isometry3d T_ecef_ltp;
    T_ecef_ltp = ltp.getECEFFromLTP();

    tf::StampedTransform T_aux;
      
    tf::toTf(T_ecef_ltp, T_aux);
    tf::poseTFToMsg(T_aux, _dji_odometry_msg.pose.pose);

    //Update odometry
    _dji_odometry_msg.header = odom_msg.header;
    _dji_odometry_msg.header.frame_id = "ecef";
    _dji_odometry_msg.child_frame_id = "dji_estimate";

    _dji_odometry_msg.twist.twist = odom_msg.twist.twist;

    //Convert from ecef frame to DJI world frame
    nav_msgs::Odometry::Ptr transformed_odom_msg(new nav_msgs::Odometry());
    // The following use of ros::Time(0) assumes all required transformations are static.
    transformOdometry(
            _dji_odometry_msg,
            *transformed_odom_msg,
            "dji_world",
            "dji_estimate",
            ros::Time(0),
            *transformer_);

    //ROS_WARN_STREAM_THROTTLE(1.0, "Transformed Odometry: x=" << (*transformed_odom_msg).pose.pose.position.x << 
    //                                                 " / y=" << (*transformed_odom_msg).pose.pose.position.y <<
    //                                                 " / z=" << (*transformed_odom_msg).pose.pose.position.z);


    if (_is_simulation) {
      
      //Use DJI position in ECEF frame
      _ins_odometry_msg.header = odom_msg.header;
      _ins_odometry_msg.header.frame_id = "ecef";
      _ins_odometry_msg.child_frame_id = "span";

      _ins_odometry_msg.pose.pose   = _dji_odometry_msg.pose.pose;
      _ins_odometry_msg.twist.twist = _dji_odometry_msg.twist.twist;
      
    }
    
    //Now update to DJI world frame
    _dji_odometry_msg = (*transformed_odom_msg);

  }
*/


