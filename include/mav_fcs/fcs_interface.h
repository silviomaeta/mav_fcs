/*
* Copyright (c) 2016 Carnegie Mellon University, Silvio Maeta <smaeta@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
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

/*
  void setTrajectory(const small_copter_msgs::Trajectory::ConstPtr traj_msg)
  {
    if (_is_simulation)
    {
      //@TODO: remove this when using real trajectory publisher (Mission Executive)
      //Need to set command start time because using topic playback
      small_copter_msgs::Trajectory::Ptr updated_traj_msg(
          new small_copter_msgs::Trajectory(*traj_msg));
      updated_traj_msg->command_start_time = ros::Time::now();
      _init_traj_time = ros::Time::now();
      _traj_msg = updated_traj_msg;
    }
    else
    {
      _init_traj_time = traj_msg->command_start_time;
      _traj_msg = traj_msg;
    }
    _has_traj_msg = true;
  }
*/
/*
  void setDjiOdometry(const nav_msgs::Odometry::ConstPtr odom_msg)
  {
    _dji_odometry_msg = *odom_msg;
  }
*/
/*
  void setDjiHome(small_copter_msgs::LatLongAlt home_msg)
  {
    _dji_home_msg = home_msg;
  }

  void setDjiGlobalPosition(small_copter_msgs::LatLongAlt global_pos_msg)
  {
    _dji_global_pose_msg = global_pos_msg;
  }

  void updateDjiOdometry(nav_msgs::Odometry odom_msg);

  small_copter_msgs::Trajectory::ConstPtr getTrajectory()
  {
    return _traj_msg;
  }

  double getCommandTime(const ros::Time & stamp)
  {
    NEA::small_copter_interface::Trajectory traj(*_traj_msg);
    return traj.relativeTime(stamp, 0.0);
  }

  ros::Time getTrajectoryInitTime(void)
  {
    return _init_traj_time;
  }

  void update(const ros::Time & stamp);


  small_copter_msgs::FCSStatus getStatus() const
  {
    return _status_msg;
  }

  nav_msgs::Odometry getDjiOdometry() const
  {
    return _dji_odometry_msg;
  }

  small_copter_msgs::LatLongAlt getDjiHome() const
  {
    return _dji_home_msg;
  }

  nav_msgs::Odometry getOdometryIns() const
  {
    return _ins_odometry_msg;
  }

  tf::StampedTransform getDjiWorldTransform() const
  {
    return _dji_world_tf;
  }

  tf::StampedTransform getDjiOdometryTransform() const
  {
    tf::StampedTransform transform;
    tf::poseMsgToTF(_dji_odometry_msg.pose.pose, transform);
    transform.stamp_    = _dji_odometry_msg.header.stamp;
    transform.frame_id_ = _dji_odometry_msg.header.frame_id;
    transform.child_frame_id_ = _dji_odometry_msg.child_frame_id;
    return transform;
  }

  void setOnGround(bool flag)
  {
    _on_ground = flag;
  }
 
  void setEnergyLevel(float level) 
  {
    _energy_level = level;
  }

  void setTrajectoryTime(float t) 
  {
    _curr_traj_time = t;
  }
*/

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

  std::string _traj_header_frame_id;
  std::string _traj_child_frame_id;
  std::string _dji_header_frame_id;
  std::string _dji_child_frame_id;
  std::string _ideal_dji_child_frame_id;
  std::string _imu_world_frame_id;
  std::string _imu_frame_id;
  //small_copter_msgs::Trajectory::ConstPtr _traj_msg;
  bool _has_traj_msg;
  //small_copter_msgs::FCSStatus _status_msg;
  nav_msgs::Odometry _dji_world_msg;
  //small_copter_msgs::LatLongAlt _dji_home_msg;
  //small_copter_msgs::LatLongAlt _dji_global_pose_msg;
  nav_msgs::Odometry _ins_odometry_msg;
  tf::TransformListener _tf_listener;
  ros::Time _init_traj_time;
  tf::StampedTransform _dji_world_tf;
  tf::TransformerPtr transformer_;

  int _utm_zone;
  bool _is_northern_hemisphere;
  //nea_geographic::UTMConfig::Ptr _utm_config;
    
};

#endif /* _MAV_FCS_INTERFACE_H_ */
