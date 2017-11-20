/*
* Copyright (c) 2016 Carnegie Mellon University, Silvio Maeta <smaeta@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/

#ifndef _MAV_FCS_PROCESSOR_H_
#define _MAV_FCS_PROCESSOR_H_

#include <limits>
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <ca_nav_msgs/PathXYZVPsi.h>

#include "mav_fcs/path_tracking_control.h"
#include "mav_fcs/speed_control.h"
#include "mav_fcs/fcs_interface.h"
#include "mav_fcs/fcs_state_machine.h"
#include "mav_fcs/copter_interface.h"

namespace mavfcs {

class FcsProcessor
{
public:
  FcsProcessor(ros::NodeHandle & traj_controller_nh, ros::NodeHandle & copter_nh, FcsInterface *fcsint);

  ~FcsProcessor(void);

  void setPeriod(double dt) { _period = dt; }
  
  int initialize(void);

  void terminate(void);

  void update(void);
  
  void updateCopterInterface(void);

private:
  tf::TransformListener _tf_listener;

  FcsInterface *_fcs_interface;

  InputFSM _fsm_data;
  FcsStateMachine *_state_machine;

  CopterInterface *_copter_interface;
  
  //----------------------------------------------------------------------------
  // Trajectory control parameters and methods
  double _period;
  PathTrackingControlParameters  _controller_parameters;
  PathTrackingControl *_controller;
  PathTrackingControl::PathTrackingControlState _controller_state;

  //Speed control parameters
  SpeedControlParameters _speed_control_params;
 
  unsigned int _user_cmd;
  unsigned int _last_event;

  bool _has_valid_traj;
  bool _follow_traj;
  bool _traj_completed;
  //bool _has_landing_point;
  Eigen::Vector3d _path_end_point;
  //small_copter_msgs::Trajectory _curr_traj_msg;
  //NEA::small_copter_interface::Trajectory *_curr_traj;
  PathXYZVPsi _wp_path;
  double _traj_start_time;
  //int _command_spline_type;
  //int _command_spline_index;
  double _waypoint_index;
  VelAccPsi _speed_cmd;
 
  double _spline_sampling_period;
 
  // Trajectory control support methods
  void initializeTrajectoryControl(ros::NodeHandle & traj_controller_nh);
 
  void trajectoryUpdate(void);
  
  void generateSpeedControlCommand(void);

  bool isInsideCaptureRadius(void);

  void setNewTrajectory(ca_nav_msgs::PathXYZVPsi path);
  
  float getCurrentTrajectoryTime(void);
  
  mav_gcs_msgs::FCSStatus getFcsStatus(void);

};

}
#endif /* _MAV_FCS_PROCESSOR_H_ */
