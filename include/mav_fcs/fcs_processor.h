/**
 * @file   fcs_processor.cpp
 * @author Silvio Maeta
 * @date   04/27/2016
 * @brief  
 *
 * @copyright
 * Copyright (C) 2016.
 */

#ifndef _MAV_FCS_PROCESSOR_H_
#define _MAV_FCS_PROCESSOR_H_

#include <limits>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <ca_nav_msgs/PathXYZVPsi.h>

#include "mav_fcs/inspect_ctrl.h"
#include "mav_fcs/fcs_interface.h"
#include "mav_fcs/fcs_state_machine.h"
#include "mav_fcs/copter_interface.h"


namespace mavfcs {

class FcsProcessor
{
public:
  FcsProcessor(ros::NodeHandle & traj_controller_nh, ros::NodeHandle & copter_nh, FcsInterface *fcsint);

  ~FcsProcessor(void);
  
  int initialize(void);

  void terminate(void);

  void update(void);
  
  void updateCopterInterface(void);

  visualization_msgs::MarkerArray getPathVisualization(void);

private:
  tf::TransformListener _tf_listener;

  FcsInterface *_fcs_interface;

  CopterInterface *_copter_interface;

  mav_gcs_msgs::FCSStatus getFcsStatus(void);
  
  //----------------------------------------------------------------------------
  // Trajectory control parameters and methods
 
  unsigned int _user_cmd;
  unsigned int _last_event;

  bool _has_valid_traj;
  bool _follow_traj;
  bool _traj_completed;
  Eigen::Vector3d _path_end_point;

  std::vector<geometry_msgs::PoseStamped> _waypoints;
  int _waypoint_index;

  geometry_msgs::PoseStamped _hoverpoint;
  bool _has_hoverpoint;

  geometry_msgs::PoseStamped _landpoint;
  bool _has_landpoint;

  bool _has_pause_point;
  geometry_msgs::PoseStamped _pause_point;
  double _pause_interval;
  double _pause_start_time;
  
  double _max_distance_waypoints;

  DjiInspectCtrl *_inspect_ctrl;
  
  double _target_capture_radius;
 
  void initializeTrajectoryControl(ros::NodeHandle & traj_controller_nh);
  
  void updateWaypointIndex(void);
  
  bool isLastWaypoint(void);

  void setNewTrajectory(ca_nav_msgs::PathXYZVPsi path);
    
  void trajectoryUpdate(void);
  
  void sendCmdCopter(void);

};

}
#endif /* _MAV_FCS_PROCESSOR_H_ */
